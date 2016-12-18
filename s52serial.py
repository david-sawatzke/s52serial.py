#!/usr/bin/env python3
# s52serial.py A programmer for the AT89S52
# Copyright (C) 2016  David Sawatzke
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
import serial
import argparse
import time
device = "hydrabus8"
chip_id = {
    # Atmel
    000: 0x1E,
    # AT89S52
    100: 0x52,
    200: 0x06
}
def parse_hex(filename):
    # WARNING: This doesn't parse everything, it's currently only meant for
    # the output of as31
    i =  open(filename, "r")
    hex_content = i.readlines()
    i.close()
    binary = b''
    for line in hex_content:
        line.strip()
        binary = parse_hex_line(binary, line)
    return binary

def parse_hex_line(binary, line):
    # WARNING: This only works for some files and doesn't check the Checksum.
    # For example it doesn't use the address field
    line = line.strip()
    if not line.startswith(":"):
        return b''
    line = bytes(line, "UTF-8")
    data_len = int(line[1:3],16)
    addr = int(line[3:7],16)
    if not line[7:9] == b'00':
        return binary
    data = line[9:9+(data_len * 2)]
    bin_data = bytes.fromhex(str(data, "UTF-8"))
    return binary + bin_data


class HydraBus:
    def __init__(self, ser):
        # Save serial port
        self.ser = ser
        try:
            self._spi_mode()
        except:
            # If we are still in binary or binary mode
            self.close()
            self._spi_mode
        # Use lowest speed
        ser.write(b'\x60')
        if ser.read(1) != b'\x01':
            raise Exception("Cannot set SPI speed")
        # Use SPI device 2 (as it's half the speed of spi 1)
        if device == "hydrabus8":
            # CHANGE THE DEVICE AFTER UPDATING (as firmware versions after 0.8 change the
            # bit meaning)
            ser.write(b'\x81')
        else:
            ser.write(b'\x81')
        if ser.read(1) != b'\x01':
            raise Exception("Cannot set SPI parameters")
    def _spi_mode(self):
        # Open binary mode
        for i in range(20):
            self.ser.write(b'\x00')
        if b'BBIO1' != self.ser.read(5):
            raise Exception("Could not get into binary mode")
        # Switching to SPI mode
        self.ser.write(b'\x01')
        if b"SPI1" != self.ser.read(4):
            raise Exception("Cannot set SPI mode")
    def transfer_spi(self, wr_data):
        wr_len = len(wr_data)
        if wr_len > 16:
            raise Exception("Data too long")
        if wr_len < 1:
            raise Exception("Data too short")
        self.ser.write(bytes([0x10 | (wr_len - 1)]))
        # If not hydrabus 8
        if (not device == "hydrabus8"):
            ret_byte = self.ser.read(1)
            if b'\x01' != ret_byte:
                print(ret_byte)
                raise Exception("Didn't receive sucess byte")
        self.ser.write(wr_data)
        if device == "hydrabus8":
            ret_byte = self.ser.read(1)
            if b'\x01' != ret_byte:
                print(ret_byte)
                raise Exception("Didn't receive sucess byte")
        return self.ser.read(wr_len)
    def transfer_spi_long(self, wr_data):
        wr_len = len(wr_data)
        i = 0
        rd_data = b''
        while i < wr_len:
            rd_data += self.transfer_spi(wr_data[i:i+16])
            i += 16
        return rd_data
    def cs_low(self):
        self.ser.write(b'\02');
        if ser.read(1) != b'\x01':
            raise Exception("Cannot put cs low")
    def cs_high(self):
        self.ser.write(b'\03');
        if ser.read(1) != b'\x01':
            raise Exception("Cannot put cs low")
    def close(self):
        # Return to main binary mode
        self.ser.write(b'\x00')
        # Reset to console mode
        self.ser.write(b'\x0F\n')
class Programmer:
    def __init__(self, hydrabus):
        self.hydrabus = hydrabus
        # Reset MCU
        self.hydrabus.cs_low()
        time.sleep(0.2)
        self.hydrabus.cs_high()
        time.sleep(0.2)
        # Enter programming mode
        return_val = hydrabus.transfer_spi(b'\xAC\x53\x00\x00')[3]
        if not (0x69 == return_val or 0x00 == return_val):
            print(return_val)
            raise Exception("Couldn't enter programming mode\n Check connections")
    def read_byte(self, addr):
        return self.hydrabus.transfer_spi(bytes([0x20, ((addr & 0xFF00) >> 8),
                                                 addr & 0xFF, 0x00]))[3]

    def write_byte(self, addr,data):
        data = data & 0xFF
        self.hydrabus.transfer_spi(bytes([0x40, ((addr & 0xFF00) >> 8), addr &
                                          0xFF, data]))
        # Just to be sure
        time.sleep(0.001)
    def read_sig_byte(self,addr):
        return self.hydrabus.transfer_spi(bytes([0x28, ((addr & 0xFF00) >> 8),
                                                 addr & 0xFF, 0x00]))[3]
    def read_page(self, addr):
        return self.hydrabus.transfer_spi_long(bytes([0x30, ((addr & 0xFF00) >>
                                                        8)])+ (b'\x00' * 256))[2:]
    def write_page(self,addr,data):
        # Fill up to 256 bytes of data
        data += b'\xFF' * (256 - len(data))
        self.hydrabus.transfer_spi(bytes([0x50, ((addr & 0xFF00) >> 8)]))
        # Each byte is sent seperately, because the at89s52 needs some time to
        # programm a byte
        # The whole transfer chain between pc and the at89s52 is slow enough to not
        # need a seperate delay
        for i in range(256):
            self.hydrabus.transfer_spi(bytes([data[i]]))
    def erase_chip(self):
        self.hydrabus.transfer_spi(b'\xAC\x80\x00\x00')
        time.sleep(0.6)


parser = argparse.ArgumentParser(description="flash AT89S52")
#parser.add_argument("-v", "--verbosity", help="increase output verbosity", action="count", default=0)
parser.add_argument("-V", "--version", help="display the version", action="store_true")
parser.add_argument("-s","--serial_port", help="select the serial Port", default="/dev/ttyACM0")
#parser.add_argument("-b","--serial_baud", help="select the baud rate of the serial port", type=int, default="115200")
#parser.add_argument("mode", help="select the mode", choices=["w","r"])
parser.add_argument('file', help="hex file to flash")
args = parser.parse_args()
if args.version:
    print("V0.1.0")
    quit()
ser = serial.Serial(args.serial_port)
dev = HydraBus(ser)
prg = Programmer(dev)
# If something fails, reset chip
try:
    prg.erase_chip()
    data = parse_hex(args.file)
    data_len = len(data)
    for i in range(0,data_len,256):
        prg.write_page(i,data[i:i+256])
        rd_data = prg.read_page(i)
        # So the comparse won't fail because of excess bytes
        rd_data = rd_data[0:data_len - i]
        if rd_data != data[i:i+256]:
            raise Exception("Verify failed")
except:
    dev.close()
    raise
dev.close()
