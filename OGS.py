#!/usr/bin/env python3
import serial

OGS1_CMD_WIDTH = bytearray(b'\x12\x01\x02\x00\x00\xC2\xD3\x00')
OGS1_CMD_CONTRAST = bytearray(b'\x12\x01\x02\x00\x00\xC3\xD2\x00')
OGS1_CMD_AMPLITUDE = bytearray(b'\x12\x01\x02\x00\x00\xC4\xD5\x00')
OGS1_CMD_ALL = bytearray(b'\x12\x01\x02\x00\x00\xC0\xD1\x00')
OGS1_CMD_LED_ON = bytearray(b'\x12\x01\x02\x00\x00\xB0\xA1\x00')
OGS1_CMD_LED_OFF = bytearray(b'\x12\x01\x02\x00\x00\xB1\xA0\x00')
OGS1_CMD_LIGHT = bytearray(b'\x12\x01\x02\x00\x00\xD5\xC4\x00')
OGS1_CMD_DARK = bytearray(b'\x12\x01\x02\x00\x00\xD4\xC5\x00')
OGS1_CMD_VALID_PIXEL_READ = bytearray(b'\x11\x00\xCF\x00\x00\xDE\x00\x00')
OGS1_CMD_INVALID_PIXEL_READ = bytearray(b'\x11\x00\xD5\x00\x00\xC4\x00\x00')

OGS2_CMD_WIDTH = bytearray(b'\x22\x01\x02\x00\x00\xC2\xE3\x00')
OGS2_CMD_CONTRAST = bytearray(b'\x22\x01\x02\x00\x00\xC3\xE2\x00')
OGS2_CMD_AMPLITUDE = bytearray(b'\x22\x01\x02\x00\x00\xC4\xE5\x00')
OGS2_CMD_ALL = bytearray(b'\x22\x01\x02\x00\x00\xC0\xE1\x00')
OGS2_CMD_LED_ON = bytearray(b'\x22\x01\x02\x00\x00\xB0\x91\x00')
OGS2_CMD_LED_OFF = bytearray(b'\x22\x01\x02\x00\x00\xB1\x90\x00')
OGS2_CMD_LIGHT = bytearray(b'\x22\x01\x02\x00\x00\xD5\xF4\x00')
OGS2_CMD_DARK = bytearray(b'\x22\x01\x02\x00\x00\xD4\xF5\x00')
OGS2_CMD_VALID_PIXEL_READ = bytearray(b'\x21\x00\xCF\x00\x00\xEE\x00\x00')
OGS2_CMD_INVALID_PIXEL_READ = bytearray(b'\x21\x00\xD5\x00\x00\xF4\x00\x00')


def edge_calculate(data):
    read_left = (data[5] + (data[6] << 8)) / 10  # Left edge Trace subpixel
    read_right = (data[7] + (data[8] << 8)) / 10  # Right edge Trace subpixel
    return read_left, read_right


class OGS:
    def __init__(self, port, baudrate=115200, bytesize=8, parity='O', stopbits=1, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout

    def connect(self):
        self.connection=serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=self.bytesize, parity=self.parity, stopbits=self.stopbits, timeout=self.timeout)
        return self.connection

    def teach_ogs(self,cmd):
        self.teach_cmd_data=cmd
        self.connection.write(self.teach_cmd_data)
        teach_data_read = self.connection.read()
        teach_data_read += self.connection.read(self.connection.inWaiting())
        print(teach_data_read)
        width_response = teach_data_read[5]  # teach width read
        return teach_data_read
    def edge_read(self,cmd):
        self.edge_cmd_data=cmd
        self.connection.write(self.edge_cmd_data)
        edge_data_read = self.connection.read()
        edge_data_read += self.connection.read(self.connection.inWaiting())
        print(edge_data_read)
        data=edge_data_read
        read_left = (data[5] + (data[6] << 8)) / 10  # Left edge Trace subpixel
        read_right = (data[7] + (data[8] << 8)) / 10  # Right edge Trace subpixel
        return read_left, read_right

