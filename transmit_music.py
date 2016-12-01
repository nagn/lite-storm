from __future__ import print_function
import random
import serial, serial.tools.list_ports
import sys
import time
import base64
if len(sys.argv) < 2:
    print ("Usage: transmit.py port")
    print ("Available ports:")
    for port in serial.tools.list_ports.comports():
        print (port)
    sys.exit(0)

set_port = sys.argv[1]
ser = serial.Serial()
ser.port = set_port
ser.baudrate=115200
ser.open()

def read_in_chunks(file_object, chunk_size=32): #read 31 characters at a time
    """Lazy function (generator) to read a file piece by piece.
    Default chunk size: 1k."""
    while True:
        data = file_object.read(chunk_size)
        if not data:
            break
        yield data


f = open('converted.txt', 'r')
for piece in read_in_chunks(f):
    try:
        print (piece)
        ser.write(piece)
        time.sleep(0.01)
    except KeyboardInterrupt:
        f.close()
        ser.close()
        sys.exit(0)
