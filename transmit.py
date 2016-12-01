from __future__ import print_function
import random
import serial, serial.tools.list_ports
import sys
import time
if len(sys.argv) < 2:
    print ("Usage: transmit.py port")
    print ("Available ports:")
    for port in serial.tools.list_ports.comports():
        print (port)
    sys.exit(0)
automode = False
delay = 1
if len(sys.argv) > 2:
    automode = True
    word_file = "words.txt"
    WORDS = open(word_file).read().splitlines()
if len(sys.argv) > 3:
    delay = float(sys.argv[3])

set_port = sys.argv[1]
ser = serial.Serial()
ser.port = set_port
ser.baudrate=115200
ser.open()
while True:
	try:
            if automode:
                word = random.choice(WORDS).encode('ascii')
                print (word + '\n')
                ser.write(word + '\n')
                time.sleep(delay)
            else:
	        input_string = raw_input("Please enter message: ")
	        ser.write(input_string.encode('ascii') + '\n')
	except KeyboardInterrupt:
	    ser.close()
	    sys.exit(0)
