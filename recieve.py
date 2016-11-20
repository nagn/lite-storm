import serial, serial.tools.list_ports
import sys
if len(sys.argv) < 2:
    print ("Usage: recieve.py port")
    print ("Available ports:")
    for port in serial.tools.list_ports.comports():
        print (port)
    sys.exit(0)
set_port = sys.argv[1]
ser = serial.Serial()
ser.port = set_port
ser.baudrate=115200
ser.open()
while True:
	try:
	    if ser.inWaiting():
                print ser.readline()[:-1]
        except KeyboardInterrupt:
	    ser.close()
	    sys.exit(0)



