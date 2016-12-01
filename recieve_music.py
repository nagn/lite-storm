import serial, serial.tools.list_ports
import sys
import base64
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
output = open('test.mp3', 'wb')
test = ''
while True:
    try:
        if ser.inWaiting():
            test += ser.readline()[:-1]
        else:
            try:
                if len(test) > 1000:
                    output.write(base64.b64decode(test))
                    print "wrote!"
                    test = ''
            except TypeError:
                print "TypeError!"
                test = ''
    except KeyboardInterrupt:
        output.close()
        ser.close()
        sys.exit(0)
