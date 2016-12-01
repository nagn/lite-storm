import sys
import base64

mp3 = open('awful.mp3', 'rb')
b = base64.b64encode(mp3.read())
mp3.close()

try:
    file = open('converted.txt', 'w')
    file.write(b)
    file.close()
except:
    print('Something went wrong!')
    sys.exit(0)
