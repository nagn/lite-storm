import time
import base64
def read_in_chunks(file_object, chunk_size=32):
    """Lazy function (generator) to read a file piece by piece.
    Default chunk size: 1k."""
    while True:
        data = file_object.read(chunk_size)
        if not data:
            break
        yield data


f = open('converted.txt', 'r')
output = open('test.mp3', 'wb')
for piece in read_in_chunks(f):
    output.write(base64.b64decode(piece))
    time.sleep(0.005)

'''
import socket

filePath = "mono.mp3"
fileData = open(filePath, "rb").read()
host = ''
port = 8808
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HTTPString = "HTTP/1.1 200 OK\r\nConnection: Keep-Alive\r\nContent-Type: audio/mpeg\r\n\r\n" + fileData

s.bind((host, port))

s.listen(10)
while 1:
    try:
        conn, addr = s.accept()
        conn.sendall(HTTPString)
    except socket.error, e:
        pass
'''
