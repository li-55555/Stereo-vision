import socket
import os
import sys
import struct

def socket_image():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('172.20.9.150', 9999))
        except socket.error as msg:
            print(msg)
            print(sys.exit(1))
        filepath = input('input the file path')
        fhead = struct.pack(b'128sq', bytes(os.path.basename(filepath), encoding='utf8'),
                            os.stat(filepath).st_size)
        s.send(fhead)
        fp = open(filepath, 'rb')
        while True:
            data = fp.read(1024)
            if not data:
                print('{0} send over'.format(filepath))
                break
            s.send(data)
        s.close()
        
if __name__ == '__main__':
    socket_image()