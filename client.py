import socket
import os
import sys
import struct

def socket_image(filepath):
    while True:
        try:

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('172.20.10.3', 9999))

        except socket.error as msg:
            print(msg)
            print(sys.exit(1))

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
        ret = s.recv(9999).decode("utf-8")
       
        print("ret:")
        if(ret) :
            print("bad box")
        else :
            print("good box")
        # print(ret)
        s.close()
        break
        
if __name__ == '__main__':
    socket_image(sys.argv[1])
    socket_image(sys.argv[2])
    # socket_image("right0.bmp")

    # socket_image("left0.bmp")