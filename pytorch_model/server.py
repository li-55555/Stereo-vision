import socket
import os
import sys
import struct 

from load import load_image
from cnn import CNN


def socket_service_image():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 9999))
        s.listen(10)
    except socket.error as msg:
        print(msg)
        sys.exit(1)
        
    print("Waiting for Connections to finish...")
    
    while True:
        sock, addr = s.accept()
        deal_image(sock, addr)
        
def deal_image(sock, addr):
    print("Accepting connection from {0}".format(addr))
    
    while True:
        fileinfo_size = struct.calcsize('128sq')
        buf = sock.recv(fileinfo_size)
        if buf:
            filename, filesize  = struct.unpack('128sq', buf)
            fn = filename.decode().strip('\x00')
            new_filename = os.path.join('./', 'new_' + fn)

            recvd_size = 0
            fp = open(new_filename, 'wb')
            while not recvd_size == filesize:
                if filesize - recvd_size > 1024:
                    data = sock.recv(1024)
                    recvd_size += len(data)
                else:
                    data = sock.recv(1024)
                    recvd_size = filesize
                fp.write(data)
            fp.close()
        
        ret_num = load_image('new_' + fn)
        print(ret_num)
        sock.send(str(ret_num).encode('utf-8'))
        sock.close()
        break
    
    
    
if __name__ == '__main__':
    socket_service_image()
                    

# tcp_server = socket(AF_INET, SOCK_STREAM)

# address = ('', 9999)
# tcp_server.bind(address)

# tcp_server.listen(128)

# client_socket, clientAddr = tcp_server.accept()
# from_client_socket = client_socket.recv(1024)

# print(from_client_socket.decode("gbk"))

# send_data = client_socket.send("hello world".encode("gbk"))

# client_socket.close().