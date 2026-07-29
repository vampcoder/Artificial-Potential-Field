import sys
import socket

class socket_connection:
    
    def connect_to_server(self,host,port):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host,port))
            return sock
        except:
            print("Unexpected error:", sys.exc_info()[0])
            print("Error in Connecting")

    def start_server(self,host,port):
        server_sock = socket.socket()         # Create a socket object
        server_sock.bind((host, port))        # use host if u know about the IP address or manually give the IP address

        print('Server Started')
        server_sock.listen(5)                 # Waiting for client
        socket_connection.c, addr = server_sock.accept()        # Establish connection with client.
        print('Got connection from', addr)
        return socket_connection.c

    def read_data(self,sock):
        size = 2
        try:
            data = sock.recv(size)
            #print data
            return data
        except :
            print("Error in Recieving")

    def send_data(self,sock):
        # one way
        dat = input('Enter Data : ')
        try:
            sock.send(dat)
            print(dat)
        except :
            print("Error in Sending")
        
            
    def __init__(self):
        print("connect_to_server(host,port)")
        print("start_server(host,port)")
'''
try:
    s = socket_connection()
    soc = s.start_server('192.168.43.96',12346)
    
    while True:
        re = s.read_data(soc)
        print(re)
        if re == '0_0':
            break
except KeyboardInterrupt:
    print('broke')
    soc.close
'''
