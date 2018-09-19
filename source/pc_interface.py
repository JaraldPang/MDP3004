import socket

'''
    Class PCWrapper wraps the PC connection interface
'''

class PcWrapper:

    #Creates a wrapper around the scoket for message passing between threads/processes
    def __init__(self,host='',port=45000):
        #create the socket object as AF_INET, which defines the address family as internet addresses and
        #sets the socket as streaming
        server_socket = None
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #set the socket to reuse IP addresses to prevent "Address in use error"
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #disable Nagle's Algorithm to force sending of packets as soon as possible to minimize latency
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            #set timeout for TCP timeout and error throw()
            #self.server_socket.settimeout()
            #bind accepts a tuple containing the host interface to bind to, as well as port
            self.server_socket.bind((host,port))
        except socket.error:
            print("Failed to create socket: " + str(socket.error))

    #we delegate read jobs to the external user
    def write_to_pc(self,msg):
        try:
            self.server_socket.sendall(msg.encode())
            return True
        except socket.error:
            return False

    #returns the socket for external handling
    def get_socket(self):
        return self.server_socket

    #may need to redefine a reconnect method