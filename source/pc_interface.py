import socket

'''
    Class PCWrapper wraps the PC connection interface
'''

class PcWrapper:

    '''
        Creates a wrapper around the scoket for message passing between threads/processes
        parameters
            host - a ip address/interface on the local machine to bind to
            port - port to bind
            timeout - timeout in seconds
    '''
    def __init__(self,host='',port=45000):
        #create the socket object as AF_INET, which defines the address family as internet addresses and
        #sets the socket as streaming
        self.server_socket = None
        self.conn = None
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #set the socket to reuse IP addresses to prevent "Address in use error"
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #disable Nagle's Algorithm to force sending of packets as soon as possible to minimize latency
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            #bind accepts a tuple containing the host interface to bind to, as well as port
            self.server_socket.bind((host,port))
            print("Listening for connections for PC interface...")
            # set socket to listen to interface
            self.server_socket.listen(0)
        except socket.error:
            print("Failed to create socket: " + str(socket.error))

    #we delegate read jobs to the external user
    def write_to_pc(self,msg):
        try:
            self.conn.sendall("{}\n".format(msg).encode())
            return True
        except socket.error:
            return False

    def accept_connection(self):
        try:
            self.conn = None
            # gets the connection object, the client's ip address and outbound port
            conn, addr = self.server_socket.accept()
            # output to console
            self.conn = conn
            print("Accepted PC Connection from %s" % str(addr))
            return conn
        except Exception as e:
            print("\nError: %s" % str(e))

    #returns the socket for external handling
    def get_socket(self):
        return self.server_socket

    def get_connection(self):
        return self.conn

    #may need to redefine a reconnect method