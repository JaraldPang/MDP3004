import socket
import queue
'''
    Class PCWrapper wraps the PC connection interface
'''

class PcWrapper:

    '''
        Creates a wrapper around the socket for message passing between threads/processes
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
        #used deque as Queue.queue did not provide easy way to peek the head of queue
        self.queue = queue.Queue()
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #set the socket to reuse IP addresses to prevent "Address in use error"
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #disable Nagle's Algorithm to force sending of packets as soon as possible to minimize latency
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            #set all new sockets created to have a default timeout of 60 seconds, excluding server socket
            socket.setdefaulttimeout(60)
            #bind accepts a tuple containing the host interface to bind to, as well as port
            self.server_socket.bind((host,port))
            print("Listening for connections for PC interface...")
            # set socket to listen to interface
            self.server_socket.listen(0)
        except socket.error:
            print("Failed to create socket: " + str(socket.error))

    #accept_connection returns the connection or client socket
    def accept_connection(self):
        self.conn = None
        # gets the connection object, the client's ip address and outbound port
        conn, addr = self.server_socket.accept()
        # output to console
        self.conn = conn
        print("Got a connection from %s" % str(addr))
        return conn

    def accept_connection_and_flush(self):
        conn = self.accept_connection()
        #no peek method makes this unnecessarily harder
        next_msg = None
        while(self.queue.empty() is False):
            try:
                if(next_msg is None):
                    next_msg = self.queue.get()
                print("Flushing PC interface...")
                conn.sendall("{}\n".format(next_msg).encode())
                next_msg = None
            except(socket.timeout,socket.error,ConnectionResetError):
                conn = self.accept_connection()
        self.conn = conn
        return conn

    #we delegate read jobs to the read thread
    #we also delegate flushing of the queue to the reader thread
    def write(self,msg):
        #print("Writing to PC: {}. Connection: {}".format(msg, self.conn))
        try:
            #if the queue is not empty there was a disconnect and the reader thread is flushing, enqueue this msg
            if(self.queue.empty() is False):
                print("Placed {} in PC queue".format(msg))
                self.queue.put(msg)
            else:
                print("Writing to PC: {}".format(msg))
                self.conn.sendall("{}\n".format(msg).encode())
            return True
        except Exception as e:
            print("PC write encountering the following error: %s" % str(e))
            self.queue.put(msg)
            return False

    #returns the socket for external handling
    def get_socket(self):
        return self.server_socket

    def get_connection(self):
        return self.conn

    def is_connected(self):
        return self.conn is not None
    #may need to redefine a reconnect method