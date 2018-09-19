import threading,multiprocessing
from pc_interface import PcWrapper
from socket import SHUT_RDWR,timeout

#The main method
def main():
    pc_wrapper = PcWrapper()
    listen_to_pc(pc_wrapper) #to spawn as thread, then test with process-based

def listen_to_pc(pc_wrapper,arduino_wrapper=None,bt_wrapper=None):
    server_socket = pc_wrapper.get_socket()
    print("Listening for connections...")
    server_socket.listen(0)
    #gets the connection object, the client's ip address and outbound port
    conn, addr =server_socket.accept()
    print("Got a connection from %s" % str(addr))
    conn.sendall(b"HELLO FROM SERVER!\n")
    server_socket.shutdown(SHUT_RDWR)
    server_socket.close()
    server_socket.listen(0)
    conn, addr =server_socket.accept()

    while(True):
        try:
            # encoding scheme is ASCII
            data = conn.recv(1024)
            msg = data.decode()
            print("RECEIVED FROM CLIENT: {}".format(msg))
            if (msg == "END" or not msg):
                break
            else:
                send = "ACK-{}\n".format(msg)
                conn.sendall(send.encode())
            print("--START NEXT RECV--")
        except timeout:
            pass
            #server_socket.shutdown(SHUT_RDWR)
            #server_socket.close()

    server_socket.shutdown(SHUT_RDWR)
    server_socket.close()

def listen_to_bluetooth():
    pass

def listen_to_arduino():
    pass


#required
if __name__ == '__main__':
    # execute only if run as a script
    main()