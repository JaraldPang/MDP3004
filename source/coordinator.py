import threading,multiprocessing
from pc_interface import PcWrapper
from bluetooth_interface import BluetoothWrapper
from socket import SHUT_RDWR,timeout
from bluetooth.btcommon import BluetoothError

#The main method
def main():
    pc_wrapper = PcWrapper()
    bt_wrapper = BluetoothWrapper()
    listen_to_pc(pc_wrapper) #to spawn as thread, then test with process-based
    #listen_to_bluetooth(bt_wrapper)

def listen_to_pc(pc_wrapper,arduino_wrapper=None,bt_wrapper=None):

    #gets the connection object, the client's ip address and outbound port
    conn = pc_wrapper.accept_connection()
    #handshake with client
    conn.sendall(b"HELLO FROM SERVER!\n")
    while(1):
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
        except (timeout,ConnectionResetError):
            print("Unexpected Disconnect occured. Awaiting reconnection...")
            conn.close()
            conn = pc_wrapper.accept_connection()
            pass

    conn.shutdown(SHUT_RDWR)
    conn.close()

def listen_to_bluetooth(bt_wrapper,pc_wrapper=None,arduino_wrapper=None,):
    conn = bt_wrapper.accept_connection()
    #handshake with client
    conn.sendall("HELLO FROM BT SERVER!")
    while(1):
        try:
            # encoding scheme is ASCII
            msg = conn.recv(1024).decode('utf-8')
            print("RECEIVED FROM CLIENT: {}".format(msg))
            if (msg == "END" or not msg):
                break
            else:
                send = "ACK-{}\n".format(msg)
                conn.sendall(send.encode('utf-8'))
            print("--START NEXT RECV--")
        except (timeout,BluetoothError):
            print("Unexpected Disconnect occured. Awaiting reconnection...")
            conn.shutdown(SHUT_RDWR)
            conn.close()
            conn = bt_wrapper.accept_connection()
            pass

    bt_wrapper.close_bt_socket()

def listen_to_arduino():
    pass


#required
if __name__ == '__main__':
    # execute only if run as a script
    main()