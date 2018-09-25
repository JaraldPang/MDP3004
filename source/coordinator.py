import threading,multiprocessing
from pc_interface import PcWrapper
from bluetooth_interface import BluetoothWrapper
from arduino_interface import ArduinoWrapper
from socket import SHUT_RDWR,timeout
from bluetooth.btcommon import BluetoothError

#The main method
def main():
    pc_wrapper = PcWrapper()
    bt_wrapper = BluetoothWrapper()
    ar_wrapper = None
    pc_thread = threading.Thread(target=listen_to_pc,args=(pc_wrapper,ar_wrapper,bt_wrapper))
    bt_thread = threading.Thread(target=listen_to_bluetooth,args=(bt_wrapper,pc_wrapper,ar_wrapper))
    ar_thread = threading.Thread(target=listen_to_arduino,args=(ar_wrapper,pc_wrapper,bt_wrapper))

    #we utilize 3 threads due to GIL contention. Any more than 3 will incur context and lock switch overheads
    pc_thread.start()
    #ar_thread.start()
    bt_thread.start()

    pc_thread.join()
    #ar_thread.join()
    bt_thread.join()

def listen_to_pc(pc_wrapper,arduino_wrapper=None,bt_wrapper=None):

    #gets the connection object, the client's ip address and outbound port
    conn = pc_wrapper.accept_connection_and_flush()
    #handshake with client
    while(1):
        try:
            # encoding scheme is ASCII
            msg = conn.recv(1024).decode()
            print("RECEIVED FROM PC INTERFACE: {}".format(msg))
            if(msg.startswith("AR")):
                arduino_wrapper.write(msg[2:])
            elif(msg.startswith("AN")):
                if(bt_wrapper.is_connected()):
                    bt_wrapper.write(msg[2:])
                else:
                    print("BT NOT CONNECTED")
            #DISCARD THIS AFTER TESTING, ELSE IT WILL CAUSE SCREWUPS IF YOU SEND
            #A MSG FROM PC WITH NO DIRECTION
            else:
               raise ConnectionResetError
        except (timeout,ConnectionResetError) as e:
            print("Unexpected Disconnect for PC occurred. Awaiting reconnection...")
            conn.close()
            conn = pc_wrapper.accept_connection_and_flush()
            pass

    conn.shutdown(SHUT_RDWR)
    conn.close()
    print("Closing PC Listener")

def listen_to_bluetooth(bt_wrapper,pc_wrapper=None,arduino_wrapper=None,):
    conn = bt_wrapper.accept_connection_and_flush()
    while(1):
        try:
            # encoding scheme is ASCII
            msg = conn.recv(1024).decode('utf-8')
            print("RECEIVED FROM BT INTERFACE: {}".format(msg))
            if(msg.startswith("AL")):
                if(pc_wrapper.is_connected()):
                    pc_wrapper.write(msg[2:])
                else:
                    print("PC NOT CONNECTED")
            elif(msg.startswith("AR")):
                arduino_wrapper.write(msg[2:])
        except (timeout,BluetoothError):
            print("Unexpected Disconnect for Bluetooth occurred. Awaiting reconnection...")
            conn.shutdown(SHUT_RDWR)
            conn.close()
            conn = bt_wrapper.accept_connection_and_flush()
            

    bt_wrapper.close_bt_socket()
    print("Closing Bluetooth Listener")

def listen_to_arduino(ar_wrapper,pc_wrapper=None,bt_wrapper=None):
    ser = ar_wrapper.get_connection()
    while(1):
        try:
            msg = ser.readline().decode('UTF-8').rstrip('\r\n') #aruino using println to send so need remove \r\n
            print("RECEIVED FROM ARDUINO INTERFACE: {}".format(msg))
            if(msg.startswith("AL")):
                pc_wrapper.write(msg[2:])
            elif(msg.startswith("AN")):
                bt_wrapper.write(msg[2:])
        except Exception:
            print("Unexpected Disconnect occurred from arduino, trying to reconnect...")
            ser = ar_wrapper.reconnect()

    print("Closing Arduino Listener")


#required
if __name__ == '__main__':
    # execute only if run as a script
    main()