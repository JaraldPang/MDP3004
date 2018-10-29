import threading
from multiprocessing import Process,Pipe
from pc_interface import PcWrapper
from bluetooth_interface import BluetoothWrapper
from arduino_interface import ArduinoWrapper
from socket import SHUT_RDWR,timeout
from bluetooth.btcommon import BluetoothError

def initialize_listeners():
    pc_wrapper = PcWrapper()
    bt_wrapper = BluetoothWrapper()
    ar_wrapper = ArduinoWrapper()
    
    pc_thread = threading.Thread(target=listen_to_pc,args=(pc_wrapper,ar_wrapper,bt_wrapper))
    bt_thread = threading.Thread(target=listen_to_bluetooth,args=(bt_wrapper,pc_wrapper,ar_wrapper))
    ar_thread = threading.Thread(target=listen_to_arduino,args=(ar_wrapper,pc_wrapper,bt_wrapper))
    #arrow_thread = threading.Thread(target=write_arrow_to_pc, args=(pc_wrapper,recog_endpoint))

    #we utilize 3~4 threads due to GIL contention. Any more than 3 will incur context and lock switch overheads
    pc_thread.start()
    ar_thread.start()
    bt_thread.start()
    #arrow_thread.start()

    pc_thread.join()
    ar_thread.join()
    bt_thread.join()
    #arrow_thread.join()

#will receive rpi_status
def listen_to_pc(pc_wrapper,arduino_wrapper=None,bt_wrapper=None,opencv_pipe=None):

    #gets the connection object, the client's ip address and outbound port
    conn = pc_wrapper.accept_connection_and_flush()
    #do not capture unless told to do so
    exploration_mode = False
    while(1):
        try:
            # encoding scheme is ASCII
            #msg = conn.recv(1024).decode('utf-8')
            msg = ""
            while(1):
                char = conn.recv(1).decode('utf-8')
                if(char is None or char is ""):
                    raise ConnectionResetError("Malformed string received: {}".format(msg))
                if(char == "\n"):
                    break
                msg += char
            print("RECEIVED FROM PC INTERFACE: {}.".format(msg))
            if(msg.startswith("rpi")):
                #signal new capture job
                #opencv_pipe.send(msg[3:])
                #block thread until received job finished from camera
                #opencv_pipe.recv()
                continue
            elif(msg.startswith("ar")):
                #if(exploration_mode):
                #   print("PC HOLDING ARDUINO: {}".format(msg))
                    #reroute all messsages to the opencv thread. opencv thread now has command authority to release instructions by algorithm to arduino
                #   arduino_wrapper.hold(msg[2:])
                #else:
                #print("PC WRITING TO ARDUINO: {}".format(msg))
                arduino_wrapper.write(msg[2:])
            elif(msg.startswith("an")):
                #print("PC WRITING TO ANDROID: {}".format(msg))
                bt_wrapper.write(msg[2:])
            #raises a connectione error for the following situation
            #1) RPI resets while PC is connected
            #2) PC reconnects
            #3) msg is sent to PC
            else:
                if(msg is False):
                    raise ConnectionResetError("Null or empty string received arising from connection reset")
                else:
                    raise ConnectionResetError("Malformed string received: {}".format(msg))
        except (timeout,ConnectionResetError) as e:
            print(e)
            print("Unexpected Disconnect for PC occurred. Awaiting reconnection...")
            conn.close()
            conn = pc_wrapper.accept_connection_and_flush()
        except Exception as e:
            print("Unexpected Disconnect for Bluetooth occurred. The following error occurred: {}. Awaiting reconnection...".format(e))
            conn.shutdown(SHUT_RDWR)
            conn.close()
            conn = pc_wrapper.accept_connection_and_flush()

    conn.shutdown(SHUT_RDWR)
    conn.close()
    print("Closing PC Listener")


def listen_to_bluetooth(bt_wrapper,pc_wrapper=None,arduino_wrapper=None,):
    conn = bt_wrapper.accept_connection_and_flush()
    while(1):
        try:
            # encoding scheme is ASCII
            msg = conn.recv(1024).decode('utf-8')
            print("RECEIVED FROM BT INTERFACE: {}.".format(msg))
            if(msg.startswith("al_")):
                #print("BT writing to PC: {}".format(msg))
                pc_wrapper.write(msg[3:])
            elif(msg.startswith("ar_")):
                #print("BT writing to ARDUINO: {}".format(msg))
                arduino_wrapper.write(msg[3:])
        except (timeout,BluetoothError):
            print("Unexpected Disconnect for Bluetooth occurred. Awaiting reconnection...")
            conn.shutdown(SHUT_RDWR)
            conn.close()
            conn = bt_wrapper.accept_connection_and_flush()
        except Exception as e:
            print("Unexpected Disconnect for Bluetooth occurred. The following error occurred: {}. Awaiting reconnection...".format(e))
            conn.shutdown(SHUT_RDWR)
            conn.close()
            conn = bt_wrapper.accept_connection_and_flush()
            

    bt_wrapper.close_bt_socket()
    print("Closing Bluetooth Listener")

def listen_to_arduino(ar_wrapper,pc_wrapper=None,bt_wrapper=None):
    ser = ar_wrapper.get_connection()
    while(1):
        try:
            msg = ser.readline().decode('ascii',errors='ignore').strip() #aruino using println to send so need remove \r\n
            #msg = ""
            #while(1):
            #   char = ser.read(1).decode('utf-8')
            #   print("{}".format(char))
            #   if(char is None or char is ""):
            #       continue
            #   msg += char
            #  if(msg.endswith("\n")):
            #       print("\n")
            #       break
            print("RECEIVED FROM ARDUINO INTERFACE: {}.".format(msg))
            if(msg.startswith("al")):
                pc_wrapper.write(msg[2:])
                print("ARDUINO wrote to PC: {}".format(msg))
            elif(msg.startswith("an")):
                bt_wrapper.write(msg[2:])
                print("ARDUINO wrote to ANDROID: {}".format(msg))
        except UnicodeDecodeError as ude:
            print(ude)
            continue
        except Exception as e:
            print(e)
            print("Unexpected Disconnect occurred from arduino, trying to reconnect...")
            ser.close()
            ser = ar_wrapper.reconnect()

    print("Closing Arduino Listener")

def write_arrow_to_pc(pc_wrapper, listener_endpoint_rpi):
    while(1):
        msg = listener_endpoint_rpi.recv()
        pc_wrapper.write(msg)
        
    

#required
if __name__ == '__main__':
    # execute only if run as a script
    initialize_listeners()