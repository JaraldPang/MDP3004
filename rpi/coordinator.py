import threading
from multiprocessing import Process,Pipe
from pc_interface import PcWrapper
from bluetooth_interface import BluetoothWrapper
from arduino_interface import ArduinoWrapper
from socket import SHUT_RDWR,timeout
from bluetooth.btcommon import BluetoothError
from img_recognition import ImageProcessor

#The main method
def main():

    listener_endpoint, opencv_endpoint = Pipe()
    pc_wrapper = PcWrapper()
    bt_wrapper = BluetoothWrapper()
    ar_wrapper = ArduinoWrapper()

    listener_process = Process(target=initialize_listeners,args=(listener_endpoint,pc_wrapper,bt_wrapper,ar_wrapper))
    opencv_process = Process(target=initialize_opencv,args=(listener_endpoint,bt_wrapper))
    
    #set daemon so that when main process ends the child processeswill die also
    listener_process.daemon = True
    opencv_process = True

    listener_process.start()
    opencv_process.start()

    listener_process.join()
    opencv_process.join()
    pass

def initialize_opencv(pipe_endpoint=None,bt_wrapper=None):
    cv_process = ImageProcessor()
    capture_thread = threading.Thread(target=cv_process.capture,args=(pipe_endpoint,))
    process_thread = threading.Thread(target=cv_process.identify,args=(pipe_endpoint,bt_wrapper))

    capture_thread.start()
    process_thread.start()

    capture_thread.join()
    process_thread.join()

def initialize_listeners(pipe_endpoint,pc_wrapper,bt_wrapper,ar_wrapper):
    pc_thread = threading.Thread(target=listen_to_pc,args=(pc_wrapper,ar_wrapper,bt_wrapper))
    bt_thread = threading.Thread(target=listen_to_bluetooth,args=(bt_wrapper,pc_wrapper,ar_wrapper))
    ar_thread = threading.Thread(target=listen_to_arduino,args=(ar_wrapper,pc_wrapper,bt_wrapper))

    #we utilize 3 threads due to GIL contention. Any more than 3 will incur context and lock switch overheads
    pc_thread.start()
    ar_thread.start()
    bt_thread.start()

    pc_thread.join()
    ar_thread.join()
    bt_thread.join()

#will receive rpi_status
def listen_to_pc(pc_wrapper,arduino_wrapper=None,bt_wrapper=None,opencv_pipe=None):

    #gets the connection object, the client's ip address and outbound port
    conn = pc_wrapper.accept_connection_and_flush()
    #do not capture unless told to do so
    exploration_mode = False
    hold_arduino_commands = False
    while(1):
        try:
            # encoding scheme is ASCII
            #msg = conn.recv(1024).decode('utf-8')
            msg = ""
            while(1):
                char = conn.recv(1).decode('utf-8')
                if(char is None or char is ""):
                    raise ConnectionResetError("Malformed string received: {}".format(msg))
                msg += char
                if(msg.endswith("\n")):
                    break
            print("RECEIVED FROM PC INTERFACE: {}.".format(msg))
            if(msg.startswith("rpi")})
                #determine message
                if(msg == "explore")
                    exploration_mode = True
                    print("Mode set to: Fastest")
                    continue
                elif(msg == "fastest")
                    exploration_mode = False
                    print("Mode set to: Exploration")
                    continue
                #if msg is not empty, then it is robot's location and orientation
                elif(msg is not None)
                    #signal new capture job
                    opencv_pipe().send(msg[3:])
                    print("New Camera Capture Job received")
                    #there are 2 approaches 
                    #1) block all further messages until camera is done. unknown if there are other messages received before or after 
                    #2) enqueue signal, then enqueue all received commands. abit moot because PC waits for ack
                    # - we explore 1) first. less complexity
                    print(opencv_pipe().recv())
            elif(msg.startswith("ar")):
                if(exploration_mode):
                    print("PC HOLDING ARDUINO: {}".format(msg))
                    #reroute all messsages to the opencv thread. opencv thread now has command authority to release instructions by algorithm to arduino
                    arduino_wrapper.hold(msg[2:])
                else
                    print("PC WRITING TO ARDUINO: {}".format(msg))
                    arduino_wrapper.write(msg[2:])
            elif(msg.startswith("an")):
                print("PC WRITING TO ANDROID: {}".format(msg))
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
            print("RECEIVED FROM BT INTERFACE: {}.".format(msg))
            if(msg.startswith("al_")):
                print("BT writing to PC: {}".format(msg))
                pc_wrapper.write(msg[3:])
            elif(msg.startswith("ar_")):
                print("BT writing to ARDUINO: {}".format(msg))
                arduino_wrapper.write(msg[3:])
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
            msg = ser.readline().decode('UTF-8').rstrip('\r').rstrip('\n') #aruino using println to send so need remove \r\n
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
                print("ARDUINO writing to PC: {}".format(msg))
                pc_wrapper.write(msg[2:])
            elif(msg.startswith("an")):
                print("ARDUINO writing to ANDROID: {}".format(msg))
                bt_wrapper.write(msg[2:])
        except Exception as e:
            print(e)
            print("Unexpected Disconnect occurred from arduino, trying to reconnect...")
            ser.close()
            ser = ar_wrapper.reconnect()

    print("Closing Arduino Listener")


#required
if __name__ == '__main__':
    # execute only if run as a script
    initialize_listeners()