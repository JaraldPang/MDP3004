from bluetooth import *


class BluetoothWrapper(object):
    def __init__(self):
        self.server_socket = None
        self.client_socket = None
        self.bt_is_connected = False
        try:
            self.server_socket = BluetoothSocket(RFCOMM)
            self.server_socket.bind(("", btport))
            self.server_socket.listen(1)  # Listen for requests
            self.port = self.server_socket.getsockname()[1]
            #assign a UUID of the application, in theory for receiving side to connect
            uuid = "00001101-0000-1000-8000-00805F9B34FB"
            #advertise the application
            advertise_service(
                self.server_socket, "SampleServer",
                service_id=uuid,
                service_classes=[uuid, SERIAL_PORT_CLASS],
                profiles=[SERIAL_PORT_PROFILE]
                )
        except Exception as e:
            print("\nError: %s" % str(e))

    def close_bt_socket(self):

        if self.client_socket:
            self.client_socket.close()
            # self.client_socket.shutdown(socket.SHUT_RDWR)
            print("Closing client socket")
        if self.server_socket:
            self.server_socket.close()
            # self.client_socket.shutdown(socket.SHUT_RDWR)
            print("Closing server socket")
        self.bt_is_connected = False

    def bt_is_connect(self):
        return self.bt_is_connected

    def accept_connection(self,btport=4):

        # Creating the server socket and bind to port
        try:
            self.client_socket = None
            print("Waiting for BT connection on RFCOMM channel %d" % self.port)
            # Accept requests
            self.client_socket, client_address = self.server_socket.accept()
            print("Accepted connection from ", client_address)
            return self.client_socket
        except Exception as e:
            print("\nError: %s" % str(e))

    def blueSend(self, message):
        """
        Write message to Nexus
        """
        try:
            self.client_socket.send(str(message))
            return True
        except BluetoothError:
            print("\nBluetooth Write Error. Connection lost")
            self.close_bt_socket()
            self.connect_bluetooth()  # Reestablish connection


