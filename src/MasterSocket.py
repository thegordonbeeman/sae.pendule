import socket

class MasterSocket:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self, ip_dest, port_dest):
        self._socket.connect((ip_dest, port_dest))

    def sendPosition(self, angle):
        angle = int(angle*100)
        self._socket.send(str(angle).encode())

    def registerAsMaster(self):
        self._socket.send(b"REG MASTER")
