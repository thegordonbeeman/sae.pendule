import socket

class SlaveSocket:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self, ip_dest, port_dest):
        self._socket.connect((ip_dest, port_dest))

    def registerAsSlave(self):
        self._socket.send(b"REG SLAVE")

    def requestPosition(self):
        self._socket.send(b"POS")
        data = None
        while data is None:
            data = self._socket.recv(5)
        return int.from_bytes(data)
