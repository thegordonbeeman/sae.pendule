import socket

class ServerSocket:

    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clients = []
        self.masters = []
        self.slaves = []

    def bind(self, ip, port):
        self._socket.bind((ip, port))
        self._socket.listen(5)

    def waitForClients(self, clients_count):
        while len(self.clients) != clients_count:
            client_socket, client_addr = self._socket.accept()
            self.clients.append(client_socket)

    def registerClients(self):
        data = None
        for c in self.clients:
            data = c.recv(20)
            if data is not None:
                data_string = str(data)
                if 'REG' in data_string:
                    if 'MASTER' in data_string:
                        self.masters.append(c)
                        print('MASTER REGISTERED')
                    elif 'SLAVE' in data_string:
                        self.slaves.append(c)
                        print('SLAVE REGISTERED')

    def waitForRequest(self):
        data = None
        for c in self.clients:
            while data is None:
                data = str(c.recv(10))
                print(data)
                if data != '':
                    self.handleRequest(data)

    def handleRequest(self, request):
        print(request)
