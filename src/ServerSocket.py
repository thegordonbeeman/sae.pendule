import socket
import threading

class ServerSocket:

    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clients = []
        self.masters = []
        self.slaves = []
        self.master_positions = []

    def bind(self, ip, port):
        self._socket.bind((ip, port))
        self._socket.listen(5)

    def close(self):
        self._socket.close()
        self._socket.shutdown()

    def waitForClients(self, clients_count):
        while len(self.clients) != clients_count:
            client_socket, client_addr = self._socket.accept()
            self.clients.append(client_socket)

    def registerClients(self):
        data = b''
        for c in self.clients:
            data = c.recv(20)
            if data != b'':
                data_string = str(data)
                if 'REG' in data_string:
                    if 'MASTER' in data_string:
                        self.masters.append(c)
                        print('MASTER REGISTERED')
                    elif 'SLAVE' in data_string:
                        self.slaves.append(c)
                        print('SLAVE REGISTERED')

    def waitForRequest(self, socket):
        data = b''
        while data == b'':
            data = socket.recv(10)
            if data != b'':
                self.handleRequest(data)
                data = b''

    def listenToClients(self):
        threads = []
        for client in self.clients:
            t = threading.Thread(target=self.waitForRequest, args=(client,))
            threads.append(t)
        for t in threads:
            t.start()

    def handleRequest(self, request):
        data_string = request.decode()
        if "POS" in data_string:
            pass
        elif "GET" in data_string:
            pass
