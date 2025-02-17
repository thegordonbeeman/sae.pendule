import socket, select

s = socket.socket()

s.bind(('127.0.0.1', 12346))
s.listen(5)
clients = []
while True:
    clientsocket, address = s.accept()
    clients.append(clientsocket)
    r2r, r2w, err = select.select(clients, clients, clients, 5)
    clientsocket.recv()