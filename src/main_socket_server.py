from MasterSocket import MasterSocket
from ServerSocket import ServerSocket
from SlaveSocket import SlaveSocket

server_sock = ServerSocket()
server_sock.bind('127.0.0.1', 12346)
server_sock.waitForClients(5)
print("CLIENTS CONNECTED")

while len(server_sock.masters) != 2 and len(server_sock.slaves) != 3: 
    server_sock.registerClients()

print(server_sock.clients)
while 1:
    server_sock.waitForRequest()