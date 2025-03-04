from MasterSocket import MasterSocket
from ServerSocket import ServerSocket
from SlaveSocket import SlaveSocket
import time

server_sock = ServerSocket()
server_sock.bind('127.0.0.1', 12345)
server_sock.waitForClients(2)
print("CLIENTS CONNECTED")

while len(server_sock.masters) != 1 and len(server_sock.slaves) != 1: 
    server_sock.registerClients()

server_sock.listenToClients()

try:
    while 1:
        time.sleep(1)
except KeyboardInterrupt:
    server_sock.close()
    server_sock.shutdown(0)
    exit()
