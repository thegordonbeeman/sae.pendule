from MasterSocket import MasterSocket
from ServerSocket import ServerSocket
from SlaveSocket import SlaveSocket
import time, random

slave = SlaveSocket()

master = MasterSocket()

sockets = [slave, master]


for s in sockets:
    s.connect('127.0.0.1', 12345)

slave.registerAsSlave()
master.registerAsMaster()

try:
    while 1:
        '''    for m in masters:
            time.sleep(1*random.uniform(0,1))
            m.sendPosition(0.2)

        for s in slaves:
            time.sleep(1*random.uniform(0,1))
            s.requestPosition()'''
        slave._socket.send(b"SLAVE")
        time.sleep(2)
        master._socket.send(b"MASTER")
        time.sleep(2)

except BrokenPipeError:
    for s in sockets:
        s.close()
        exit()