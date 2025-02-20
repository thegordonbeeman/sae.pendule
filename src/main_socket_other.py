from MasterSocket import MasterSocket
from ServerSocket import ServerSocket
from SlaveSocket import SlaveSocket
import time, random

s1 = SlaveSocket()
s2 = SlaveSocket()
s3 = SlaveSocket()
s4 = MasterSocket()
s5 = MasterSocket()

sockets = [s1, s2, s3, s4, s5]
masters = [s4, s5]
slaves = [s1, s2, s3]

for s in sockets:
    s.connect('127.0.0.1', 12346)

for s in masters:
    s.registerAsMaster()

for s in slaves:
    s.registerAsSlave()

while 1:
    '''    for m in masters:
        time.sleep(1*random.uniform(0,1))
        m.sendPosition(0.2)

    for s in slaves:
        time.sleep(1*random.uniform(0,1))
        s.requestPosition()'''

    masters[0]._socket.send(b"FOOBAR")
