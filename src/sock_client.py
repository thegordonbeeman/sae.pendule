import socket
import time
import random


s1, s1_prefix, s1_values = socket.socket(socket.AF_INET, socket.SOCK_STREAM), 'M1', 10
s2, s2_prefix, s2_values = socket.socket(socket.AF_INET, socket.SOCK_STREAM), 'M2', 10
s3, s3_prefix, s3_values = socket.socket(socket.AF_INET, socket.SOCK_STREAM), 'S1', 5
s4, s4_prefix, s4_values = socket.socket(socket.AF_INET, socket.SOCK_STREAM), 'M3', 20
s5, s5_prefix, s5_values = socket.socket(socket.AF_INET, socket.SOCK_STREAM), 'S2', 5

sockets = [s1, s2, s3, s4, s5]
values = [s1_values, s2_values, s3_values, s4_values, s5_values]
prefixes = [s1_prefix, s2_prefix, s3_prefix, s4_prefix, s5_prefix]

print(sockets)

for s in sockets:
    s.connect(('127.0.0.1', 12345))

while True:
    for sock, prefix, value in zip(sockets, prefixes, values):
        payload = prefix +':'+ str(value)
        print(payload)
        sock.send(payload.encode())
    time.sleep(2)