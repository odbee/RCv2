import socket

ROBOT_HOST_IP = '192.168.1.112'
RTDE_PORT = 30004

SECONDARY_PORT=30002

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ROBOT_HOST_IP, SECONDARY_PORT))
input()
comm = "movel(p[0.8, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)" + " \n"
s.send(comm.encode())
input()
comm = "movel(p[0.5, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)" + " \n"
s.send(comm.encode())

s.close()