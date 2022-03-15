import socket

HEADER = 64
PORT = 65432
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
HOST = '127.0.0.1' 
ADDR = (HOST, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

def send(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client.send(send_length)
    client.send(message)
    print(client.recv(2048).decode(FORMAT))




commm='{"commands": [{"command": "a, 255, 0, 0", "commandtype": "gripper"}]}'
co= '{"commands": [{"command": "movel(p[0.7, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"}]}'
cooo= '{"commands": [{"command": "movel(p[1.0, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"}]}'


coo= '{"commands": [\
    {"command": "movel(p[0.8, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"},\
{"command": "movel(p[0.5, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"}]}'
send(cooo)
send(coo)



send(DISCONNECT_MESSAGE)
client.close()