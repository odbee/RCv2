import rclpy
import json
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String


# HOST = '127.0.0.1'
HOST = '192.168.1.141'
PORT = 65432
ADDR = (HOST, PORT)
HEADER = 64
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"


class ExternalCommandPublisher(Node):

    def __init__(self):
        super().__init__('external_commands_publisher')
        
        self.publisher_ = self.create_publisher(String, 'external_commands_topic', 10)
        self.get_logger().info("STARTING PUBLISHER ON %s" % HOST)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen()
        while True:
            conn, addr = self.server.accept()
            thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            thread.start()
            # self.get_logger().info('[ACTIVE CONNECTIONS] "%s"' % str(threading.activeCount() - 1))



    def handle_client(self, conn, addr):
        # self.get_logger().info('[NEW CONNECTION] "%s" connected.' % str(addr))
        connected = True
        while connected:
            msg_length = conn.recv(HEADER).decode(FORMAT)
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length).decode(FORMAT)
                if msg == DISCONNECT_MESSAGE:
                    connected = False
                else:
                    # self.get_logger().info('[MSG RECEIVED] "%s" ' % str(msg))
                    print(f"[{addr}] {msg}")
                    self.parse_message_and_publish(msg)
                    conn.send("Msg received".encode(FORMAT))
        conn.close()
        # self.get_logger().info('[DISCONNECTED]')

    def parse_message_and_publish(self,message):
        parsed_message = json.loads(message)
        for singlemsg in parsed_message["commands"]:
            msg_publish=String()
            dumpedmsg = json.dumps(singlemsg)

            msg_publish.data=dumpedmsg
            self.get_logger().info('[SENDING COMMAND] "%s" ' % str(dumpedmsg))
            self.publisher_.publish(msg_publish)

        



def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = ExternalCommandPublisher()
    

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.server.close()
    minimal_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
