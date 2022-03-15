import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String

import socket


# HOST=socket.gethostbyname(socket.gethostname())
HOST='192.168.1.147'
PORT=53005
PORT2=53004

buffersize=1024
UDPServerSocket=socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


# TODO CHANGE TO TCP INSTEAD OF UPD
class QueueShouter(Node):
    def __init__(self):
        super().__init__('queue_shouter')
        self.commandList=[]
        self.firstempty=True
        self.qsubscriber_ = self.create_subscription(
            Int32,
            'queuelength',
            self.qlistener_callback,
            10)
        self.ocvsubscriber_=self.create_subscription(
            String,
            'objectseen',
            self.ocvlistener_callback,
            10
        )
        
        # UDPServerSocket.bind((HOST,PORT))
        self.qsubscriber_  # prevent unused variable warning
        self.get_logger().info("HELLO STARTING SHOUTER ON %s" % HOST)


    def qlistener_callback(self, msg):
        qlen= msg.data
        msgbyte=str.encode(str(qlen))
        UDPServerSocket.sendto(msgbyte,(HOST,PORT))
        self.get_logger().info('shouting %s' % (str(qlen)))
    
    def ocvlistener_callback(self, msg):
        qlen= msg.data
        msgbyte=str.encode(str(qlen))
        UDPServerSocket.sendto(msgbyte,(HOST,PORT2))
        self.get_logger().info('shouting %s' % (str(qlen)))
    
    



def main(args=None):
    rclpy.init(args=args)
    qshouter=QueueShouter()
    rclpy.spin(qshouter)
    qshouter.destroy_node()
    rclpy.shutdown

if __name__ == '__name__':
    main()