import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32

class RobotLooper(Node):
    def __init__(self):
        super().__init__('robot_looper')
        self.commands=[
            '{"command": "movel(p[0.8, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"}',
            '{"command": "movel(p[0.5, -.13, 0.4,0.0,0.0,0], a=0.6, v=0.1, t=0, r=0)", "commandtype": "urscript"}']
        self.publisher_=self.create_publisher(String, 'external_commands_topic', 10)
        self.subscriber_=self.create_subscription(
            Int32,
            'queuelength',
            self.queue_callback,
            10
        )
        self.subscriber_
        self.publishmsg=String()
    
    def queue_callback(self,msg):
        if msg.data == "0" or msg.data == 0:
            for cmd in self.commands:
                self.publishmsg.data=cmd
                self.publisher_.publish(self.publishmsg)

def main(args=None):
    rclpy.init(args=args)
    robotlooper= RobotLooper()
    rclpy.spin(robotlooper)
    robotlooper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()