import rclpy
from rclpy.node import Node
from adam_msgs.srv import ReceiveCommand
from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Int32


class CommandListManager(Node):

    def __init__(self):
        super().__init__('command_list_manager')
        self.commandList=[]
        self.firstempty=True
        self.subscription = self.create_subscription(
            String,
            'external_commands_topic',
            self.listener_callback,
            100)
        self.publisher_=self.create_publisher(
            Int32,
            'queuelength',
            100
        )
        self.ql=Int32()

        self.srv=self.create_service(ReceiveCommand, 'recv_cmd', self.pull_first_cmd_callback)
        self.srv2=self.create_service(Empty, 'remove_first_cmd',self.remove_first_cmd_callback)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.commandList.append(msg.data)
        # self.get_logger().info('Added Command: "%s"\n' % msg.data)
        # self.get_logger().info('Current Length of List is: %s' %str(len(self.commandList)))
        # self.get_logger().info('current list: "%s"\n' % str(self.commandList))

    def pull_first_cmd_callback(self, request, response):
        try:

            response.msg=self.commandList[0]
            self.get_logger().info('Received Request. \n Sending command: %s' % (str(response.msg)))
            self.firstempty=True
            self.ql.data=len(self.commandList)
            # self.publisher_.publish(self.ql)
            
        except:
            if self.firstempty==True:
                self.get_logger().info('[PULLBACK] Command List Empty. Add more commands and try again')
                self.firstempty=False
            response.msg='ERROR' #TODO TAKE CAARE OF THIS
        return response # TODO RETURN ERROR INSTEAD OF REGULAR RESPONSE

    def remove_first_cmd_callback(self, request, response):
        try:
            # del self.commandList[0]
            popped=self.commandList.pop(0)

            self.get_logger().info('Received Request to remove first item. \n Removed Command: %s' % (str(popped)))
            # self.get_logger().info('Deleted Element: %s, Current Length of List is: %s'% popped   %str(len(self.commandList)))
            self.ql.data=len(self.commandList)
            self.publisher_.publish(self.ql)
            
        except:
            # self.get_logger().info('Command List Empty. Add more commands and try again')
            self.firstempty=False
            errmess='ERROR' #TODO TAKE CAARE OF THIS
        return response # TODO RETURN ERROR INSTEAD OF REGULAR RESPONSE

        

        



def main(args=None):
    rclpy.init(args=args)

    cmd_manager = CommandListManager()

    rclpy.spin(cmd_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()