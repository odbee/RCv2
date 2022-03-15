import rclpy
from rclpy.node import Node
from adam_msgs.srv import ReceiveCommand
from adam_msgs.srv import SendOrderReturnResult
import json
import time 
from std_srvs.srv import Empty
from std_msgs.msg import String

# TODO ADD REMOVE FIRST ELEMENT AFTER SUCCESSFUL EXECUTION
# TODO FIX EXITS AFTER ERROR

class CommandExecutionManager(Node):

    def __init__(self):
        super().__init__('command_execution_manager')
        self.commandList=[]
        
        #create client to get command from lists
        self.cli=self.create_client(ReceiveCommand, 'recv_cmd')
        
        #create clients to forward command to executers
        self.gripperclient=self.create_client(SendOrderReturnResult, 'gripper_order')
        self.robotclient=self.create_client(SendOrderReturnResult, 'robot_order')
        
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req=ReceiveCommand.Request()

    def send_request(self):
        self.future=self.cli.call_async(self.req)

    def send_order(self,order):
        parsed_cmd= self.parse_message(order)
        
        if (parsed_cmd["commandtype"]=="gripper"):
            self.get_logger().info('service not available, waiting again...')
            self.gripperorder = SendOrderReturnResult.Request()
            
            while not self.gripperclient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            sendorder = parsed_cmd["command"].replace(" ", "")

            self.gripperorder.order=sendorder

            self.future_order=self.gripperclient.call_async(self.gripperorder)

        if (parsed_cmd["commandtype"]=="urscript"):
            self.urscriptorder = SendOrderReturnResult.Request()
            while not self.gripperclient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.gripperorder = SendOrderReturnResult.Request()
            self.gripperorder.order=parsed_cmd["command"]
            self.future_order=self.gripperclient.call_async(self.gripperorder)
            
            pass
        try:
            result= self.future_order.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            self.get_logger().info(
                'Result of Command request %s' %
                result )

    def parse_message(self,message):
        parsed_msg = json.loads(message)
        return parsed_msg


        



def main(args=None):
    rclpy.init(args=args)

    exec_manager = CommandExecutionManager()
    exec_manager.get_logger().info(
                'starting command execution manager' )
    
    while rclpy.ok():
        # exec_manager.get_logger().info(
                # 'starting rclpy loop' )
        exec_manager.send_request()
        rclpy.spin_once(exec_manager)

        # exec_manager.get_logger().info(exec_manager.future.done() )
        if exec_manager.future.done():
            try:
                response=exec_manager.future.result()
            except Exception as e:
                exec_manager.get_logger().info('Service call failed %r' % (e,))
            else:
                exec_manager.get_logger().info(
                    'Result of Command request %s' %
                    response.msg )
                if (response.msg=='ERROR'):
                    exec_manager.get_logger().info(
                    'List empty, trying again in 1 second')
                    time.sleep(1)
                else:    
                    exec_manager.send_order(response.msg)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    exec_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()