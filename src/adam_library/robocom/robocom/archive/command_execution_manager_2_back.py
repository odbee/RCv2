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
        self.allowedToRun=True
        self.readytorequest=True
        #create client to get command from lists
        self.cli=self.create_client(ReceiveCommand, 'recv_cmd')
        self.remove_first_item_client=self.create_client(Empty, 'remove_first_cmd')
        #create clients to forward command to executers
        self.gripperclient=self.create_client(SendOrderReturnResult, 'gripper_order')
        self.robotclient=self.create_client(SendOrderReturnResult, 'robot_order')
        self.ocvclient=self.create_client(SendOrderReturnResult, 'ocv_service')
        
    
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command list service not available, waiting again...')
        self.req=ReceiveCommand.Request()

    def send_request(self):
        self.future=self.cli.call_async(self.req)

    def send_order(self,order):
        parsed_cmd= self.parse_message(order)
        
        if (parsed_cmd["commandtype"]=="gripper") and self.readytorequest==True:
            self.get_logger().info('sending gripper request')
            self.gripperorder = SendOrderReturnResult.Request()
            while not self.gripperclient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('gripper service not available, waiting again...')
            self.gripperorder.order=parsed_cmd["command"]
            self.future_order=self.gripperclient.call_async(self.gripperorder)
            rclpy.spin_until_future_complete(self,self.future_order)
            self.readytorequest=False

        if (parsed_cmd["commandtype"]=="urscript") and self.readytorequest==True:
            self.gripperorder = SendOrderReturnResult.Request()
            while not self.gripperclient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('robot service not available, waiting again...')
            self.gripperorder.order=parsed_cmd["command"]
            self.get_logger().info('sending ur order: %s' %self.gripperorder.order)
            self.future_order=self.robotclient.call_async(self.gripperorder)
            rclpy.spin_until_future_complete(self,self.future_order)
            self.readytorequest=False
            
            pass

        if (parsed_cmd["commandtype"]=="opencv") and self.readytorequest==True:
            self.gripperorder = SendOrderReturnResult.Request()
            while not self.gripperclient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('robot service not available, waiting again...')
            self.gripperorder.order=parsed_cmd["command"]
            self.get_logger().info('sending ur order: %s' %self.gripperorder.order)
            self.future_order=self.ocvclient.call_async(self.gripperorder)
            rclpy.spin_until_future_complete(self,self.future_order)
            self.readytorequest=False
            
            pass

        

        if self.future_order.done():
            
            resultcommand=self.future_order.result()
            self.get_logger().info(
                'Result of Gripper Order %s' %
                str(resultcommand) )
            emptyreq = Empty.Request()
            # self.nofuture=self.remove_first_item_client.call_async(emptyreq)
            self.allowedToRun=True
            self.readytorequest=True

    def parse_message(self,message):
        parsed_msg = json.loads(message)
        return parsed_msg


def main(args=None):
    rclpy.init(args=args)

    exec_manager = CommandExecutionManager()
    exec_manager.get_logger().info(
                'starting command execution manager' )
    
    while rclpy.ok():

        if (exec_manager.allowedToRun):
            exec_manager.allowedToRun=False    
            exec_manager.future=exec_manager.cli.call_async(exec_manager.req)
            rclpy.spin_until_future_complete(exec_manager,exec_manager.future)
            
        
        if exec_manager.future.done():
            result = exec_manager.future.result()
            # exec_manager.get_logger().info('Result of Command request %s' %result.msg )
            if (result.msg=='ERROR'):
                #exec_manager.get_logger().info(#'List empty, trying again in 1 second')
                time.sleep(1)
                exec_manager.allowedToRun=True

            else:    
                exec_manager.send_order(result.msg)
                pass

    exec_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()