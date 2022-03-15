import time
from threading import Event
from adam_msgs.srv import MoveGripper
from adam_msgs.srv import SendOrderReturnResult
import rclpy
from rclpy import callback_groups
from rclpy import executors
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
# TODO CHECK WHEN GRIPPER COMMAND INCOMPATIBLE
class ServiceFromService(Node):
    def __init__(self):
        super().__init__('service_from_service')
        self.service_done_event=Event()
        self.callback_group=ReentrantCallbackGroup()
        self.srv=self.create_service(
            SendOrderReturnResult,
            'gripper_order',
            self.return_result_callback)
        self.cli=self.create_client(
            MoveGripper,
            "move_gripper",
            callback_group=self.callback_group
            )
    
    def return_result_callback(self, request,response):
        self.get_logger().info('Received Gripper Order.')
        
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No Gripper service available')
            return 1
        self.get_logger().info('Gripper Service Available.')
        self.get_logger().info('clearing event.')
        self.service_done_event.clear()
        self.get_logger().info('cleared event.')
        event=Event()
        def done_callback(future):
            nonlocal event
            event.set()
        
        self.req=MoveGripper.Request()
        self.get_logger().info('parsing request: %s' % request)
        goal=self.parse_request(request.order)
        self.get_logger().info('parsed request!, result %s' %str(goal))
        
        self.req.command = goal[0]
        self.req.position=goal[1]
        self.req.speed=goal[2]
        self.req.force=goal[3]
        self.get_logger().info('Sending Future Call.')
        future=self.cli.call_async(self.req)
        future.add_done_callback(done_callback)
        self.get_logger().info('Waiting for Callback to return a result')
        event.wait()
        result= future.result()
        self.get_logger().info('received result: %s'%result)
        response.result=0
        return response
    
    def parse_request(self, goal):
        goal= goal.replace(' ', '')
        goal= goal.split(',')
        goal[1]=int(goal[1])
        goal[2]=int(goal[2])
        goal[3]=int(goal[3])
        return goal

def main(args=None):
    rclpy.init(args=args)
    service_from_service=ServiceFromService()
    executor=MultiThreadedExecutor()
    rclpy.spin(service_from_service,executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()