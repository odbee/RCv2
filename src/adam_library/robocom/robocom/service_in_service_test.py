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
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No Gripper service available')
            return response
        self.service_done_event.clear()
        event=Event()
        def done_callback(future):
            nonlocal event
            event.set()
        request
        
        future=self.cli.call_async(request)
        future.add_done_callback(done_callback)
        event.wait()
        result= future.result()
    
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