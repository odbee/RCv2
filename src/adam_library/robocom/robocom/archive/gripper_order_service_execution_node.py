import rclpy
from adam_msgs.srv import SendOrderReturnResult
from adam_msgs.action import FollowGripper
from adam_msgs.srv import MoveGripper

from rclpy.node import Node
from rclpy.action import ActionClient
import socket
from action_msgs.msg import GoalStatus


class GripperOrderExecuter(Node):
    def __init__(self):
        super().__init__('gripper_order_executer')
        self.srv=self.create_service(SendOrderReturnResult, 'gripper_order',self.return_result_callback)
        self.cli=self.create_client(MoveGripper,"move_gripper")
        self._action_client = ActionClient(self, FollowGripper, 'followgripper')
        
        self.req=FollowGripper.Request()
    
    
    def return_result_callback(self, request, response):
        response = self.send_goal(request.order)
        
        return response


    
    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.currentposition))



    def move_gripper_execution(self,goal):
        goal= goal.replace(' ', '')
        goal= goal.split(',')
        goal[1]=int(goal[1])
        goal[2]=int(goal[2])
        goal[3]=int(goal[3])
        self.req.command = goal[0]
        self.req.position=goal[1]
        self.req.speed=goal[2]
        self.req.force=goal[3]
        self.future=self.cli.call_async(self.req)
        if self.future.done():
            try:
                response=self.future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                self.get_logger().info( 'Move successful' )
                return 0
        pass

    def send_goal(self, goal):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Waiting for action server done!')
        goal_msg = FollowGripper.Goal()
        self.get_logger().info('received string %s' %goal)
        goal= goal.replace(' ', '')
        goal= goal.split(',')
        goal[1]=int(goal[1])
        goal[2]=int(goal[2])
        goal[3]=int(goal[3])
        self.get_logger().info('edited string: %s' %goal)

        goal_msg.command = goal[0]
        goal_msg.position=goal[1]
        goal_msg.speed=goal[2]
        goal_msg.force=goal[3]
        
        self.get_logger().info('goal to be sent: %s' %goal_msg)
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        goal_handle=self._send_goal_future.result()
        
        # if not goal_handle.accepted:
        #     self.get_logger().info('Goal rejected')
        #     return
        
        self.get_logger().info('Goal accepted')
        
        get_result_future=goal_handle.get_result()
        result= get_result_future.result().result
        status= get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
            'Goal succeeded! Result: {0}'.format(result.returncode))
        else:
            self.get_logger().info('Goal failed with status code: {0}'.format(status))
        return result.returncode


def main(args=None):
    rclpy.init(args=args)

    gripper_order_node = GripperOrderExecuter()

    rclpy.spin(gripper_order_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_order_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

