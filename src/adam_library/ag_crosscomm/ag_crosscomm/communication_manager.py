import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CommunicationManager(Node):

    def __init__(self):
        super().__init__('communication_manager')
        self._action_client = ActionClient(self, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    
    def send_goal(self):
        self.get_logger().info('starting send goal')
        point = JointTrajectoryPoint()
        point.positions=[0.785, -1.57, 0.785, 0.785, 0.785, 0.785]
        point.time_from_start=Duration(sec=4)

        traj = JointTrajectory()
        traj.joint_names=[ "shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ]
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory=traj
        goal_msg.goal_time_tolerance=Duration(sec=4)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_a,sync(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print("moshi")
        return
    
    def goal_response_callback(self,future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected!')
            return
        
        self.get_logger().info('Goal Accepted')
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self,future):
        result=future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))
        rclpy.shutdown()
    def feedback_callback(self,feedback_msg):
        feedback=feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.actual))

    


def main(args=None):
    rclpy.init(args=args)

    action_client = CommunicationManager()

    action_client.send_goal()

    rclpy.spin(action_client)

if  __name__== '__main__':
    main()