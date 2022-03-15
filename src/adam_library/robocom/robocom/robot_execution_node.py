# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#  gripper communication:
import socket
import threading
import time
# end gripper communication

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from adam_msgs.srv import SendOrderReturnResult

from std_msgs.msg import String

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class RobotControlServer(Node):

    def __init__(self):

        ############################# START INIT UR VARIABLES
        self.ROBOT_HOST_IP = '192.168.1.104'
        self.RTDE_PORT = 30004

        self.SECONDARY_PORT=30002
        self.robot_status=0

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ROBOT_HOST_IP, self.SECONDARY_PORT))
        ############################## END INIT UR VARIABLES

        self.isRunning=False
        isIdle=True

        self.service_done_event=Event()
        self.callback_group = ReentrantCallbackGroup()

        super().__init__('robot_control_server')
        self.srv=self.create_service(SendOrderReturnResult,
            'robot_order',
            self.move_robot_callback,
            callback_group=self.callback_group)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10,
            callback_group=self.callback_group)
        self.subscription

    def send_this_command_urscript(self,comm):

        comm = comm + " \n"
        self.s.send(comm.encode())


        return
    
    def listener_callback(self,msg):
        # self.get_logger().info('Callback Returned from Robot State Publisher %s' %msg.data)
        self.robot_status=msg.data
        # self.get_logger().info('Callback Returned from Robot State Publisher %s' %msg.data)
        if self.isRunning == False and self.robot_status=="1":
            self.get_logger().info('Set Robot to Running')
            self.isRunning = True
        if self.isRunning == True and self.robot_status=="0":
            self.get_logger().info('Set Robot to Idle, releasing to run new command')
            self.isRunning = False
            self.service_done_event.set()

 
        
        pass

    def move_robot_callback(self,request,response):
        self.get_logger().info('Sending UR Command to Execution...')
        self.send_this_command_urscript(request.order)
        self.get_logger().info('Sent UR Command to Execution')
        self.service_done_event.clear()
        self.get_logger().info('Event Cleared')
        self.service_done_event.wait(timeout=60.0)
        self.get_logger().info('Event Released')
        
        
        

        self.get_logger().info('set response')

        return response





def main(args=None):
    rclpy.init(args=args)
    robot_control_server=RobotControlServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(robot_control_server, executor)
    
    robot_control_server.s.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()