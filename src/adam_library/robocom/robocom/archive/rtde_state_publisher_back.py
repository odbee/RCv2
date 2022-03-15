from os import terminal_size
import rclpy
from rclpy.node import Node


from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from adam_msgs.srv import SendOrderReturnResult

import rtde.serialize as serialize
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time 


# TODO MAKE ASYNCH SO I CAN ACCESS ACTIVE WHILE CALLBACK RUNNING
class RTDEStatePublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        
        
        self.service_done_event=Event()
        self.callback_group = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(String, 'topic', 10,
            callback_group=self.callback_group)
        self.srv=self.create_service(
            SendOrderReturnResult,
            'axis_order',
            self.return_result_callback,
            callback_group=self.callback_group)

        self.sampling_frequency=125
        timer_period = 1/self.sampling_frequency  # seconds
        dir = get_package_share_directory('robocom')
        self.get_logger().info('%s' % dir)
        self.msg=String()
        self.cachedval=''
        self.ROBOT_HOST_IP = '192.168.1.104'
        self.RTDE_PORT = 30004
        self.config_file=dir + '/record_configuration.xml'
        
        self.conf=rtde_config.ConfigFile(self.config_file)
        self.output_names, self.output_types = self.conf.get_recipe('out')
        self.setp_names, self.setp_types = self.conf.get_recipe('setp')
        
        self.rtdedict = {}
        self.rtde_connection = rtde.RTDE(self.ROBOT_HOST_IP, self.RTDE_PORT)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rtde_connection.connect()

        self.setp= self.rtde_connection.send_input_setup(self.setp_names, self.setp_types)
                # setup recipes
        if not self.rtde_connection.send_output_setup(self.output_names, self.output_types, frequency = self.sampling_frequency):
            self.get_logger().error('Unable to configure output')
            return 10
            
        #start data synchronization
        if not self.rtde_connection.send_start():
            self.get_logger().error("Unable to start synchronization")
            return 10

    def timer_callback(self):
        try:
            state=self.rtde_connection.receive()
            if state is not None:
                
                val=''
                for i in range(len(self.output_names)):
                    self.rtdedict[self.output_names[i]]=state.__dict__[self.output_names[i]]
                val=state.__dict__["script_control_line"]
                # self.get_logger().info(' rtdedict: %s, state: %s' % (str(self.rtdedict), str(state.__dict__))  )
                output_bits=format(self.rtdedict['actual_digital_output_bits'],"018b")
                output_bits=output_bits[len(output_bits)::-1]
                input_bits=format(self.rtdedict['actual_digital_input_bits'],"018b")
                input_bits=input_bits[len(input_bits)::-1]
                self.active=input_bits[2]
                self.referenced=input_bits[3]
                # self.get_logger().info('output bits: %s, input bits: %s ' % (output_bits, input_bits)  )
                
                self.publisher_.publish(self.msg)
                # self.get_logger().info('Publishing: "%s"' % self.msg.data)
                
                if self.cachedval != val:
                    self.cachedval=val
                    self.msg.data=str(val)
                    # self.publisher_.publish(self.msg)
                    self.get_logger().info('Publishing: "%s"' % self.msg.data)

        except rtde.RTDEException:
            self.rtde_connection.disconnect()
            self.get_logger().error("ERROR with connection")    

    def timer_callback2(self):
        try:
            state=self.rtde_connection.receive()
            if state is not None:
                
                val=''
                for i in range(len(self.output_names)):
                    val=state.__dict__[self.output_names[i]]
                self.publisher_.publish(self.msg)
                # self.get_logger().info('Publishing: "%s"' % self.msg.data)
                if self.cachedval != val:
                    self.cachedval=val
                    self.msg.data=str(val)
                    # self.publisher_.publish(self.msg)
                    self.get_logger().info('Publishing: "%s"' % self.msg.data)

        except rtde.RTDEException:
            self.rtde_connection.disconnect()
            self.get_logger().error("ERROR with connection")    

    def return_result_callback(self,request,response):
        # self.service_done_event.set()
        if self.referenced==False:
            self.get_logger().info('not yet referenced, sending home command first')
            self.move_axis("1")
        
        self.move_axis(request.order)
        # self.service_done_event.clear()

        return response
    
    def move_axis(self, order):
        self.get_logger().error("moving axis")           
        self.setp.__dict__["standard_digital_output_mask"]=2*int(order)-2
        self.setp.__dict__["standard_digital_output"]=2*int(order)-2

        self.rtde_connection.send(self.setp)
        time.sleep(0.1)
        self.setp.__dict__["standard_digital_output_mask"]=2**6
        self.setp.__dict__["standard_digital_output"]=2**6
        self.rtde_connection.send(self.setp)
        time.sleep(0.1)
        self.setp.__dict__["standard_digital_output_mask"]=2**6
        self.setp.__dict__["standard_digital_output"]=0
        self.rtde_connection.send(self.setp)
        time.sleep(0.1)
        self.setp.__dict__["standard_digital_output_mask"]=2*int(order)-2
        self.setp.__dict__["standard_digital_output"]=0
        self.rtde_connection.send(self.setp)
        while self.active:
            time.sleep(0.1)
        return 
    

def main(args=None):
    rclpy.init(args=args)

    state_publisher = RTDEStatePublisher()
    executor=MultiThreadedExecutor()

    rclpy.spin(state_publisher, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()