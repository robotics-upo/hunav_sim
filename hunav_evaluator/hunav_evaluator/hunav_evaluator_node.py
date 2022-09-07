import numpy as np
import sys

import rclpy
from rclpy.node import Node

from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from std_srvs.srv import Trigger

class HunavEvaluatorNode(Node):

    def __init__(self):
        super().__init__("hunav_evaluator_node")
        
        self.agents_list = []
        self.robot_list = []

        # Two modes:
        # 1- The user start/stop the recording through the
        #    the service /hunav_trigger_recording
        # 2- The recording start/stop process is automatic.
        #    It starts when the first topic is received.
        #    It stops when a certain time pass without receiving data. 
        self.mode = self.declare_parameter('mode', 2).get_parameter_value().integer_value

        # Indicate the frequency of capturing the data 
        # (it must be slower than data publishing).
        # If the value is set to zero, the data is captured
        # at the same frequency than it is published.
        self.freq = self.declare_parameter('frequency', 0.0).get_parameter_value().double_value

        self.get_logger().info("Hunav evaluator:")
        self.get_logger().info("mode: %i" % self.mode)
        self.get_logger().info("freq: %.1f" % self.freq)

        if(self.freq > 0.0):
            self.agents = Agents()
            self.robot = Agent()
            self.record_timer = self.create_timer(1/self.freq, self.timer_record_callback)

        if(self.mode == 1):
            self.recording = False
            self.recording_srv = self.create_service(Trigger, 'hunav_trigger_recording', self.recording_service)
        elif(self.mode == 2):
            self.recording = True
            self.time_period = 3.0  # seconds
            self.last_time = self.get_clock().now()
            self.init = False
            self.end_timer = self.create_timer(1.0, self.timer_end_callback)
            #self.end_timer.cancel()
        else:
            self.get_logger().error("Mode not recognized. Only modes 1 or 2 are allowed")

        self.agent_sub = self.create_subscription(Agents, 'human_states', self.human_callback, 1)
        self.agent_sub  # prevent unused variable warning
        self.robot_sub = self.create_subscription(Agent, 'robot_states', self.robot_callback, 1)
        self.robot_sub


    def human_callback(self, msg):
        if(self.mode == 2):
            self.init = True
            self.last_time = self.get_clock().now()
            #self.end_timer.reset()
            #self.get_logger().info("reseting1")
        if(self.recording == True):
            #self.get_logger().info("human received")
            if(self.freq == 0.0):
                self.agents_list.append(msg)
            else:
                self.agents = msg

    def robot_callback(self, msg):
        if(self.mode == 2):
            self.init = True
            self.last_time = self.get_clock().now()
            #self.end_timer.reset()
            #self.get_logger().info("reseting2")
        if(self.recording == True):
            #self.get_logger().info("robot received")
            if(self.freq == 0.0):
                self.robot_list.append(msg)
            else:
                self.robot = msg

    def recording_service(self, request, response):
        response.success = True
        if(self.recording == True):
            self.get_logger().info("Hunav evaluator stopping recording!")
            self.recording = False
            response.message = 'Hunav recording stopped'
            #self.agent_sub.destroy()
            #self.robot_sub.destroy()
            self.compute_metrics()
        else:
            self.get_logger().info("Hunav evaluator started recording!")
            self.recording = True
            self.get_logger().info("Hunav evaluator started recording 2222")
            response.message = 'Hunav recording started'
            self.get_logger().info("Hunav evaluator started recording 3333")
        

    def timer_end_callback(self):
        if(self.init == True):
            secs = (self.get_clock().now() - self.last_time).to_msg().sec
            self.get_logger().info("secs: %.2f" % secs)
            if(secs >= self.time_period):
                self.recording == False
                self.get_logger().info("Hunav evaluator stopping recording!")
                self.compute_metrics()


    def timer_record_callback(self):
        if(self.recording == True):
            self.get_logger().info("Saving data...")
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)


    def compute_metrics(self):
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        self.get_logger().info("Hunav evaluator. Collected %i messages of agents and %i of robot" % (agents_size, robot_size))
        #self.agent_sub.destroy()
        #self.robot_sub.destroy()
        #self.end_timer.destroy()
        self.destroy_node()
        sys.exit()
        #return


def main(args=None):
    rclpy.init(args=args)
    node = HunavEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
