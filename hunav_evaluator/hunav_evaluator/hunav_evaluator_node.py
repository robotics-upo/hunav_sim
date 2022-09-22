import numpy as np
import sys
import os

import rclpy
from rclpy.node import Node
from hunav_evaluator import hunav_metrics

from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

class HunavEvaluatorNode(Node):

    def __init__(self):
        super().__init__("hunav_evaluator_node")
        
        name = 'hunav_evaluator'
        self.agents_list = []
        self.robot_list = []
        self.robot_goal = None
        self.metrics_to_compute = {}

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

        
        self.declare_parameter('result_file', '1')
        self.result_file_path = self.get_parameter('result_file').get_parameter_value().string_value

        # Read metrics
        self.declare_parameter('experiment_tag', 'tag')
        self.exp_tag = self.get_parameter('experiment_tag').get_parameter_value().string_value
        self.declare_parameter('metrics', '')
        yaml_metrics = self.get_parameter('metrics').get_parameter_value().string_array_value
        self.get_logger().info("read metrics: %s" % yaml_metrics)
        for m in yaml_metrics:
            self.metrics_to_compute[m] = 0.0


        self.get_logger().info("Hunav evaluator:")
        self.get_logger().info("mode: %i" % self.mode)
        self.get_logger().info("freq: %.1f" % self.freq)
        self.get_logger().info("result_file: %s" % self.result_file_path)
        self.get_logger().info("experiment_tag: %s" % self.exp_tag)
        self.get_logger().info("Metrics:")
        for m in self.metrics_to_compute.keys():
            self.get_logger().info("m: %s, value: %s" % (m, self.metrics_to_compute[m]))

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
        # Subscribe to the robot goal to add it to the robot state
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        self.goal_sub


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
            robot_msg = msg
            if(self.robot_goal is not None):
                robot_msg.goals.clear()
                robot_msg.goals.append(self.robot_goal.pose)

            if(self.freq == 0.0):
                self.robot_list.append(robot_msg)
            else:
                self.robot = robot_msg


    def goal_callback(self, msg):
        self.robot_goal = msg


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
            response.message = 'Hunav recording started'
        

    def timer_end_callback(self):
        if(self.init == True):
            secs = (self.get_clock().now() - self.last_time).to_msg().sec
            #self.get_logger().info("secs: %.2f" % secs)
            if(secs >= self.time_period):
                self.recording == False
                self.get_logger().info("Hunav evaluator stopping recording!")
                self.compute_metrics()


    def timer_record_callback(self):
        if(self.recording == True and self.init == True):
            #self.get_logger().info("Saving data...")
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)


    def compute_metrics(self):
        self.check_data()
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        self.get_logger().info("Hunav evaluator. Collected %i messages of agents and %i of robot" % (agents_size, robot_size))
        self.get_logger().info("Computing metrics...")
        for m in self.metrics_to_compute.keys():
            self.metrics_to_compute[m] = hunav_metrics.metrics[m](self.agents_list, self.robot_list)
        print('Metrics computed:')
        print(self.metrics_to_compute)

        self.store_metrics()
        
        #self.agent_sub.destroy()
        #self.robot_sub.destroy()
        #self.end_timer.destroy()
        self.destroy_node()
        sys.exit()
        #return

    
    def store_metrics(self):

        # add extension if it does not have it
        if not self.result_file_path.endswith(".txt"):
            self.result_file_path += '.txt'

        file_was_created = os.path.exists(self.result_file_path)

        # open the file
        file = open(self.result_file_path,'a+')
        if(file is None):
            self.get_logger().error("RESULT METRICS FILE NOT CREATED! FILE: %s" % self.result_file_path)

        # if the file is new, create a header
        if file_was_created == False:
            file.write('experiment_tag')
            file.write('\t')
            for m in self.metrics_to_compute.keys():
                file.write(m)
                file.write('\t')
            file.write('\n')
        
        # write the data
        file.write(self.exp_tag)
        file.write('\t')
        for v in self.metrics_to_compute.values():
            file.write(str(v))
            file.write('\t')
        file.write('\n')
        file.close()


    def check_data(self):

        #First check the number of messages
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        if(abs(agents_size - robot_size) != 0):
            while(len(self.agents_list) > len(self.robot_list)):
                self.agents_list.pop()
            while(len(self.robot_list) > len(self.agents_list)):
                self.robot_list.pop()
            
        # check that the robot msg contains a goal?
        # check when the robot reaches the goal?


def main(args=None):
    rclpy.init(args=args)
    node = HunavEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
