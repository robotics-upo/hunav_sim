import numpy as np
import sys
import os

import rclpy
from rclpy.node import Node
from hunav_evaluator import hunav_metrics

from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
import pandas as pd


class HunavEvaluatorNode(Node):

    def __init__(self):
        super().__init__("hunav_evaluator_node")

        self.agents_list = []
        self.robot_list = []
        self.robot_goal = None
        self.metrics_to_compute = {}
        self.metrics_lists = {}
        self.number_of_behaviors = 6

        # The user start/stop the recording through the
        #    the services /hunav_start_recording and
        #    /hunav_stop_recording.

        # Indicate the frequency of capturing the data
        # (it must be slower than data publishing).
        # If the value is set to zero, the data is captured
        # at the same frequency than it is published.
        self.freq = (
            self.declare_parameter("frequency", 0.0).get_parameter_value().double_value
        )

        # base name of the result files
        self.declare_parameter("result_file", "metrics")

        self.metrics_to_compute = self.get_metrics_to_compute()

        self.get_logger().info("Hunav evaluator:")
        self.get_logger().info(f"freq: {self.freq:.2f} Hz")
        self.get_logger().info("Metrics:")
        for m in self.metrics_to_compute.keys():
            self.get_logger().info(f"   {m}")

        if self.freq > 0.0:
            self.agents = Agents()
            self.robot = Agent()
            self.record_timer = self.create_timer(
                1 / self.freq, self.timer_record_callback
            )

        self.recording = False
        self.recording_service_start = self.create_service(
            StartEvaluation, "hunav_start_recording", self.recording_service_start
        )
        self.recording_service_stop = self.create_service(
            Empty, "hunav_stop_recording", self.recording_service_stop
        )

        self.agent_sub = self.create_subscription(
            Agents, "human_states", self.human_callback, 1
        )
        self.robot_sub = self.create_subscription(
            Agent, "robot_states", self.robot_callback, 1
        )

    def recording_service_start(
        self, request: StartEvaluation.Request, response: StartEvaluation.Response
    ):
        if self.recording == True:
            self.get_logger().warn("Hunav evaluator already recording!")
            response.success = False
        else:
            self.get_logger().info("Hunav evaluator started recording!")
            self.recording = True
            self.robot_goal = (
                request.robot_goal if request.robot_goal != PoseStamped() else None
            )
            self.exp_tag = request.experiment_tag
            self.run_id = request.run_id
            self.agents_list.clear()
            self.robot_list.clear()
            self.last_time = self.get_clock().now()
            response.success = True
        return response

    def recording_service_stop(self, request, response):
        if self.recording == False:
            self.get_logger().warn("Hunav evaluator not recording!")
            return response

        self.get_logger().info("Hunav evaluator stopping recording!")
        self.recording = False
        self.compute_metrics()
        return response

    def human_callback(self, msg: Agents):
        """Callback for the human agents data.
        If the frequency is set to zero, the data is stored
        at the same frequency than it is published.
        If the frequency is greater than zero, the data is stored
        at the specified frequency.

        Args:
            msg (Agents): The message containing the human agents data."""
        if self.recording:
            if self.freq == 0.0:
                self.agents_list.append(msg)
            else:
                self.agents = msg

    def robot_callback(self, msg: Agent):
        """Callback for the robot data.
        If the frequency is set to zero, the data is stored
        at the same frequency than it is published.
        If the frequency is greater than zero, the data is stored
        at the specified frequency.
        Args:
            msg (Agent): The message containing the robot data."""
        if self.recording:
            robot_msg = msg
            if self.robot_goal is not None:
                robot_msg.goals.clear()
                robot_msg.goals.append(self.robot_goal.pose)
                robot_msg.goal_radius = 0.2

            if self.freq == 0.0:
                self.robot_list.append(robot_msg)
            else:
                self.robot = robot_msg

    def timer_record_callback(self):
        """Timer callback to record the data at the specified frequency
        this is only used if the frequency is greater than zero."""
        if self.recording == True:
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)

    def compute_metrics(self):
        """Compute the metrics based on the collected data.
        This method checks if the data is valid, computes the metrics
        for all agents, and then filters the data according to the
        different behaviors."""
        if not self.check_data():
            self.get_logger().warn("Data not collected. Not computing metrics.")
            return
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        self.get_logger().info(
            f"Hunav evaluator. Collected {agents_size} messages of agents and {robot_size} of robot"
        )
        self.get_logger().debug("Computing metrics...")

        # compute metrics for all agents
        # metrics_lists is a dictionary that will contain all the metrics
        self.metrics_lists["time_stamps"] = hunav_metrics.get_time_stamps(
            self.agents_list, self.robot_list
        )

        # for each metric, compute the value
        for m in self.metrics_to_compute.keys():
            # call the metric function with the agents and robot lists
            metric = hunav_metrics.metrics[m](self.agents_list, self.robot_list)
            #
            self.metrics_to_compute[m] = metric[0]
            # if the metric function returns more than one value,
            # store the second value in the metrics_lists dictionary
            if len(metric) > 1:
                self.metrics_lists[m] = metric[1]

        rclpy.logging.get_logger("hunav_evaluator").info(
            f"Metrics computed: {self.metrics_to_compute.keys()}"
        )
        self.store_metrics(self.result_file_path)

        # Now, filter according to the different behaviors
        for i in range(1, (self.number_of_behaviors + 1)):
            self.compute_metrics_behavior(i)

    def get_metrics_to_compute(self) -> dict:
        """Get the metrics to compute based on parameters.

        Returns:
            dict: A dictionary with the metrics to compute."""
        metrics_to_compute = {}
        for m in hunav_metrics.metrics.keys():
            ok = (
                self.declare_parameter("metrics." + m, True)
                .get_parameter_value()
                .bool_value
            )
            if ok:
                metrics_to_compute[m] = 0.0
        return metrics_to_compute

    def compute_metrics_behavior(self, behavior: int):
        """Compute the metrics for a specific behavior

        Args:
            behavior (int): The behavior to compute the metrics for.
        """

        # first, get the agents with the indicated behavior
        beh_agents = []  # list of Agents with the behavior
        beh_robot = []  # list of Agent with the behavior
        beh_active = [0] * len(
            self.agents_list
        )  # list to indicate if the behavior is active for each agent
        i = 0  # index for the agents and robots
        for la, lr in zip(
            self.agents_list, self.robot_list
        ):  # la is the Agents message, lr is the Agent message
            ag = Agents()  # create a new Agents message
            ag.header = la.header  # copy the header from the Agents message
            for a in la.agents:  # iterate over the agents in the Agents message
                if a.behavior == behavior:  # check if the agent has the behavior
                    ag.agents.append(a)  # add the agent to the Agents message
                if (
                    a.behavior.state != a.behavior.BEH_NO_ACTIVE
                ):  # check if the agent is active
                    beh_active[i] = 1
            if len(ag.agents) > 0:  # if there are agents with the behavior
                beh_agents.append(ag)
                beh_robot.append(lr)
            else:
                rclpy.logging.get_logger("hunav_evaluator").debug(
                    f"No agents of behavior {behavior} found at step {i}."
                )
                return
            i += 1

        self.metrics_lists["behavior_active"] = (
            beh_active  # store the behavior active list
        )
        # then, compute the metrics for those agents
        for m in self.metrics_to_compute.keys():  # iterate over the metrics to compute
            metric = hunav_metrics.metrics[m](beh_agents, beh_robot)
            self.metrics_to_compute[m] = metric[0]
            if len(metric) > 1:  # if the metric function returns more than one value,
                self.metrics_lists[m] = metric[1]

        rclpy.logging.get_logger("hunav_evaluator").debug(
            f"Metrics computed for behavior {behavior}: {self.metrics_to_compute}"
        )
        store_file = self.result_file_path  # base name of the result file
        if not store_file.endswith(".csv"):
            store_file += ".csv"
        store_file = store_file.replace(".csv", f"_beh_{behavior}.csv")

        self.store_metrics(store_file)  # store the metrics in a file

    def store_metrics(self, result_file: str):
        """Store the computed metrics in a file."""

        if not result_file.endswith(".csv"):
            result_file += ".csv"  # ensure the file has a .csv extension
        # add extension if it does not have it

        file_was_created = os.path.exists(result_file)
        # be sure thath the parent directory exists
        os.makedirs(os.path.dirname(result_file), exist_ok=True)

        df_metrics = pd.DataFrame(self.metrics_to_compute, index=[self.exp_tag])
        df_metrics.index.name = "experiment_tag"
        df_metrics["run_id"] = self.run_id  # add the run id to the metrics

        # save the metrics to a CSV file
        df_metrics.to_csv(result_file, mode="a", header=not file_was_created)
        # open and write the second file (metric for each step)

        df_steps = pd.DataFrame(self.metrics_lists)
        df_steps.index.name = "time_stamps"
        # save the steps to a CSV file
        steps_csv_file = result_file.replace(
            ".csv", f"_steps_{self.exp_tag}_{self.run_id}.csv"
        )
        df_steps.to_csv(steps_csv_file, index=True)
        self.get_logger().info(
            f"Metrics steps stored in {result_file} and {steps_csv_file}"
        )

    def check_data(self) -> bool:
        """Check that the data is valid for computing the metrics."""
        # check if list is empty
        min_length = min(len(self.agents_list), len(self.robot_list))
        if min_length == 0:
            self.get_logger().error("No data collected. Cannot compute metrics.")
            return False
        # resize the lists to the minimum length
        self.agents_list = self.agents_list[:min_length]
        self.robot_list = self.robot_list[:min_length]
        return True

    @property
    def result_file_path(self):
        """Get the result file path from the parameter."""
        return self.get_parameter("result_file").get_parameter_value().string_value


def main(args=None):
    rclpy.init(args=args)
    node = HunavEvaluatorNode()
    try:
        node.get_logger().info("Hunav evaluator node started.")
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        if rclpy.ok():
            node.get_logger().info("Shutting down Hunav evaluator node...")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
