import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray, GoalStatus
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty

class ActionMonitorNode(Node):
    def __init__(self):
        super().__init__('action_monitor_node')

        self.goal_pub = self.create_publisher(PoseStamped, '/hunav_goal_pose', 10)

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10)

        self.start_record_cli = self.create_client(StartEvaluation, '/hunav_start_recording')
        self.stop_record_cli = self.create_client(Empty, '/hunav_stop_recording')

        self.run_id = 1

        self.current_goal_id = None
        self.recording_active = False
        self.goal_pose_published = False

        self.get_logger().info('Action Monitor Node started. Waiting for Nav2 goals...')

    def status_callback(self, msg: GoalStatusArray):
        # Check if our current goal has finished
        active_goal = None
        
        for status in msg.status_list:
            if self.current_goal_id is not None and status.goal_info.goal_id == self.current_goal_id:
                # Check if it finished
                if status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                    self.get_logger().info('Navigation finished. Stopping recording...')
                    self.stop_recording()
                    self.current_goal_id = None
                    self.recording_active = False
                    self.goal_pose_published = False
                break
                
            # If we don't have a current goal, look for a new one that is starting
            if self.current_goal_id is None:
                if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                    active_goal = status

        # If we found a new active goal and we weren't tracking anything
        if self.current_goal_id is None and active_goal is not None:
            self.get_logger().info('New navigation goal detected. Waiting for plan to extract goal pose...')
            self.current_goal_id = active_goal.goal_info.goal_id
            self.goal_pose_published = False
            self.recording_active = False

    def plan_callback(self, msg: Path):
        # If we have an active goal but we haven't published its pose yet
        if self.current_goal_id is not None and not self.goal_pose_published:
            if len(msg.poses) > 0:
                goal_pose = msg.poses[-1]
                
                # Publish to /hunav_goal_pose
                self.goal_pub.publish(goal_pose)
                self.get_logger().info('Goal pose published to /hunav_goal_pose')
                
                # Start recording
                self.start_recording(goal_pose)
                
                self.goal_pose_published = True
                self.recording_active = True
                
    def start_recording(self, goal_pose: PoseStamped):
        if not self.start_record_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /hunav_start_recording not available yet...')
            
        req = StartEvaluation.Request()
        req.robot_goal = goal_pose
        req.experiment_tag = 'nav2_experiment'
        req.run_id = self.run_id
        self.run_id += 1
        
        # We don't block waiting for response to avoid hanging the node's callbacks
        self.start_record_cli.call_async(req)
        self.get_logger().info('Called /hunav_start_recording service.')

    def stop_recording(self):
        if not self.stop_record_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /hunav_stop_recording not available yet...')
            
        req = Empty.Request()
        self.stop_record_cli.call_async(req)
        self.get_logger().info('Called /hunav_stop_recording service.')

def main(args=None):
    rclpy.init(args=args)
    node = ActionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
