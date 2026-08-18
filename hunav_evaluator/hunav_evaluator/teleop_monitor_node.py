import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs # To use do_transform_point
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TeleopMonitorNode(Node):
    def __init__(self):
        super().__init__('teleop_monitor_node')

        self.declare_parameter('topic_goal', '/hunav_goal_pose')
        self.topic_goal = self.get_parameter('topic_goal').value
        self.goal_pub = self.create_publisher(PoseStamped, self.topic_goal, 10)

        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10)

        self.declare_parameter('odom_topic', '/mobile_base_controller/odom')
        self.odom_topic = self.get_parameter('odom_topic').value

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)

        self.start_record_cli = self.create_client(StartEvaluation, '/hunav_start_recording')
        self.stop_record_cli = self.create_client(Empty, '/hunav_stop_recording')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.run_id = 1
        self.recording_active = False
        self.current_goal = None
        self.goal_tolerance = 0.5 # meters
        self.speed_tolerance = 0.03 # m/s

        self.get_logger().info('Teleop Monitor Node started. Waiting for /clicked_point...')

    def goal_callback(self, msg: PointStamped):
        self.current_goal = msg
        
        # Publish goal
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)
        #self.get_logger().info(f'Published goal: {goal_pose}')
        
        # Publish start recording
        if not self.start_record_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /hunav_start_recording not available yet...')
            
        req = StartEvaluation.Request()
        req.robot_goal = goal_pose
        req.experiment_tag = 'teleop'
        req.run_id = self.run_id
        self.run_id += 1
        
        self.start_record_cli.call_async(req)
        self.get_logger().info(f'Called /hunav_start_recording service. run_id: {req.run_id}')
        
        self.recording_active = True

    def odom_callback(self, msg: Odometry):
        if not self.recording_active or self.current_goal is None:
            return

        goal_point = self.current_goal
        
        # Transform goal coordinates if they are in different frames
        if self.current_goal.header.frame_id != msg.header.frame_id:
            try:
                # We use rclpy.time.Time() to get the latest available transform
                transform = self.tf_buffer.lookup_transform(
                    msg.header.frame_id,
                    self.current_goal.header.frame_id,
                    rclpy.time.Time())
                goal_point = tf2_geometry_msgs.do_transform_point(self.current_goal, transform)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Could not transform goal to {msg.header.frame_id} frame: {e}')
                return

        # Check distance to goal
        dx = msg.pose.pose.position.x - goal_point.point.x
        dy = msg.pose.pose.position.y - goal_point.point.y
        dist = math.sqrt(dx**2 + dy**2)

        # Check speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)

        if dist < self.goal_tolerance and speed < self.speed_tolerance:
            self.get_logger().info(f'Robot stopped near goal (dist: {dist:.2f}, speed: {speed:.2f}). Stopping recording...')
            self.stop_recording()
            
            self.recording_active = False
            self.current_goal = None

    def stop_recording(self):
        if not self.stop_record_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /hunav_stop_recording not available yet...')
            
        req = Empty.Request()
        self.stop_record_cli.call_async(req)
        self.get_logger().info('Called /hunav_stop_recording service.')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMonitorNode()
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
