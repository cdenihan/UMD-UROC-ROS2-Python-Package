import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sys

def ned_to_flu_xyz(xyz_enu):
    # ENU→FLU conversion if needed; here identity
    return list(xyz_enu)

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'  # match your TF root
        self.path_pub = self.create_publisher(Path, '/drone/flight_path', 10)

        # State
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]

        # QoS matching MAVROS publisher (BEST_EFFORT)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to MAVROS pose with matching QoS
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos
        )

        # Timer for publishing transforms, pose, and path
        self.timer = self.create_timer(0.01, self.publish_loop)

    def mavros_pose_callback(self, msg: PoseStamped):
        # Receive ENU pose
        self.drone_pos = [msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z]
        self.drone_q = [msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        self.latest_header = msg.header

    def publish_loop(self):
        # Use latest header timestamp, fallback to now
        if hasattr(self, 'latest_header'):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # Broadcast TF: map->drone_frame
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.path.header.frame_id
        tf_msg.child_frame_id = 'drone_frame'
        tf_msg.transform.translation.x = self.drone_pos[0]
        tf_msg.transform.translation.y = self.drone_pos[1]
        tf_msg.transform.translation.z = self.drone_pos[2]
        tf_msg.transform.rotation.x = self.drone_q[0]
        tf_msg.transform.rotation.y = self.drone_q[1]
        tf_msg.transform.rotation.z = self.drone_q[2]
        tf_msg.transform.rotation.w = self.drone_q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Current PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.path.header.frame_id
        pose_msg.pose.position.x = self.drone_pos[0]
        pose_msg.pose.position.y = self.drone_pos[1]
        pose_msg.pose.position.z = self.drone_pos[2]
        pose_msg.pose.orientation.x = self.drone_q[0]
        pose_msg.pose.orientation.y = self.drone_q[1]
        pose_msg.pose.orientation.z = self.drone_q[2]
        pose_msg.pose.orientation.w = self.drone_q[3]
        self.drone_pose_pub.publish(pose_msg)

        # Append to Path and publish
        self.path.poses.append(pose_msg)
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)


def main(args=None):
    print("UROC Foxglove 3D Visualization Node Initiated. Press CTRL-C to exit.")
    rclpy.init(args=args)
    node = VisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting Down UROC Foxglove 3D Visualization Node")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
