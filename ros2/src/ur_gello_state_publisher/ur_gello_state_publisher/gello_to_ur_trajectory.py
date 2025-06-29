# ur_joint_trajectory_sender.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
import os
import yaml
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class GelloToURTrajectory(Node):
    def __init__(self):
        super().__init__('gello_to_ur_trajectory')

        # Load config to get joint names
        self.joint_names = self.load_joint_names()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            JointState,
            '/gello/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            qos_profile
        )

        # Initialize timestamp before subscription
        self.last_publish_time = self.get_clock().now().nanoseconds

        self.get_logger().info("GelloToURTrajectory initialized and listening to /gello/joint_states")

    def load_joint_names(self):
        config_path = os.path.join(
            get_package_share_directory("ur_gello_state_publisher"),
            "config",
            "gello_config.yaml",
        )

        port = self.detect_port()
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        if port not in config:
            self.get_logger().error(f"Port {port} not found in config.")
            return []

        joint_names = config[port].get("joint_names", [])
        self.get_logger().info(f"Using joint names from config: {joint_names}")
        return joint_names

    def detect_port(self):
        import glob

        matches = glob.glob("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter*")
        if matches:
            return matches[0].split("/")[-1]
        return "usb-FTDI_USB__-__Serial_Converter_UNKNOWN"


    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now().nanoseconds
        dt = (now - self.last_publish_time) / 1e9  # seconds

        if dt < 0.1:  # only send at 10Hz 
            self.get_logger().debug(f"Skipping publish, dt={dt:.3f} seconds")
            # FIXME: Position control is a bit brittle since it can easily violate velocity limits 
            return
        self.last_publish_time = now

        if not all(j in msg.name for j in self.joint_names):
            self.get_logger().warn("JointState missing expected joint names")
            return

        joint_positions = [msg.position[msg.name.index(j)] for j in self.joint_names]

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)  # 0.2 seconds # FIXME: Needs to be configurable or dynamically calculated

        traj_msg.points.append(point)
        self.trajectory_publisher.publish(traj_msg)
        self.get_logger().debug(f"Published trajectory with positions: {joint_positions}")
    

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloToURTrajectory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception occurred: {e}")
        else:
            print(f"Exception occurred before node creation: {e}")
    finally:
        if rclpy.ok():
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
