import os
import sys
import glob
import time
import yaml
import numpy as np
from typing import Dict, Optional, Tuple, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# Setup import path for gello framework
gello_path = os.path.abspath(
    os.path.join(get_package_prefix("ur_gello_state_publisher"), "../../../")
)
sys.path.insert(0, gello_path)

from gello.agents.gello_agent import DynamixelRobotConfig
from gello.agents.agent import Agent


class URGelloAgent(Agent):
    """Wrapper around Dynamixel-based Gello robot for UR5e."""

    def __init__(self, port: str, config: DynamixelRobotConfig):
        self._robot = config.make_robot(port=port)

    def get_joint_state(self) -> np.ndarray:
        return self._robot.get_joint_state()

    def act(self, obs):
        """Return the current joint state as the action."""
        # NOTE: Required for Agent interface compliance, not required if using ros2_control or just publishing joint states.
        return self._robot.get_joint_state() 

class GelloPublisher(Node):
    def __init__(self):
        super().__init__("gello_publisher")

        self.com_port = self.declare_and_get_com_port()
        self.port = os.path.basename(self.com_port)

        self.config = self.load_yaml_config()
        port_cfg = self.config[self.port]

        self.num_robot_joints = port_cfg["num_joints"]
        self.joint_names = tuple(port_cfg["joint_names"])
        self.joint_signs = port_cfg["joint_signs"]
        self.best_offsets = np.array(port_cfg["best_offsets"])
        self.gripper = port_cfg.get("gripper", False)

        # Optional params
        self.gripper_config = tuple(port_cfg.get("gripper_config", [7, 200, 158]))
        self.baudrate = port_cfg.get("baudrate", 57600)
        config_rate = port_cfg.get("publish_rate", -1.0)

        self.__post_init__()

        # Load Agent
        custom_cfg = DynamixelRobotConfig(
            joint_ids=tuple(range(1, self.num_robot_joints + 1)),
            joint_offsets=tuple(self.best_offsets),
            joint_signs=tuple(self.joint_signs),
            gripper_config=self.gripper_config,
        )
        self.robot = URGelloAgent(port=self.com_port, config=custom_cfg)

        # Resolve publish rate
        user_rate = self.declare_and_get_publish_rate()
        estimate = self.estimate_driver_polling_rate()
        self.publish_rate = self.resolve_publish_rate(user_rate, config_rate, estimate)
        self.get_logger().info(f"Using publish rate: {self.publish_rate:.1f} Hz")

        # ROS 2 publishers
        qos = QoSProfile(depth=10)
        self.robot_joint_publisher = self.create_publisher(JointState, "/gello/joint_states", qos)
        self.gripper_joint_publisher = self.create_publisher(Float32, "/gripper_client/target_gripper_width_percent", qos)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_jog)

    def __post_init__(self):
        assert len(self.joint_signs) == self.num_robot_joints
        for i, sign in enumerate(self.joint_signs):
            assert sign in (-1, 1), f"Joint sign for joint {i} must be -1 or 1"

    def declare_and_get_com_port(self) -> str:
        matches = glob.glob("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter*")
        default_port = matches[0] if matches else "INVALID_COM_PORT"
        if matches:
            self.get_logger().info(f"Auto-detected com_ports: {matches}")
        else:
            self.get_logger().warn("No com_ports detected. Please specify manually.")
        self.declare_parameter("com_port", default_port)
        return self.get_parameter("com_port").get_parameter_value().string_value

    def declare_and_get_publish_rate(self) -> float:
        self.declare_parameter("publish_rate", -1.0)
        return self.get_parameter("publish_rate").get_parameter_value().double_value

    def load_yaml_config(self) -> dict:
        config_path = os.path.join(
            get_package_share_directory("ur_gello_state_publisher"),
            "config",
            "gello_config.yaml"
        )
        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def estimate_driver_polling_rate(self, trials: int = 50) -> float:
        durations = []
        for _ in range(trials):
            start = time.perf_counter()
            _ = self.robot.get_joint_state()
            end = time.perf_counter()
            durations.append(end - start)
        avg = sum(durations) / trials
        rate = 1.0 / avg
        self.get_logger().info(f"Estimated polling: {avg:.6f} s --> Max rate: {rate:.1f} Hz")
        return rate

    def resolve_publish_rate(self, user: float, config: float, estimate: float) -> float:
        if user > 0:
            preferred = user
        elif config > 0:
            preferred = config
        else:
            preferred = estimate
        final = min(preferred, estimate)
        self.get_logger().info(
            f"Resolved publish rate: {final:.1f} Hz (User: {user}, Config: {config}, Max HW: {estimate:.1f})"
        )
        return final

    def publish_joint_jog(self):
        joints = self.robot.get_joint_state()
        self.get_logger().debug(f"Current joint state: {joints}")
        robot_joints = joints[:self.num_robot_joints]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        msg.position = list(robot_joints)
        self.robot_joint_publisher.publish(msg)

        gripper_msg = Float32()
        gripper_msg.data = float(joints[-1]) if self.gripper else 0.0
        self.gripper_joint_publisher.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloPublisher()
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
        # Only shutdown if not already shutdown
        if rclpy.ok():
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
