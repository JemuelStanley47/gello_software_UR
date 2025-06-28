import os
import sys
import glob
from typing import Tuple
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from rclpy.qos import QoSProfile

class GelloPublisher(Node):
    def __init__(self):
        super().__init__("gello_publisher")

        default_com_port = self.determine_default_com_port()
        self.declare_parameter("com_port", default_com_port)
        self.com_port = self.get_parameter("com_port").get_parameter_value().string_value
        self.port = self.com_port.split("/")[-1]

        config_path = os.path.join(
            get_package_share_directory("ur_gello_state_publisher"),
            "config",
            "gello_config.yaml",
        )
        self.get_values_from_config(config_path)

        # Load user-defined or fallback publish rate
        self.declare_parameter("publish_rate", -1.0)
        user_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        config_rate = self.config.get("publish_rate", -1.0)
        estimated_rate = self.estimate_driver_polling_rate()
        # estimated_safe = min(estimated_rate * 0.8, 100.0)

        self.publish_rate = self.resolve_publish_rate(user_rate, config_rate, estimated_rate)
        self.get_logger().info(f"Using publish rate: {self.publish_rate:.1f} Hz")

        # Publishers
        self.qos = QoSProfile(depth=10)
        self.robot_joint_publisher = self.create_publisher(JointState, "/gello/joint_states", self.qos)
        self.gripper_joint_publisher = self.create_publisher(
            Float32, "/gripper_client/target_gripper_width_percent", self.qos
        )

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_jog)

    def resolve_publish_rate(self, user_rate: float, config_rate: float, estimate: float) -> float:
        """Resolve final publish rate, preferring: user param > config file > estimated.
        Ensures the final rate does not exceed the estimated max supported by hardware.
        """
        preferred = estimate  # Fallback
        if user_rate > 0:
            preferred = user_rate
        elif config_rate > 0:
            preferred = config_rate

        # Cap to what hardware supports
        final_rate = min(preferred, estimate)
        self.get_logger().info(
            f"Resolved publish rate: {final_rate:.1f} Hz "
            f"(User: {user_rate}, Config: {config_rate}, Max HW: {estimate:.1f})"
        )
        return final_rate

        
    def determine_default_com_port(self) -> str:
        matches = glob.glob("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter*")
        if matches:
            self.get_logger().info(f"Auto-detected com_ports: {matches}")
            return matches[0]
        else:
            self.get_logger().warn("No com_ports detected. Please specify the com_port manually.")
            return "INVALID_COM_PORT"

    def get_values_from_config(self, config_file: str):
        with open(config_file, "r") as file:
            self.config = yaml.safe_load(file)

        self.num_robot_joints: int = self.config[self.port]["num_joints"]
        """The number of joints in the robot."""

        self.joint_signs: Tuple[float, ...] = self.config[self.port]["joint_signs"]
        """Depending on how the motor is mounted on the Gello, its rotation direction can be reversed."""

        self.gripper: bool = self.config[self.port]["gripper"]
        """Whether or not the gripper is attached."""

        joint_ids = list(range(1, self.num_joints + 1))
        self.add_dynamixel_driver_path()
        from gello.dynamixel.driver import DynamixelDriver

        self.driver = DynamixelDriver(joint_ids, port=self.com_port, baudrate=57600)
        """The driver for the Dynamixel motors."""

        self.best_offsets = np.array(self.config[self.port]["best_offsets"])
        """The best offsets for the joints."""

        self.gripper_range_rad: Tuple[float, float] = self.config[self.port]["gripper_range_rad"]
        """The range of the gripper in radians."""
        self.joint_names: Tuple[str, ...] = tuple(self.config[self.port]["joint_names"])
        """The names of the joints in the robot."""
        self.__post_init__()

    def __post_init__(self):
        assert len(self.joint_signs) == self.num_robot_joints
        for idx, j in enumerate(self.joint_signs):
            assert j == -1 or j == 1, f"Joint idx: {idx} should be -1 or 1, but got {j}."

    @property
    def num_joints(self) -> int:
        return self.num_robot_joints + (1 if self.gripper else 0)

    def publish_joint_jog(self):
        current_joints = self.driver.get_joints()
        self.get_logger().debug(f"Raw joint positions: {current_joints.shape}")
        current_robot_joints = current_joints[: self.num_robot_joints]
        current_joints_corrected = (current_robot_joints - self.best_offsets) * self.joint_signs

        robot_joint_states = JointState()
        robot_joint_states.header.stamp = self.get_clock().now().to_msg()
        robot_joint_states.name = self.joint_names
        robot_joint_states.header.frame_id = "base_link"
        robot_joint_states.position = [float(joint) for joint in current_joints_corrected]

        gripper_joint_states = Float32()
        if self.gripper:
            gripper_position = current_joints[-1]
            self.get_logger().debug(f"Raw gripper position: {gripper_position}")
            gripper_joint_states.data = self.gripper_readout_to_percent(gripper_position)
        else:
            gripper_joint_states.data = 0.0

        self.robot_joint_publisher.publish(robot_joint_states)
        self.gripper_joint_publisher.publish(gripper_joint_states)

    def gripper_readout_to_percent(self, gripper_position: float) -> float:
        gripper_percent = (gripper_position - self.gripper_range_rad[0]) / (
            self.gripper_range_rad[1] - self.gripper_range_rad[0]
        )
        return max(0.0, min(1.0, gripper_percent))

    def add_dynamixel_driver_path(self):
        gello_path = os.path.abspath(
            os.path.join(get_package_prefix("ur_gello_state_publisher"), "../../../")
        )
        sys.path.insert(0, gello_path)

    def estimate_driver_polling_rate(self, trials=50) -> float:
        import time
        times = []
        for _ in range(trials):
            start = time.perf_counter()
            _ = self.driver.get_joints()
            end = time.perf_counter()
            times.append(end - start)

        avg_duration = sum(times) / len(times)
        rate = 1.0 / avg_duration
        self.get_logger().info(
            f"Estimated average polling duration: {avg_duration:.6f} s --> Max rate: {rate:.1f} Hz"
        )
        return rate



def main(args=None):
    rclpy.init(args=args)
    node = GelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
