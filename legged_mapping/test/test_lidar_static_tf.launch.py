import importlib.util
import pathlib
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_msgs.msg import TFMessage


SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "invert_homogeneous_transform.py"
)
SPEC = importlib.util.spec_from_file_location("invert_homogeneous_transform", SCRIPT_PATH)
TRANSFORM_SCRIPT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRANSFORM_SCRIPT)


def quaternion_from_matrix(matrix):
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.array([qx, qy, qz, qw])
    return quaternion / np.linalg.norm(quaternion)


def expected_quaternion(order, angle_unit, angles):
    angles_array = np.asarray(angles, dtype=float)
    angles_rad = TRANSFORM_SCRIPT._angles_to_rad(angles_array, angle_unit)
    rotation = TRANSFORM_SCRIPT.euler_to_matrix(angles_rad, order)
    return quaternion_from_matrix(rotation)


def quaternion_equivalent(lhs, rhs, atol=1e-9):
    return np.allclose(lhs, rhs, atol=atol) or np.allclose(lhs, -rhs, atol=atol)


def generate_test_description():
    params_file = (
        get_package_share_directory("legged_mapping") + "/config/lidar_static_tf.yaml"
    )

    static_tf_node = launch_ros.actions.Node(
        package="legged_mapping",
        executable="lidar_static_tf_node",
        name="lidar_static_tf_node",
        parameters=[params_file],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            static_tf_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLidarStaticTfNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _receive_transforms(self):
        node = rclpy.create_node("lidar_static_tf_test")
        received_messages = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        subscription = node.create_subscription(
            TFMessage,
            "/tf_static",
            received_messages.append,
            qos,
        )

        deadline = time.time() + 5.0
        while time.time() < deadline and not received_messages:
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_subscription(subscription)
        node.destroy_node()
        self.assertTrue(received_messages, "Did not receive a message on /tf_static")
        return received_messages[-1].transforms

    def test_publishes_expected_static_transforms(self):
        transforms = self._receive_transforms()
        frame_pairs = {
            (transform.header.frame_id, transform.child_frame_id)
            for transform in transforms
        }
        self.assertEqual(
            frame_pairs,
            {
                ("odom", "camera_init"),
                ("body", "base"),
            },
        )

    def test_default_config_quaternions_match_rotation_definition(self):
        transforms = self._receive_transforms()
        transform_map = {
            (transform.header.frame_id, transform.child_frame_id): transform
            for transform in transforms
        }

        odom_quaternion = expected_quaternion(
            "extrinsic_xyz",
            "deg",
            [-2.059313, 12.900457, -0.424709],
        )
        body_quaternion = expected_quaternion(
            "extrinsic_xyz",
            "deg",
            [2.0153836829205622, -12.907278063151713, -0.036128310417152414],
        )

        actual_odom_quaternion = np.array(
            [
                transform_map[("odom", "camera_init")].transform.rotation.x,
                transform_map[("odom", "camera_init")].transform.rotation.y,
                transform_map[("odom", "camera_init")].transform.rotation.z,
                transform_map[("odom", "camera_init")].transform.rotation.w,
            ]
        )
        actual_body_quaternion = np.array(
            [
                transform_map[("body", "base")].transform.rotation.x,
                transform_map[("body", "base")].transform.rotation.y,
                transform_map[("body", "base")].transform.rotation.z,
                transform_map[("body", "base")].transform.rotation.w,
            ]
        )

        self.assertTrue(quaternion_equivalent(actual_odom_quaternion, odom_quaternion))
        self.assertTrue(quaternion_equivalent(actual_body_quaternion, body_quaternion))
