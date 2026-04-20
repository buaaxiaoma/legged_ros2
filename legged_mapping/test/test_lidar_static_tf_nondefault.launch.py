import importlib.util
import pathlib
import time
import unittest

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

PARAMS_FILE = (
    pathlib.Path(__file__).resolve().parent / "lidar_static_tf_nondefault.yaml"
)


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
    static_tf_node = launch_ros.actions.Node(
        package="legged_mapping",
        executable="lidar_static_tf_node",
        name="lidar_static_tf_node",
        parameters=[str(PARAMS_FILE)],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            static_tf_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLidarStaticTfNodeNondefault(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_nondefault_order_and_deg_unit(self):
        node = rclpy.create_node("lidar_static_tf_nondefault_test")
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

        transforms = {
            (transform.header.frame_id, transform.child_frame_id): transform
            for transform in received_messages[-1].transforms
        }

        expected_pairs = {
            ("odom", "tracking_origin"),
            ("tracking_body", "base"),
        }
        self.assertEqual(set(transforms), expected_pairs)

        expected_origin_quaternion = expected_quaternion(
            "intrinsic_zyx",
            "deg",
            [10.0, 20.0, 30.0],
        )
        expected_base_quaternion = expected_quaternion(
            "intrinsic_zyx",
            "deg",
            [-15.0, 5.0, 40.0],
        )

        actual_origin_quaternion = np.array(
            [
                transforms[("odom", "tracking_origin")].transform.rotation.x,
                transforms[("odom", "tracking_origin")].transform.rotation.y,
                transforms[("odom", "tracking_origin")].transform.rotation.z,
                transforms[("odom", "tracking_origin")].transform.rotation.w,
            ]
        )
        actual_base_quaternion = np.array(
            [
                transforms[("tracking_body", "base")].transform.rotation.x,
                transforms[("tracking_body", "base")].transform.rotation.y,
                transforms[("tracking_body", "base")].transform.rotation.z,
                transforms[("tracking_body", "base")].transform.rotation.w,
            ]
        )

        self.assertTrue(
            quaternion_equivalent(actual_origin_quaternion, expected_origin_quaternion)
        )
        self.assertTrue(
            quaternion_equivalent(actual_base_quaternion, expected_base_quaternion)
        )
