import importlib.util
import pathlib
import subprocess
import sys
import unittest

import numpy as np


SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "invert_homogeneous_transform.py"
)
SPEC = importlib.util.spec_from_file_location("invert_homogeneous_transform", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class InvertHomogeneousTransformTest(unittest.TestCase):
    def test_zero_transform_inverse_is_identity(self):
        transform = MODULE.compose_transform([0.0, 0.0, 0.0], "extrinsic_xyz", [0.0, 0.0, 0.0])
        inverse = MODULE.invert_transform(transform)
        self.assertTrue(np.allclose(transform, np.eye(4)))
        self.assertTrue(np.allclose(inverse, np.eye(4)))

    def test_deg_and_rad_build_same_transform(self):
        translation = np.array([1.0, -2.0, 0.5])
        angles_deg = np.array([10.0, 20.0, 30.0])
        angles_rad = np.deg2rad(angles_deg)

        transform_deg = MODULE.compose_transform(
            translation,
            "extrinsic_zyx",
            MODULE._angles_to_rad(angles_deg, "deg"),
        )
        transform_rad = MODULE.compose_transform(translation, "extrinsic_zyx", angles_rad)

        self.assertTrue(np.allclose(transform_deg, transform_rad))

    def test_intrinsic_xyz_matches_extrinsic_zyx(self):
        angles = np.array([0.1, -0.2, 0.3])
        intrinsic_matrix = MODULE.euler_to_matrix(angles, "intrinsic_xyz")
        extrinsic_matrix = MODULE.euler_to_matrix(angles[::-1], "extrinsic_zyx")
        self.assertTrue(np.allclose(intrinsic_matrix, extrinsic_matrix))

    def test_double_inverse_returns_original_transform(self):
        transform = MODULE.compose_transform(
            [1.0, 2.0, 3.0],
            "extrinsic_zxz",
            [0.2, 0.3, -0.4],
        )
        inverse = MODULE.invert_transform(transform)
        recovered = MODULE.invert_transform(inverse)
        self.assertTrue(np.allclose(recovered, transform))
        self.assertTrue(np.allclose(transform @ inverse, np.eye(4), atol=1e-9))

    def test_inverse_rotation_round_trip_matches_transpose(self):
        angles = np.array([0.2, -0.3, 0.4])
        transform = MODULE.compose_transform([0.1, 0.2, 0.3], "extrinsic_xyz", angles)
        inverse = MODULE.invert_transform(transform)
        inverse_angles = MODULE.matrix_to_euler(inverse[:3, :3], "extrinsic_xyz")
        rebuilt_inverse_rotation = MODULE.euler_to_matrix(inverse_angles, "extrinsic_xyz")
        self.assertTrue(np.allclose(rebuilt_inverse_rotation, inverse[:3, :3]))

    def test_singular_rotation_raises_error(self):
        singular_matrix = MODULE.euler_to_matrix([0.0, np.pi / 2.0, 0.0], "extrinsic_xyz")
        with self.assertRaises(ValueError):
            MODULE.matrix_to_euler(singular_matrix, "extrinsic_xyz")

    def test_cli_prints_expected_fields(self):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--translation",
                "1",
                "2",
                "3",
                "--rotation",
                "10",
                "20",
                "30",
                "--order",
                "extrinsic_zyx",
                "--angle-unit",
                "deg",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("input_angle_unit: deg", result.stdout)
        self.assertIn("transform_matrix:", result.stdout)
        self.assertIn("inverse_transform_matrix:", result.stdout)
        self.assertIn("identity_check:", result.stdout)

    def test_cli_rejects_invalid_order(self):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--translation",
                "1",
                "2",
                "3",
                "--rotation",
                "0.1",
                "0.2",
                "0.3",
                "--order",
                "bad_order",
            ],
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("invalid choice", result.stderr)


if __name__ == "__main__":
    unittest.main()
