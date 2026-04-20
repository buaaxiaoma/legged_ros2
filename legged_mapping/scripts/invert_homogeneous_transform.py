#!/usr/bin/env python3

import argparse
import math
from typing import Dict
from typing import Iterable
from typing import Tuple

import numpy as np


_NEXT_AXIS = [1, 2, 0, 1]
_EPS = 1e-9
_AXES_TO_TUPLE: Dict[str, Tuple[int, int, int, int]] = {
    "sxyz": (0, 0, 0, 0),
    "sxyx": (0, 0, 1, 0),
    "sxzy": (0, 1, 0, 0),
    "sxzx": (0, 1, 1, 0),
    "syzx": (1, 0, 0, 0),
    "syzy": (1, 0, 1, 0),
    "syxz": (1, 1, 0, 0),
    "syxy": (1, 1, 1, 0),
    "szxy": (2, 0, 0, 0),
    "szxz": (2, 0, 1, 0),
    "szyx": (2, 1, 0, 0),
    "szyz": (2, 1, 1, 0),
    "rzyx": (0, 0, 0, 1),
    "rxyx": (0, 0, 1, 1),
    "ryzx": (0, 1, 0, 1),
    "rxzx": (0, 1, 1, 1),
    "rxzy": (1, 0, 0, 1),
    "ryzy": (1, 0, 1, 1),
    "rzxy": (1, 1, 0, 1),
    "ryxy": (1, 1, 1, 1),
    "ryxz": (2, 0, 0, 1),
    "rzxz": (2, 0, 1, 1),
    "rxyz": (2, 1, 0, 1),
    "rzyz": (2, 1, 1, 1),
}
_ORDER_TO_AXES = {
    f"{'intrinsic' if axes[0] == 'r' else 'extrinsic'}_{axes[1:]}": axes
    for axes in _AXES_TO_TUPLE
}
_SUPPORTED_ORDERS = tuple(sorted(_ORDER_TO_AXES))


def _format_supported_orders() -> str:
    return ", ".join(_SUPPORTED_ORDERS)


def _rotation_tuple(order: str) -> Tuple[int, int, int, int]:
    try:
        return _AXES_TO_TUPLE[_ORDER_TO_AXES[order]]
    except KeyError as error:
        raise ValueError(
            f"Unsupported order '{order}'. Supported values: {_format_supported_orders()}"
        ) from error


def _validate_vector(name: str, values: Iterable[float]) -> np.ndarray:
    vector = np.asarray(list(values), dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must contain exactly 3 values.")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite values.")
    return vector


def _angles_to_rad(angles: np.ndarray, angle_unit: str) -> np.ndarray:
    if angle_unit == "rad":
        return angles
    if angle_unit == "deg":
        return np.deg2rad(angles)
    raise ValueError(f"Unsupported angle unit '{angle_unit}'. Use 'rad' or 'deg'.")


def _angles_from_rad(angles_rad: np.ndarray, angle_unit: str) -> np.ndarray:
    if angle_unit == "rad":
        return angles_rad
    if angle_unit == "deg":
        return np.rad2deg(angles_rad)
    raise ValueError(f"Unsupported angle unit '{angle_unit}'. Use 'rad' or 'deg'.")


def euler_to_matrix(angles_rad: Iterable[float], order: str) -> np.ndarray:
    ai, aj, ak = _validate_vector("angles_rad", angles_rad)
    firstaxis, parity, repetition, frame = _rotation_tuple(order)

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    matrix = np.eye(3, dtype=float)
    if repetition:
        matrix[i, i] = cj
        matrix[i, j] = sj * si
        matrix[i, k] = sj * ci
        matrix[j, i] = sj * sk
        matrix[j, j] = -cj * ss + cc
        matrix[j, k] = -cj * cs - sc
        matrix[k, i] = -sj * ck
        matrix[k, j] = cj * sc + cs
        matrix[k, k] = cj * cc - ss
    else:
        matrix[i, i] = cj * ck
        matrix[i, j] = sj * sc - cs
        matrix[i, k] = sj * cc + ss
        matrix[j, i] = cj * sk
        matrix[j, j] = sj * ss + cc
        matrix[j, k] = sj * cs - sc
        matrix[k, i] = -sj
        matrix[k, j] = cj * si
        matrix[k, k] = cj * ci

    return matrix


def matrix_to_euler(matrix: np.ndarray, order: str) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("matrix must be a 3x3 rotation matrix.")

    firstaxis, parity, repetition, frame = _rotation_tuple(order)
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if repetition:
        sy = math.sqrt(matrix[i, j] * matrix[i, j] + matrix[i, k] * matrix[i, k])
        if sy <= _EPS:
            raise ValueError(
                f"Rotation matrix is singular for order '{order}' and cannot be "
                "uniquely converted back to Euler angles."
            )
        ax = math.atan2(matrix[i, j], matrix[i, k])
        ay = math.atan2(sy, matrix[i, i])
        az = math.atan2(matrix[j, i], -matrix[k, i])
    else:
        cy = math.sqrt(matrix[i, i] * matrix[i, i] + matrix[j, i] * matrix[j, i])
        if cy <= _EPS:
            raise ValueError(
                f"Rotation matrix is singular for order '{order}' and cannot be "
                "uniquely converted back to Euler angles."
            )
        ax = math.atan2(matrix[k, j], matrix[k, k])
        ay = math.atan2(-matrix[k, i], cy)
        az = math.atan2(matrix[j, i], matrix[i, i])

    angles = np.array([ax, ay, az], dtype=float)
    if parity:
        angles = -angles
    if frame:
        angles = angles[[2, 1, 0]]
    return angles


def compose_transform(
    translation: Iterable[float],
    order: str,
    angles_rad: Iterable[float],
) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = euler_to_matrix(angles_rad, order)
    transform[:3, 3] = _validate_vector("translation", translation)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix.")

    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def format_matrix(matrix: np.ndarray) -> str:
    return np.array2string(matrix, precision=6, suppress_small=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute a homogeneous transform and its inverse from translation, "
            "Euler angles, and an explicit intrinsic/extrinsic rotation order."
        ),
        epilog=(
            "The three values passed to --rotation are interpreted according to "
            "the selected --order. Supported orders: " + _format_supported_orders()
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--translation",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        required=True,
        help="Translation vector.",
    )
    parser.add_argument(
        "--rotation",
        nargs=3,
        type=float,
        metavar=("A1", "A2", "A3"),
        required=True,
        help="Euler angles for the selected order.",
    )
    parser.add_argument(
        "--order",
        required=True,
        choices=_SUPPORTED_ORDERS,
        help="Explicit intrinsic/extrinsic Euler order.",
    )
    parser.add_argument(
        "--angle-unit",
        default="rad",
        choices=("rad", "deg"),
        help="Angle unit for both input and output values.",
    )
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        translation = _validate_vector("translation", args.translation)
        rotation = _validate_vector("rotation", args.rotation)
        rotation_rad = _angles_to_rad(rotation, args.angle_unit)

        transform = compose_transform(translation, args.order, rotation_rad)
        inverse_transform_matrix = invert_transform(transform)
        inverse_translation = inverse_transform_matrix[:3, 3]
        inverse_rotation_rad = matrix_to_euler(inverse_transform_matrix[:3, :3], args.order)
        inverse_rotation = _angles_from_rad(inverse_rotation_rad, args.angle_unit)
        identity_check = transform @ inverse_transform_matrix

        print(f"input_translation: {translation.tolist()}")
        print(f"input_rotation: {rotation.tolist()}")
        print(f"input_order: {args.order}")
        print(f"input_angle_unit: {args.angle_unit}")
        print("transform_matrix:")
        print(format_matrix(transform))
        print("inverse_transform_matrix:")
        print(format_matrix(inverse_transform_matrix))
        print(f"inverse_translation: {inverse_translation.tolist()}")
        print(f"inverse_rotation: {inverse_rotation.tolist()}")
        print("identity_check:")
        print(format_matrix(identity_check))
        return 0
    except ValueError as error:
        parser.exit(status=1, message=f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
