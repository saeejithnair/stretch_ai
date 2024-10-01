# Copyright (c) Hello Robot, Inc.
# All rights reserved.
#
# This source code is licensed under the license found in the LICENSE file in the root directory
# of this source tree.
#
# Some code may be adapted from other open-source works with their respective licenses. Original
# license information maybe found below, if so.

# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
from typing import Iterable, Tuple

import numpy as np
import sophuspy as sp
from scipy.spatial.transform import Rotation

from stretch.core.interfaces import Pose


def normalize_ang_error(ang):
    return (ang + np.pi) % (2 * np.pi) - np.pi


def point_global_to_base(XYZ, current_pose):
    """
    Transforms the point cloud into geocentric frame to account for
    camera position
    Input:
        XYZ                     : ...x3
        current_pose            : base position (x, y, theta (radians))
    Output:
        XYZ : ...x3
    """
    pose_world2target = xyt2sophus(current_pose)
    return (pose_world2target.inverse() * xyz2sophus(XYZ)).translation()


def xyt_global_to_base(XYT, current_pose):
    """
    Transforms the point cloud into geocentric frame to account for
    camera position
    Input:
        XYZ                     : ...x3
        current_pose            : base position (x, y, theta (radians))
    Output:
        XYZ : ...x3
    """
    pose_world2target = xyt2sophus(XYT)
    pose_world2base = xyt2sophus(current_pose)
    pose_base2target = pose_world2base.inverse() * pose_world2target
    return sophus2xyt(pose_base2target)


def pose_global_to_base(pose, current_pose):
    """
    Transforms the point cloud into geocentric frame to account for
    camera position
    Input:
        XYZ                     : ...x3
        current_pose            : base position (x, y, theta (radians))
    Output:
        XYZ : ...x3
    """
    pose_world2target = pose2sophus(pose)
    pose_world2base = xyt2sophus(current_pose)
    pose_base2target = pose_world2base.inverse() * pose_world2target
    return sophus2pose(pose_base2target)


def pose2sophus(pose: np.ndarray) -> sp.SE3:
    """Convert a 4x4 matrix in se(3) into a Sophus SE3 object.

    Args:
        pose: np.ndarray of shape (4, 4)

    Returns:
        sp.SE3
    """
    return sp.SE3(pose[:3, :3], pose[:3, 3])


def sophus2pose(pose: sp.SE3) -> np.ndarray:
    """Convert a Sophus SE3 object into a 4x4 matrix in se(3).

    Args:
        pose: sp.SE3

    Returns:
        np.ndarray of shape (4, 4)
    """
    pose_mat = pose.matrix()
    return pose_mat


def pose_global_to_base_xyt(pose_world2target: Pose, pose_world2base: Pose) -> np.ndarray:
    """Transform a pose from global to base frame, getting the relative xyt.
    Input:
        pose: Pose object
        current_pose: Pose object
    Output:
        xyz: np.ndarray
    """
    pose_base2target = pose_world2base.inverse() * pose_world2target
    return sophus2xyt(pose_base2target)


def xyt_base_to_global(out_XYT, current_pose):
    """
    Transforms the point cloud from base frame into geocentric frame
    Input:
        XYZ                     : ...x3
        current_pose            : base position (x, y, theta (radians))
    Output:
        XYZ : ...x3
    """
    pose_base2target = xyt2sophus(out_XYT)
    pose_world2base = xyt2sophus(current_pose)
    pose_world2target = pose_world2base * pose_base2target
    return sophus2xyt(pose_world2target)


def xyz2sophus(xyz: np.ndarray) -> sp.SE3:
    """
    Converts XYZ coordinates to an sophus SE3 pose object.
    """
    return sp.SE3(np.eye(3), xyz)


def xyt2sophus(xyt: np.ndarray) -> sp.SE3:
    """
    Converts SE2 coordinates (x, y, rz) to an sophus SE3 pose object.
    """
    x = np.array([xyt[0], xyt[1], 0.0])
    r_mat = sp.SO3.exp([0.0, 0.0, xyt[2]]).matrix()
    return sp.SE3(r_mat, x)


def sophus2xyt(se3: sp.SE3) -> np.ndarray:
    """
    Converts an sophus SE3 pose object to SE2 coordinates (x, y, rz).
    """
    x_vec = se3.translation()
    r_vec = se3.so3().log()
    return np.array([x_vec[0], x_vec[1], r_vec[2]])


def posquat2sophus(pos: Iterable[float], quat: Iterable[float]) -> sp.SE3:
    r_mat = Rotation.from_quat(quat).as_matrix()
    return sp.SE3(r_mat, pos)


def sophus2posquat(se3: sp.SE3) -> Tuple[Iterable[float], Iterable[float]]:
    pos = se3.translation()
    quat = Rotation.from_matrix(se3.so3().matrix()).as_quat()
    return pos, quat


def obs2xyt(pose: Pose):
    pos = pose.position
    quat = pose.orientation
    return sophus2xyt(posquat2sophus(pos, quat))


def xyt2obs(xyt: np.ndarray):
    pose_sp = xyt2sophus(xyt)
    return sophus2obs(pose_sp)


def sophus2obs(pose_sp):
    return Pose(
        position=pose_sp.translation(),
        orientation=Rotation.from_matrix(pose_sp.so3().matrix()).as_quat(),
    )
