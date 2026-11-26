#!/usr/bin/env python3
"""Regenerate the installed Hand and finger collision volumes from their meshes."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from generate_cbf_collision_geometry import mesh_vertices
from generate_umi_finger_model import mesh_properties

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/cbf_safety_filter/config"
MODEL = ROOT / "src/teleop_controllers/config"


def origin(joint):
    pose = joint.find("origin")
    if pose is None:
        return Rotation.identity(), np.zeros(3)
    return (
        Rotation.from_euler("xyz", np.fromstring(pose.get("rpy", "0 0 0"), sep=" ")),
        np.fromstring(pose.get("xyz", "0 0 0"), sep=" "),
    )


def finger_capsules():
    settings = yaml.safe_load((MODEL / "umi_fingers.yaml").read_text())
    points = mesh_properties(MODEL / "meshes/umi_v1.stl", settings["mesh_scale_m"])[0]
    # Fit parallel to the long axis. The transverse bounding-box centre avoids
    # letting a dense patch of STL triangles bias the fit.
    lower, upper = points.min(axis=0), points.max(axis=0)
    center = (lower[:2] + upper[:2]) / 2
    lateral = np.linalg.norm(points[:, :2] - center, axis=1)
    mesh_radius = lateral.max()
    radius = mesh_radius + settings["mount_uncertainty_m"]
    reach = np.sqrt(np.maximum(0, mesh_radius ** 2 - lateral ** 2))
    start = min(np.min(points[:, 2] + reach), upper[2])
    end = max(np.max(points[:, 2] - reach), start)
    a, b = np.r_[center, start], np.r_[center, end]
    offset_z = points[:, 2] - np.clip(points[:, 2], start, end)
    if np.hypot(lateral, offset_z).max() > mesh_radius + 1e-10:
        raise ValueError("UMI capsule does not enclose the STL vertices")
    urdf = ET.parse(MODEL / "panda_umi.urdf")
    hand_rotation, hand_offset = origin(urdf.find("joint[@name='panda_hand_joint']"))
    result = []
    for side, index in (("left", 1), ("right", 2)):
        finger = urdf.find(f"joint[@name='panda_finger_joint{index}']")
        finger_rotation, finger_offset = origin(finger)
        attachment, attachment_offset = origin(urdf.find(f"joint[@name='panda_umi_{side}finger_joint']"))
        axis = np.fromstring(finger.find("axis").get("xyz"), sep=" ")
        finger_offset -= axis * settings["reference_opening_per_finger_m"]

        def transform(point):
            return hand_rotation.apply(
                finger_rotation.apply(attachment.apply(point) + attachment_offset) + finger_offset
            ) + hand_offset

        tip = transform(a)
        # Flange +Z points toward world -Z in the home posture.
        tip[2] -= .012
        result.append(dict(frame=7, group=10, offset=tip.tolist(), end=transform(b).tolist(),
                           radius_m=float(radius - .002), opening_axis=hand_rotation.apply(axis).tolist()))
    return result


def hand_capsules():
    urdf = ET.parse(MODEL / "panda_umi.urdf")
    hand_rotation, hand_offset = origin(urdf.find("joint[@name='panda_hand_joint']"))

    # Fit a single transverse capsule directly around the housing mesh.
    # A capsule is convex, so enclosing its vertices includes the triangles.
    housing = np.asarray(mesh_vertices("hand"))
    lower, upper = housing.min(axis=0), housing.max(axis=0)
    a = (lower + upper) / 2
    b = a.copy()
    transverse = np.linalg.norm(housing[:, [0, 2]] - a[[0, 2]], axis=1)
    radius = transverse.max()
    reach = np.sqrt(np.maximum(0., radius * radius - transverse * transverse))
    a[1] = min(np.min(housing[:, 1] + reach), upper[1])
    b[1] = max(np.max(housing[:, 1] - reach), a[1])
    # Flange +Z points toward world -Z in the home posture.
    housing_offset = hand_offset + np.array([0., 0., .005])
    result = [dict(frame=7, group=9, offset=(hand_rotation.apply(a) + housing_offset).tolist(),
                   end=(hand_rotation.apply(b) + housing_offset).tolist(), radius_m=float(radius - .005))]

    # Fit each stock finger at zero opening. The runtime shifts its sphere
    # with the same measured opening used by the UMI capsule.
    settings = yaml.safe_load((MODEL / "umi_fingers.yaml").read_text())
    mesh = np.asarray(mesh_vertices("finger"))
    for side, index in (("left", 1), ("right", 2)):
        joint = urdf.find(f"joint[@name='panda_finger_joint{index}']")
        visual = urdf.find(f"link[@name='panda_{side}finger']/visual")
        finger_rotation, finger_offset = origin(joint)
        visual_rotation, visual_offset = origin(visual)
        axis = finger_rotation.apply(np.fromstring(joint.find("axis").get("xyz"), sep=" "))
        finger_offset -= axis * settings["reference_opening_per_finger_m"]
        points = finger_rotation.apply(visual_rotation.apply(mesh) + visual_offset) + finger_offset
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        radius = np.linalg.norm(points - center, axis=1).max()
        opening_axis = hand_rotation.apply(axis)
        center = hand_rotation.apply(center) + hand_offset
        # Separate the nominal sphere from the UMI tip capsule in the preview.
        center += .005 * opening_axis
        center[2] -= .010
        result.append(dict(frame=7, group=9, offset=center.tolist(),
                           radius_m=float(radius - .015), opening_axis=opening_axis.tolist()))
    return result


def main():
    single = CONFIG / "single_panda/geometry_panda_umi.yaml"
    arm = [volume for volume in yaml.safe_load(single.read_text())["body_volumes"]
           if volume["group"] < 9]
    cover = arm + hand_capsules() + finger_capsules()
    for path in (single, CONFIG / "dual_panda/geometry.left.yaml",
                 CONFIG / "dual_panda/geometry.right.yaml"):
        lines = path.read_text().splitlines(keepends=True)
        document = yaml.safe_load("".join(lines))
        document["body_volumes"] = cover
        path.write_text(lines[0] + yaml.safe_dump(document, sort_keys=False))
        print(f"{path.name}: {len(cover)} collision volumes")


if __name__ == "__main__":
    main()
