#!/usr/bin/env python3
"""Build the UMI attachment URDF and its mesh properties offline.

Only numpy/PyYAML are needed. The pinned OSCBF base asset is never edited.
STL volume integrals assume a homogeneous print, scaled to measured mass.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/teleop_controllers/config"


def mesh_properties(path, scale):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError("expected complete binary STL")
    triangles = np.array(
        [struct.unpack_from("<12fH", data, 84 + 50 * i)[3:12] for i in range(count)]
    ).reshape(-1, 3, 3) * scale
    if not np.isfinite(triangles).all():
        raise ValueError("nonfinite mesh")
    # Closed consistently oriented surface: each directed edge has a reverse.
    edges = {}
    for triangle in triangles:
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            edge = (tuple(a), tuple(b))
            edges[edge] = edges.get(edge, 0) + 1
    if any(n != 1 or edges.get((b, a)) != 1 for (a, b), n in edges.items()):
        raise ValueError("mass integration requires a closed oriented mesh")
    a, b, c = triangles.transpose(1, 0, 2)
    volumes = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    volume = volumes.sum()
    if volume <= 0:
        raise ValueError("nonpositive signed volume")
    sums = triangles.sum(axis=1)
    com = (volumes[:, None] * sums).sum(axis=0) / (4 * volume)
    second = np.einsum("i,ij,ik->jk", volumes, sums, sums)
    second += np.einsum("i,iaj,iak->jk", volumes, triangles, triangles)
    second /= 20
    inertia = np.trace(second) * np.eye(3) - second
    inertia -= volume * (com @ com * np.eye(3) - np.outer(com, com))
    return triangles.reshape(-1, 3), volume, com, inertia / volume


def vector_text(value):
    return " ".join(format(float(v), ".17g") for v in value)


def properties():
    settings = yaml.safe_load((CONFIG / "umi_fingers.yaml").read_text())
    mesh = CONFIG / "meshes/umi_v1.stl"
    if hashlib.sha256(mesh.read_bytes()).hexdigest() != settings["mesh_sha256"]:
        raise ValueError("attachment mesh fingerprint differs")
    vertices, volume, com, unit_inertia = mesh_properties(mesh, settings["mesh_scale_m"])
    mass = settings["mass_per_finger_kg"]
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError("measured attachment mass required")
    # STL X is the screw axis; STL -Z points toward the fingertip.
    rotation = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    mount = np.asarray(settings["stock_finger_mount_xyz_m"])
    anchor = np.asarray(settings["mesh_mount_xyz_m"])
    translation = mount - rotation @ anchor
    reference = settings["reference_opening_per_finger_m"]
    travel = settings["opening_per_finger_range_m"]
    padding = settings["mount_uncertainty_m"]
    if not 0 <= travel[0] <= reference <= travel[1] <= .04 or padding < 0:
        raise ValueError("invalid opening range/reference or mounting uncertainty")
    swept = []
    poses = []
    for sign in [1, -1]:
        mirror = np.diag([sign, sign, 1.])
        r, t = mirror @ rotation, mirror @ translation
        poses.append((r, t))
        for opening in travel:
            points = vertices @ r.T + t + [0, sign * opening, .0584]
            swept.append(points)
    swept = np.concatenate(swept)
    lower, upper = swept.min(axis=0) - padding, swept.max(axis=0) + padding
    # Partition the enclosing box; each cell is wholly inside its circumsphere.
    # 6 lateral x 2 axial cells fit current table/point/sphere capacities.
    counts = np.array([1, 6, 2])
    half = (upper - lower) / (2 * counts)
    radius = float(np.linalg.norm(half))
    centres = [lower + (np.array(index) * 2 + 1) * half
               for index in itertools.product(*(range(n) for n in counts))]
    yaw = -np.pi / 4
    hand_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                              [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.]])
    spheres = [dict(frame=7, offset=(hand_rotation @ p).tolist(), radius_m=radius, group=10)
               for p in centres]
    # The whole triangle and continuous travel interval lie in the box, not only
    # sampled vertices. This check additionally catches transform/export mistakes.
    nearest = np.full(len(swept), np.inf)
    for centre in centres:
        nearest = np.minimum(nearest, np.linalg.norm(swept - centre, axis=1))
    if nearest.max() > radius + 1e-12:
        raise ValueError("swept mesh is outside its generated cover")
    return settings, vertices, volume, com, unit_inertia, poses, lower, upper, spheres


def generated():
    s, vertices, volume, com, inertia, poses, lower, upper, spheres = properties()
    root = ET.parse(CONFIG / "panda_oscbf.urdf").getroot()
    for side, (rotation, translation) in zip(["left", "right"], poses):
        name = f"panda_umi_{side}finger"
        link = ET.SubElement(root, "link", name=name)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=vector_text(com), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(s["mass_per_finger_kg"]))
        tensor = s["mass_per_finger_kg"] * inertia
        ET.SubElement(inertial, "inertia", **{key: format(float(tensor[i, j]), ".17g")
            for key, i, j in [("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2),
                              ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2)]})
        visual = ET.SubElement(link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename="package://teleop_controllers/config/meshes/umi_v1.stl",
                      scale="0.001 0.001 0.001")
        # A tight local box for generic URDF consumers. Runtime CBF uses the
        # separate opening-swept cover below, never this visual/collision tag.
        collision = ET.SubElement(link, "collision")
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        ET.SubElement(collision, "origin", xyz=vector_text((lo + hi) / 2), rpy="0 0 0")
        ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=vector_text(hi - lo))
        joint = ET.SubElement(root, "joint", name=name + "_joint", type="fixed")
        ET.SubElement(joint, "parent", link=f"panda_{side}finger")
        ET.SubElement(joint, "child", link=name)
        # Both transforms have roll=pi; yaw is +/-pi/2.
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        ET.SubElement(joint, "origin", xyz=vector_text(translation), rpy=vector_text([np.pi, 0, yaw]))
        finger_joint = root.find(f"joint[@name='panda_finger_joint{1 if side == 'left' else 2}']/origin")
        sign = 1 if side == "left" else -1
        finger_joint.set("xyz", vector_text([0, sign * s["reference_opening_per_finger_m"], .0584]))
    ET.indent(root, space="  ")
    urdf = ('<?xml version="1.0"?>\n<!-- Generated by scripts/generate_umi_finger_model.py. '
            'OSCBF base retained; UMI print mass is measured, its density model is approximate. -->\n'
            + ET.tostring(root, encoding="unicode") + "\n")
    report = dict(mesh_sha256=s["mesh_sha256"], mass_per_finger_kg=s["mass_per_finger_kg"],
                  volume_m3=volume, density_assumption="homogeneous watertight STL scaled to measured total attachment mass",
                  implied_density_kg_m3=s["mass_per_finger_kg"] / volume,
                  mesh_com_m=com.tolist(), mesh_inertia_per_kg_m2=inertia.tolist(),
                  reference_opening_per_finger_m=s["reference_opening_per_finger_m"],
                  hand_swept_box_lower_m=lower.tolist(), hand_swept_box_upper_m=upper.tolist(),
                  cover_spheres=len(spheres), cover_radius_m=spheres[0]["radius_m"],
                  mount_uncertainty_m=s["mount_uncertainty_m"],
                  mesh_tip_direction_in_stock_frame=[0, 0, 1],
                  mesh_to_stock_finger=[dict(rotation=r.tolist(), translation_m=t.tolist()) for r, t in poses])
    return urdf, json.dumps(report, indent=2) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    targets = [CONFIG / "panda_umi.urdf", CONFIG / "umi_fingers.generated.json"]
    for target, text in zip(targets, generated()):
        if args.check:
            if not target.exists() or target.read_text() != text:
                raise SystemExit(f"generated attachment differs: {target}")
        else:
            target.write_text(text)
    print("UMI URDF and measured-mass inertia verified")


if __name__ == "__main__":
    main()
