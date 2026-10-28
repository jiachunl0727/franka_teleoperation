#!/usr/bin/env python3
"""Render the actual CBF capsules/AABBs using the pinned URDF and saved home pose.

Robot-free documentation tool. Requires numpy, matplotlib and PyYAML; no ROS,
network or robot connection. Reads capsule definitions from C++, checks them
against the URDF collision primitives at safety_distance=0, and exports exact
world endpoint/bounds coordinates as CSV/JSON as well as figures.
"""

import argparse
import ast
import csv
import itertools
import json
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLCONFIGDIR", "/tmp/teleop-geometry-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "src/third_party/franka_arm_ros2/franka_description"
COLLADA = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
COLORS = plt.get_cmap("tab20")(np.arange(14))


def scalar(text):
    """Evaluate only arithmetic constants used in pinned xacro attributes."""
    text = text.removeprefix("${").removesuffix("}")

    def visit(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return node.value
        if isinstance(node, ast.Name):
            return {"pi": math.pi, "safety_distance": 0.0}[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -visit(node.operand)
        if isinstance(node, ast.BinOp):
            a, b = visit(node.left), visit(node.right)
            if isinstance(node.op, ast.Add):
                return a + b
            if isinstance(node.op, ast.Sub):
                return a - b
            if isinstance(node.op, ast.Mult):
                return a * b
            if isinstance(node.op, ast.Div):
                return a / b
        raise ValueError(f"Unsupported xacro expression: {text}")

    return float(visit(ast.parse(text, mode="eval").body))


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (
        np.eye(3) * math.cos(angle)
        + (1 - math.cos(angle)) * np.outer(axis, axis)
        + math.sin(angle) * skew
    )


def origin(element):
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [scalar(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [scalar(v) for v in element.get("rpy", "0 0 0").split()]
    transform[:3, :3] = (
        rotation([0, 0, 1], rpy[2])
        @ rotation([0, 1, 0], rpy[1])
        @ rotation([1, 0, 0], rpy[0])
    )
    transform[:3, 3] = xyz
    return transform


def transform_points(transform, points):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def load_capsules():
    source = (ROOT / "src/safety_filter/src/cbf_safety_filter.cpp").read_text()
    if "Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ())" not in source:
        raise ValueError("Hand mounting yaw changed; update URDF comparison")
    body = source.split("capsules_ = {", 1)[1].split("\n  };", 1)[0]
    body = re.sub(r"//[^\n]*", "", body)
    pattern = re.compile(
        r"\{\s*(\d+),\s*(-?\d+),\s*(flange_from_hand\s*\*\s*)?"
        r"(?:Eigen::Vector3d)?\s*\{([^}]+)\},\s*"
        r"(flange_from_hand\s*\*\s*)?(-?)Eigen::Vector3d::Unit([XYZ])\(\),"
        r"\s*([\d.]+),\s*([\d.]+)\s*\}"
    )
    capsules = []
    hand_rotation = rotation([0, 0, 1], -math.pi / 4)
    for match in pattern.finditer(body):
        link, frame, rotate_center, center, rotate_axis, sign, axis, length, radius = (
            match.groups()
        )
        center = np.array([float(v) for v in center.split(",")])
        direction = np.eye(3)["XYZ".index(axis)] * (-1 if sign else 1)
        if rotate_center:
            center = hand_rotation @ center
        if rotate_axis:
            direction = hand_rotation @ direction
        capsules.append(
            dict(
                id=len(capsules),
                link=int(link),
                frame=int(frame),
                center=center,
                direction=direction,
                length=float(length),
                radius=float(radius),
            )
        )
    if len(capsules) != 14 or body.count("Eigen::Vector3d::Unit") != 14:
        raise ValueError("Capsule initializer changed; update parser before rendering")
    return capsules


def check_urdf(capsules, arm, hand):
    """Collapse cylinder/end-sphere triples, retaining standalone flange spheres."""
    expected = []
    for tree in (arm, hand):
        for link in tree.findall(".//link"):
            name = link.get("name")
            if name.endswith("_hand"):
                index = 9
            elif re.search(r"_link\d+$", name):
                index = int(name.rsplit("link", 1)[1])
            else:
                continue
            cylinders, spheres = [], []
            for collision in link.findall("collision"):
                pose = origin(collision.find("origin"))
                cylinder = collision.find("geometry/cylinder")
                sphere = collision.find("geometry/sphere")
                if cylinder is not None:
                    cylinders.append(
                        (pose[:3, 3], pose[:3, 2], scalar(cylinder.get("length")),
                         scalar(cylinder.get("radius")))
                    )
                if sphere is not None:
                    spheres.append((pose[:3, 3], scalar(sphere.get("radius"))))
            for center, radius in spheres:
                if not any(
                    abs(radius - r) < 1e-9
                    and any(np.allclose(center, c + s * d * length / 2, atol=1e-9)
                            for s in (-1, 1))
                    for c, d, length, r in cylinders
                ):
                    cylinders.append((center, np.array([0., 0., 1.]), 0., radius))
            for center, direction, length, radius in cylinders:
                if index == 9:
                    turn = rotation([0, 0, 1], -math.pi / 4)
                    center, direction = turn @ center, turn @ direction
                expected.append((index, center, direction, length, radius))
    assert len(expected) == len(capsules) == 14
    for capsule, (link, center, direction, length, radius) in zip(capsules, expected):
        assert capsule["link"] == link
        np.testing.assert_allclose(capsule["center"], center, atol=1e-9)
        assert abs(capsule["length"] - length) < 1e-9
        assert abs(capsule["radius"] - radius) < 1e-9
        if length:
            assert abs(abs(capsule["direction"] @ direction) - 1) < 1e-9


def link_poses(arm, q):
    poses = [np.eye(4)]
    for j in range(1, 9):
        joint = next(e for e in arm.findall(".//joint")
                     if e.get("name") == "${arm_id}_joint" + str(j))
        pose = poses[-1] @ origin(joint.find("origin"))
        if j <= 7:
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            turn = np.eye(4)
            turn[:3, :3] = rotation(axis, q[j - 1])
            pose = pose @ turn
        poses.append(pose)
    return poses


def load_mesh(path):
    """Read local COLLADA triangles and node matrices without external libraries."""
    root = ET.parse(path).getroot()
    assert root.find("c:asset/c:unit", COLLADA).get("meter") == "1"
    assert root.find("c:asset/c:up_axis", COLLADA).text == "Z_UP"
    geometry = {}
    for entry in root.findall("c:library_geometries/c:geometry", COLLADA):
        mesh = entry.find("c:mesh", COLLADA)
        sources = {}
        for source in mesh.findall("c:source", COLLADA):
            stride = int(source.find("c:technique_common/c:accessor", COLLADA).get("stride"))
            sources[source.get("id")] = np.fromstring(
                source.find("c:float_array", COLLADA).text, sep=" "
            ).reshape(-1, stride)
        vertices = mesh.find("c:vertices", COLLADA)
        position = next(i.get("source")[1:] for i in vertices
                        if i.get("semantic") == "POSITION")
        triangles = []
        for element in mesh.findall("c:triangles", COLLADA):
            inputs = element.findall("c:input", COLLADA)
            stride = max(int(i.get("offset")) for i in inputs) + 1
            offset = next(int(i.get("offset")) for i in inputs
                          if i.get("semantic") == "VERTEX")
            indices = np.fromstring(element.find("c:p", COLLADA).text, sep=" ", dtype=int)
            indices = indices.reshape(-1, 3, stride)[:, :, offset]
            triangles.append(sources[position][indices, :3])
        geometry[entry.get("id")] = np.concatenate(triangles)
    result = []
    for node in root.findall("c:library_visual_scenes/c:visual_scene/c:node", COLLADA):
        matrix = node.find("c:matrix", COLLADA)
        pose = (np.fromstring(matrix.text, sep=" ").reshape(4, 4)
                if matrix is not None else np.eye(4))
        for instance in node.findall("c:instance_geometry", COLLADA):
            triangles = geometry[instance.get("url")[1:]]
            result.append(transform_points(pose, triangles))
    return np.concatenate(result)


def box_edges(low, high):
    vertices = np.array(list(itertools.product(*zip(low, high))))
    return np.array([[a, b] for i, a in enumerate(vertices)
                     for j, b in enumerate(vertices) if (i ^ j) in (1, 2, 4) and i < j])


def capsule_surface(a, b, radius):
    axis = b - a
    length = np.linalg.norm(axis)
    axis = axis / length if length > 1e-10 else np.array([0., 0., 1.])
    perpendicular = np.cross(axis, [1., 0., 0.] if abs(axis[0]) < .8 else [0., 1., 0.])
    perpendicular /= np.linalg.norm(perpendicular)
    other = np.cross(axis, perpendicular)
    phi = np.r_[np.linspace(-math.pi / 2, 0, 9), np.linspace(0, math.pi / 2, 9)]
    theta = np.linspace(0, 2 * math.pi, 25)
    axial = radius * np.sin(phi)
    axial[9:] += length
    surface = (a + axial[:, None, None] * axis
               + radius * np.cos(phi)[:, None, None]
               * (np.cos(theta)[None, :, None] * perpendicular
                  + np.sin(theta)[None, :, None] * other))
    return surface


def capsule_projection(a, b, radius):
    direction = b - a
    angle = math.atan2(direction[1], direction[0])
    first = np.linspace(angle + math.pi / 2, angle + 3 * math.pi / 2, 40)
    second = np.linspace(angle - math.pi / 2, angle + math.pi / 2, 40)
    return np.concatenate([a + radius * np.c_[np.cos(first), np.sin(first)],
                           b + radius * np.c_[np.cos(second), np.sin(second)]])


def style_3d(ax, limits):
    ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2],
           xlabel="world x [m]", ylabel="world y [m]", zlabel="world z [m]")
    ax.set_box_aspect([b - a for a, b in limits])
    ax.view_init(elev=22, azim=-62)
    ax.tick_params(labelsize=8)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((.96, .97, .98, 1))


def draw_figures(output, capsules, meshes):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.titleweight": "bold", "savefig.facecolor": "white"})
    fig = plt.figure(figsize=(17, 9.5), layout="constrained")
    grid = fig.add_gridspec(2, 3, height_ratios=[1, .20])
    ax = fig.add_subplot(grid[0, 0], projection="3d")
    for mesh in meshes:
        ax.add_collection3d(Poly3DCollection(mesh, facecolor="#98a6b8", edgecolor="none",
                                           alpha=.13, rasterized=True))
    for c in capsules:
        surface = capsule_surface(c["a"], c["b"], c["radius"])
        ax.plot_surface(*np.moveaxis(surface, -1, 0), color=COLORS[c["id"]], alpha=.65,
                        linewidth=0, shade=True, rasterized=True)
        ax.plot(*np.array([c["a"], c["b"]]).T, color="#1d2636", lw=1)
    limits = [(-.28, .60), (-.22, .22), (-.08, .94)]
    style_3d(ax, limits)
    ax.set_title("A  URDF visual meshes + CBF capsules", pad=16)

    for column, coordinates, name in ((1, [0, 2], "B  Front projection: x–z"),
                                       (2, [1, 2], "C  Side projection: y–z")):
        axis = fig.add_subplot(grid[0, column])
        for mesh in meshes:
            axis.add_collection(PolyCollection(mesh[:, :, coordinates], facecolors="#dbe0e7",
                                               edgecolors="none", alpha=.24, rasterized=True))
        for c in capsules:
            a, b = c["a"][coordinates], c["b"][coordinates]
            axis.add_patch(Polygon(capsule_projection(a, b, c["radius"]),
                                   facecolor=COLORS[c["id"]], edgecolor=COLORS[c["id"]],
                                   alpha=.24, lw=1))
            edges = box_edges(c["bounds_min"], c["bounds_max"])[:, :, coordinates]
            for edge in edges:
                axis.plot(*edge.T, color="#142f53", lw=1.2, alpha=.85)
        axis.set(xlim=limits[coordinates[0]], ylim=limits[coordinates[1]],
                 xlabel=f"world {'xyz'[coordinates[0]]} [m]", ylabel="world z [m]", title=name)
        axis.set_aspect("equal")
        axis.grid(alpha=.18)
    legend_ax = fig.add_subplot(grid[1, :])
    legend_ax.axis("off")
    handles = [Line2D([0], [0], color=COLORS[c["id"]], lw=7,
                      label=f"C{c['id']:02d}  {'hand' if c['link'] == 9 else 'link' + str(c['link'])}")
               for c in capsules]
    legend_ax.legend(handles=handles, ncol=7, loc="upper center", frameon=False, fontsize=10)
    legend_ax.text(.5, .20,
                   "Colored volumes: capsule collision model.  Dark blue: stored centerline AABBs.\n"
                   "The boxes exclude radius; radius and safety margins enter the pair-distance threshold separately.",
                   ha="center", va="center", transform=legend_ax.transAxes, color="#263b55")
    fig.suptitle("Panda collision geometry at the configured home posture", fontsize=21, weight="bold")
    fig.supxlabel("Nominal pinned URDF kinematics • world = robot base • dimensions in meters • "
                  "arm + hand housing visual meshes; finger meshes omitted", fontsize=10)
    fig.savefig(output / "cbf_geometry_home.png", dpi=180)
    fig.savefig(output / "cbf_geometry_home.pdf", dpi=180)
    plt.close(fig)

    fig = plt.figure(figsize=(15, 9), layout="constrained")
    grid = fig.add_gridspec(2, 2, height_ratios=[1, .29])
    ax = fig.add_subplot(grid[0, 0], projection="3d")
    c = capsules[3]
    surface = capsule_surface(c["a"], c["b"], c["radius"])
    ax.plot_surface(*np.moveaxis(surface, -1, 0), color="#50a9c6", alpha=.42,
                    shade=True, linewidth=0)
    ax.add_collection3d(Line3DCollection(box_edges(c["bounds_min"], c["bounds_max"]),
                                        color="#142f53", linewidth=2.4))
    low, high = c["bounds_min"] - c["radius"], c["bounds_max"] + c["radius"]
    ax.add_collection3d(Line3DCollection(box_edges(low, high), color="#d88412",
                                        linewidth=1.8, linestyle="--"))
    ax.plot(*np.array([c["a"], c["b"]]).T, color="#142f53", lw=2)
    ax.scatter(*np.array([c["a"], c["b"]]).T, color="#142f53", s=30)
    ax.text(*c["a"], "  endpoint A", fontsize=9)
    ax.text(*c["b"], "  endpoint B", fontsize=9)
    style_3d(ax, list(zip(low - .025, high + .025)))
    ax.set_title("D  C03 / link3: what is actually bounded", pad=15)

    ax = fig.add_subplot(grid[0, 1], projection="3d")
    for c in capsules[8:]:
        surface = capsule_surface(c["a"], c["b"], c["radius"])
        ax.plot_surface(*np.moveaxis(surface, -1, 0), color=COLORS[c["id"]], alpha=.38,
                        shade=True, linewidth=0)
        ax.add_collection3d(Line3DCollection(box_edges(c["bounds_min"], c["bounds_max"]),
                                            color="#142f53", linewidth=1.6))
        center = (c["a"] + c["b"]) / 2
        label_height = {8: .75, 9: .58, 10: .63, 11: .68, 12: .52, 13: .46}
        label = np.array([.45, .10, label_height[c["id"]]])
        ax.plot(*np.array([center, label]).T, color=COLORS[c["id"]], lw=1.4)
        ax.text(*label, f"C{c['id']:02d}", fontsize=11, weight="bold", color=COLORS[c["id"]])
    style_3d(ax, [(.20, .52), (-.13, .15), (.38, .78)])
    ax.set_title("E  Wrist, flange and hand detail", pad=15)
    note = fig.add_subplot(grid[1, :])
    note.axis("off")
    note.legend(handles=[Line2D([0], [0], color="#142f53", lw=2.4, label="Stored centerline AABB"),
                         Line2D([0], [0], color="#d88412", lw=1.8, ls="--",
                                label="Radius-expanded envelope: illustration only")],
                loc="upper left", ncol=2, fontsize=10, frameon=False)
    note.text(.02, .66,
              "A = T_world,frame · (center − length/2 · axis)     "
              "B = T_world,frame · (center + length/2 · axis)\n"
              "stored lower corner = min(A, B)     stored upper corner = max(A, B) "
              "[component by component]\n"
              "These boxes stay aligned with world x/y/z. At this posture some are flat or lines; "
              "C10/C11 are spheres, so their stored boxes are points.\n"
              "C12/C13 are attached to the flange with the URDF hand yaw −45°. "
              "They do not use a configurable TCP frame.",
              transform=note.transAxes, va="top", fontsize=11, linespacing=1.7, color="#263b55")
    fig.suptitle("Exact stored boxes versus the capsule envelope", fontsize=21, weight="bold")
    fig.savefig(output / "cbf_geometry_boxes.png", dpi=180)
    fig.savefig(output / "cbf_geometry_boxes.pdf", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "src/teleop_bringup/config/single_panda.yaml")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "docs/figures/cbf_geometry")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    q = np.asarray(config["controller"]["home_joint_position"], dtype=float)
    arm = ET.parse(DESCRIPTION / "robots/panda_arm.xacro").getroot()
    hand = ET.parse(DESCRIPTION / "robots/hand.xacro").getroot()
    capsules = load_capsules()
    check_urdf(capsules, arm, hand)
    poses = link_poses(arm, q)
    meshes = [transform_points(poses[index], load_mesh(DESCRIPTION / f"meshes/visual/link{index}.dae"))
              for index in range(8)]
    hand_pose = poses[8].copy()
    hand_pose[:3, :3] = hand_pose[:3, :3] @ rotation([0, 0, 1], -math.pi / 4)
    meshes.append(transform_points(hand_pose, load_mesh(DESCRIPTION / "meshes/visual/hand.dae")))
    rows = []
    for c in capsules:
        pose = poses[c["frame"] + 1] if c["frame"] >= 0 else poses[0]
        ends = transform_points(pose, [c["center"] - c["length"] / 2 * c["direction"],
                                       c["center"] + c["length"] / 2 * c["direction"]])
        c.update(a=ends[0], b=ends[1], bounds_min=ends.min(axis=0), bounds_max=ends.max(axis=0))
        row = dict(capsule=f"C{c['id']:02d}", link="hand" if c["link"] == 9 else f"link{c['link']}",
                   model_frame_index=c["frame"], frame="base" if c["frame"] < 0 else f"link{c['frame']+1}",
                   length_m=c["length"], radius_m=c["radius"])
        for field, value in (("center_local", c["center"]), ("axis_local", c["direction"]),
                             ("endpoint_a", c["a"]), ("endpoint_b", c["b"]),
                             ("box_min", c["bounds_min"]), ("box_max", c["bounds_max"])):
            row.update({f"{field}_{axis}": float(v) for axis, v in zip("xyz", value)})
        rows.append(row)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "placement.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / "placement.json").write_text(json.dumps(
        dict(home_joint_position_rad=q.tolist(), world_from_base=np.eye(4).tolist(),
             note="Nominal URDF home posture. All 14 capsules shown regardless of enabled families. "
                  "Stored bounds enclose centerlines only; radius and safety reserves enter pair thresholds separately.",
             capsules=rows), indent=2) + "\n")
    draw_figures(args.output_dir, capsules, meshes)
    print(f"Verified all 14 C++ capsules against pinned URDF; rendered {sum(len(m) for m in meshes):,} visual triangles.")
    print(f"Home q [rad]: {q.tolist()}")
    print(f"Figures and exact coordinate exports: {args.output_dir}")


if __name__ == "__main__":
    main()
