#!/usr/bin/env python3
"""Render current URDF kinematics and actual CBF covers without ROS or hardware.

NumPy, SciPy and PyYAML only. Panda surfaces use the archived Franka visual
meshes, and the UMI attachments use the supplied STL.
"""
import argparse
import base64
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/teleop_controllers/config/panda_umi.urdf"
DUAL_CONFIG = ROOT / "src/cbf_safety_filter/config/dual_panda/cbf.yaml"
REPLAY = ROOT / "log/commissioning/2026-11-09-fingers-avoidance"
ASSETS = ROOT / "log/commissioning/2026-11-06-oscbf-model/sources/oscbf-6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/assets/franka_panda"


def transform(element):
    result = np.eye(4)
    if element is not None:
        result[:3, :3] = Rotation.from_euler("xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")).as_matrix()
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    return result


def franka_visual_mesh(path):
    """Read the triangle surfaces and normals in the pinned Franka DAE files."""
    root = ET.parse(path).getroot()
    for element in root.iter():
        element.tag = element.tag.rsplit("}", 1)[-1]
    if root.findtext("asset/up_axis") != "Z_UP" or float(root.find("asset/unit").get("meter")) != 1:
        raise ValueError(f"Expected a metre-scale Z-up visual mesh: {path}")

    geometries = {g.get("id"): g.find("mesh") for g in root.findall("library_geometries/geometry")}
    vertices, normals = [], []

    def visit(node, parent):
        non_matrix = {"translate", "rotate", "scale", "lookat", "skew", "instance_node"}
        if any(element.tag in non_matrix for element in node):
            raise ValueError(f"Unsupported visual scene transform: {path}")
        matrix = parent.copy()
        for element in node.findall("matrix"):
            matrix = matrix @ np.fromstring(element.text, sep=" ").reshape(4, 4)
        normal_matrix = np.linalg.inv(matrix[:3, :3]).T
        for instance in node.findall("instance_geometry"):
            mesh = geometries[instance.get("url")[1:]]
            sources = {
                source.get("id"): np.fromstring(source.findtext("float_array"), sep=" ").reshape(-1, 3)
                for source in mesh.findall("source")
                if source.find("technique_common/accessor").get("stride") == "3"
            }
            position_ids = {
                vertex.get("id"): vertex.find("input[@semantic='POSITION']").get("source")[1:]
                for vertex in mesh.findall("vertices")
            }
            for triangles in mesh.findall("triangles"):
                position = triangles.find("input[@semantic='VERTEX']")
                normal = triangles.find("input[@semantic='NORMAL']")
                stride = max(int(item.get("offset")) for item in triangles.findall("input")) + 1
                corners = np.fromstring(triangles.findtext("p"), dtype=int, sep=" ").reshape(-1, stride)
                points = sources[position_ids[position.get("source")[1:]]][corners[:, int(position.get("offset"))]]
                directions = sources[normal.get("source")[1:]][corners[:, int(normal.get("offset"))]]
                vertices.append(points @ matrix[:3, :3].T + matrix[:3, 3])
                normals.append(directions @ normal_matrix.T)
        for child in node.findall("node"):
            visit(child, matrix)

    scene_id = root.find("scene/instance_visual_scene").get("url")[1:]
    scene = root.find(f"library_visual_scenes/visual_scene[@id='{scene_id}']")
    for node in scene.findall("node"):
        visit(node, np.eye(4))
    # Keep the full triangle mesh. Deduplicate identical corners and pack the
    # display data so the standalone HTML stays small.
    corners = np.column_stack((np.concatenate(vertices), np.concatenate(normals))).astype("<f4")
    samples, indices = np.unique(corners, axis=0, return_inverse=True)
    return dict(
        samples=base64.b64encode(samples.tobytes()).decode("ascii"),
        indices=base64.b64encode(indices.astype("<u4").tobytes()).decode("ascii"),
        triangles=len(indices) // 3,
    )


def capsule_mesh(a, b, radius):
    a, b = np.asarray(a), np.asarray(b)
    delta = b - a
    length = np.linalg.norm(delta)
    axis = delta / length if length > 1e-12 else np.array([0., 0., 1.])
    perpendicular = np.cross(axis, [1, 0, 0] if abs(axis[0]) < .8 else [0, 1, 0])
    perpendicular /= np.linalg.norm(perpendicular)
    other = np.cross(axis, perpendicular)
    vertices = []
    # Parametric hemispheres joined by a cylinder; length=0 is a sphere.
    sides = 24
    rings = [(theta, b) for theta in np.linspace(0, np.pi / 2, 9)]
    rings += [(theta, a) for theta in np.linspace(np.pi / 2, np.pi, 9)]
    for theta, centre in rings:
        for phi in np.linspace(0, 2 * np.pi, sides, endpoint=False):
            vertices.append(centre + radius * (np.cos(theta) * axis + np.sin(theta) *
                                               (np.cos(phi) * perpendicular + np.sin(phi) * other)))
    faces = []
    for i in range(len(rings) - 1):
        for j in range(sides):
            p, n = i * sides + j, i * sides + (j + 1) % sides
            faces.extend([[p, n, p + sides], [n, n + sides, p + sides]])
    return np.asarray(vertices), np.asarray(faces)


def build_scene(side):
    robot = ET.parse(MODEL).getroot()
    config = yaml.safe_load(DUAL_CONFIG.read_text())
    geometry_path = DUAL_CONFIG.parent / config["geometry"]["files_by_controller"][side]
    geometry = yaml.safe_load(geometry_path.read_text())
    spec = importlib.util.spec_from_file_location("umi", ROOT / "scripts/generate_umi_finger_model.py")
    umi = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(umi)
    settings, _, _, _, _, _, box_lower, box_upper, _ = umi.properties()
    joints = []
    for joint in robot.findall("joint"):
        joints.append(dict(name=joint.get("name"), parent=joint.find("parent").get("link"),
                           child=joint.find("child").get("link"), type=joint.get("type"),
                           origin=transform(joint.find("origin")),
                           axis=np.fromstring(joint.find("axis").get("xyz"), sep=" ") if joint.find("axis") is not None else np.array([0., 0., 1.])))
    meshes, drawables = {}, []
    for link in robot.findall("link"):
        name = link.get("name")
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename")
            if name.startswith("panda_umi"):
                key = "umi"
                path = ROOT / "src/teleop_controllers/config/meshes/umi_v1.stl"
                vertices = umi.mesh_properties(path, .001)[0]
                faces = np.arange(len(vertices)).reshape(-1, 3)
                meshes[key] = dict(vertices=vertices, faces=faces)
                colour = [1., .56, .09]
            else:
                key = Path(filename).stem
                if key not in meshes:
                    # The URDF uses collision meshes as visuals for links 0 and 7.
                    meshes[key] = franka_visual_mesh(ASSETS / "meshes/visual" / f"{key}.dae")
                colour = [.59, .64, .71] if "finger" not in name else [.76, .79, .83]
            drawables.append(dict(key=key, link=name, origin=transform(visual.find("origin")),
                                  side=1 if name in ["panda_leftfinger", "panda_umi_leftfinger"] else
                                  (-1 if name in ["panda_rightfinger", "panda_umi_rightfinger"] else 0), colour=colour))

    def fk(q):
        links = {"panda_link0": np.eye(4)}
        pending = list(joints)
        while pending:
            progressed = False
            for joint in pending[:]:
                if joint["parent"] not in links:
                    continue
                local = joint["origin"].copy()
                if joint["type"] == "revolute":
                    turn = np.eye(4)
                    turn[:3, :3] = Rotation.from_rotvec(joint["axis"] * q[int(joint["name"].replace("panda_joint", "")) - 1]).as_matrix()
                    local = local @ turn
                links[joint["child"]] = links[joint["parent"]] @ local
                pending.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError("disconnected URDF")
        ee = links["panda_link8"] @ transform(ET.fromstring('<origin xyz="0 0 .1034" rpy="0 0 -0.7853981633974483"/>'))
        frames = [links[f"panda_link{i}"] for i in range(1, 8)] + [links["panda_link8"], ee]
        return links, frames

    def recorded(trial, cycle):
        data = np.fromfile(REPLAY / trial / "recorded-input.bin", dtype="<f8", offset=24).reshape(-1, 54)
        row = data[np.flatnonzero(data[:, 0] == cycle)[0]]
        return row[2:9], row[45:48], row[48:52]

    home = yaml.safe_load((ROOT / "src/teleop_bringup/config/dual_panda/dual_panda.yaml").read_text())["controller"]["home_joint_position"]
    choices = [("Configured home", home, None),
               ("Recorded first table correction · cycle 39366", *recorded("contact", 39366)[:1], recorded("contact", 39366)[1:]),
               ("Recorded closest table state · cycle 45918", *recorded("contact", 45918)[:1], recorded("contact", 45918)[1:]),
               ("Recorded minimum singularity margin · 0.310 vs 0.025", *recorded("free-space", 26386)[:1], recorded("free-space", 26386)[1:])]
    base = np.eye(4)
    xyz_xyzw = geometry["world_from_base_xyz_xyzw"]
    base[:3, :3] = Rotation.from_quat(xyz_xyzw[3:]).as_matrix()
    base[:3, 3] = xyz_xyzw[:3]
    poses = []
    errors = []
    for label, q, measured in choices:
        links, frames = fk(q)
        if measured is not None:
            error = max(np.max(np.abs(frames[8][:3, 3] - measured[0])),
                        np.max(np.abs(frames[8][:3, :3] - Rotation.from_quat(measured[1]).as_matrix())))
            errors.append(float(error))
            if error > 1e-6:
                raise ValueError("visualization FK differs from recorded measured pose")
        frames = [base @ frame for frame in frames]
        matrices = [base @ links[d["link"]] @ d["origin"] for d in drawables]
        poses.append(dict(label=label, q=np.asarray(q), matrices=matrices, frames=frames,
                          hand=base @ links["panda_hand"]))
    primitives = []
    part_counts = {}
    for body in geometry["body_volumes"]:
        group = body["group"]
        index = part_counts.get(group, 0)
        part_counts[group] = index + 1
        if group == 9:
            label = ["Hand housing", "Left stock finger", "Right stock finger"][index]
        elif group == 10:
            label = ["Left UMI fingertip", "Right UMI fingertip"][index]
        elif group == 5:
            label = ["Link 5 · main", "Link 5 · offset section"][index]
        else:
            label = {0: "Base", 8: "Wrist flange"}.get(group, f"Link {group}")
        vertices, faces = capsule_mesh(body["offset"], body.get("end", body["offset"]), body["radius_m"])
        fingertip = group == 10
        primitives.append(dict(frame=body["frame"], vertices=vertices,
                               faces=faces, label=label, added=fingertip, table=group >= 2,
                               opening_axis=body.get("opening_axis"),
                               colour=[.91, .18, .50] if fingertip else [.08, .63, .38]))
    return dict(meshes=meshes, drawables=drawables, poses=poses, primitives=primitives,
                base=base, box_lower=box_lower, box_upper=box_upper, fk_errors=errors,
                settings=settings, table_height=config["constraints"]["table"]["height_m"],
                clearance=config["constraints"]["table"]["clearance_m"],
                inter_arm_clearance=config["constraints"]["inter_arm_collision"]["clearance_m"],
                geometry_source=str(geometry_path.relative_to(ROOT)))


def write_viewer(scene, out):
    def encode(value):
        if isinstance(value, np.ndarray):return np.round(value, 9).tolist()
        raise TypeError(type(value).__name__)
    payload = json.dumps(scene, default=encode, separators=(",", ":")).replace("<", "\\u003c")
    template = (ROOT / "scripts/umi_model_viewer.html").read_text()
    (out / "viewer.html").write_text(template.replace("__MODEL_DATA__", payload))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/figures/collision_model")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    arms = {side: build_scene(side) for side in ("left", "right")}
    common = {key: arms["left"][key] for key in ("meshes", "drawables", "box_lower", "box_upper")}
    for scene in arms.values():
        for key in common:
            del scene[key]
    write_viewer(dict(**common, arms=arms), args.output)


if __name__ == "__main__":
    main()
