#!/usr/bin/env python3
"""Generate sphere covers of pinned Franka primitives and the complete hand mesh.

No mesh processing or model queries in a control callback. Source dimensions are
un-padded; the CBF clearance is applied separately. Run from the repository root.
"""
import ast
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "src/third_party/franka_arm_ros2/franka_description"


def number(text):
    text = (
        text.replace("${", "")
        .replace("}", "")
        .replace("safety_distance", "0")
        .replace("pi", str(math.pi))
    )
    node = ast.parse(text, mode="eval")
    if any(
        not isinstance(
            n,
            (
                ast.Expression,
                ast.Constant,
                ast.BinOp,
                ast.UnaryOp,
                ast.Add,
                ast.Sub,
                ast.Mult,
                ast.Div,
                ast.USub,
                ast.UAdd,
            ),
        )
        for n in ast.walk(node)
    ):
        raise ValueError("unsupported source expression: " + text)
    return float(eval(compile(node, "<numeric xacro>", "eval"), {"__builtins__": {}}))


def vector(text):
    return [number(v) for v in re.findall(r"\$\{[^}]+\}|[^\s]+", text)]


def rotate(v, rpy):
    x, y, z = v
    a, b, c = rpy
    y, z = math.cos(a) * y - math.sin(a) * z, math.sin(a) * y + math.cos(a) * z
    x, z = math.cos(b) * x + math.sin(b) * z, -math.sin(b) * x + math.cos(b) * z
    return [math.cos(c) * x - math.sin(c) * y, math.sin(c) * x + math.cos(c) * y, z]


def mesh_vertices(name):
    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    tree = ET.parse(DESCRIPTION / "meshes/visual" / (name + ".dae"))
    arrays = {n.attrib["id"]: n for n in tree.findall(".//c:source", namespace)}
    points = []
    for vertices in tree.findall(".//c:vertices", namespace):
        source = vertices.find("c:input[@semantic='POSITION']", namespace).attrib[
            "source"
        ][1:]
        values = [
            float(v)
            for v in arrays[source].find("c:float_array", namespace).text.split()
        ]
        points.extend(zip(values[::3], values[1::3], values[2::3]))
    if not points:
        raise ValueError(name + " source has no position vertices")
    return points


def finger_vertices():
    return mesh_vertices("finger")


def hand_housing_cover():
    # The primitive hand collision model omits parts of the actual housing.
    # Partition the visual mesh's enclosing box along its long axis. Each
    # sphere encloses a complete cell, so triangle interiors are covered too.
    vertices = mesh_vertices("hand")
    low = [min(p[j] for p in vertices) for j in range(3)]
    high = [max(p[j] for p in vertices) for j in range(3)]
    half = [(high[j] - low[j]) / (8 if j == 1 else 4 if j == 2 else 2) for j in range(3)]
    radius = math.sqrt(sum(v * v for v in half))
    return [dict(frame=7, offset=rotate(
        [(low[0] + high[0]) / 2, low[1] + (2*i+1)*half[1], low[2] + (2*k+1)*half[2]],
        [0, 0, -math.pi / 4]), radius_m=radius, group=9) for i in range(4) for k in range(2)]


def check_finger_cover(spheres):
    # Independent sweep check against source mesh vertices, both fingers and
    # their full travel; the enclosing-box construction also covers triangle interiors.
    hand = [v for v in spheres if v["group"] == 9]
    count = 0
    for p in finger_vertices():
        for sign in (-1, 1):
            for i in range(9):
                position = rotate(
                    [sign * p[0], sign * (p[1] + i * 0.005), p[2] + 0.0584],
                    [0, 0, -math.pi / 4],
                )
                if not any(
                    sum((a - b) ** 2 for a, b in zip(position, s["offset"]))
                    <= s["radius_m"] ** 2 + 1e-12
                    for s in hand
                ):
                    raise ValueError("finger mesh sweep is not enclosed")
                count += 1
    return count


def generate():
    spheres = []
    for name in ("panda_arm", "hand"):
        tree = ET.parse(DESCRIPTION / "robots" / (name + ".xacro"))
        for link in tree.iter("link"):
            label = link.attrib["name"]
            if name == "hand" and not label.endswith("_hand"):
                continue
            group = 9 if name == "hand" else int(re.search(r"link(\d+)$", label)[1])
            frame = -1 if group == 0 else (7 if group in (8, 9) else group - 1)
            for collision in link.findall("collision"):
                origin = collision.find("origin")
                xyz = vector(origin.attrib.get("xyz", "0 0 0"))
                rpy = vector(origin.attrib.get("rpy", "0 0 0"))
                shape = list(collision.find("geometry"))[0]
                radius = number(shape.attrib["radius"])
                parts = [(xyz, radius)]
                if shape.tag == "cylinder":
                    length = number(shape.attrib["length"])
                    count = max(1, math.ceil(length / 0.10))
                    parts = []
                    for i in range(count):
                        dz = -length / 2 + (i + 0.5) * length / count
                        d = rotate([0, 0, dz], rpy)
                        parts.append(
                            (
                                [a + b for a, b in zip(xyz, d)],
                                math.hypot(radius, length / (2 * count)),
                            )
                        )
                for offset, r in parts:
                    if group == 9:
                        offset = rotate(offset, [0, 0, -math.pi / 4])
                    spheres.append(
                        dict(frame=frame, offset=offset, radius_m=r, group=group)
                    )
    # Fingers have visual meshes only in the source. Enclose their full swept
    # vertex bounding box over 0..40 mm travel, both sides (independent of Hand I/O).
    vertices = finger_vertices()
    xmax = max(abs(p[0]) for p in vertices)
    ymax = max(abs(p[1]) for p in vertices) + 0.04
    zmin = min(p[2] for p in vertices) + 0.0584
    zmax = max(p[2] for p in vertices) + 0.0584
    z = (zmin + zmax) / 2
    r = math.hypot(xmax, (zmax - zmin) / 2)
    length = 2 * ymax
    n = 3
    for i in range(n):
        y = -length / 2 + (i + 0.5) * length / n
        spheres.append(
            dict(
                frame=7,
                offset=rotate([0, y, z], [0, 0, -math.pi / 4]),
                radius_m=math.hypot(r, length / (2 * n)),
                group=9,
            )
        )
    spheres.extend(hand_housing_cover())
    # Remove only a sphere wholly enclosed by another sphere on the same body.
    keep = []
    for i, s in enumerate(spheres):
        contained = False
        for j, t in enumerate(spheres):
            if i == j or s["group"] != t["group"]:
                continue
            distance = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(s["offset"], t["offset"]))
            )
            if distance + s["radius_m"] <= t["radius_m"] + 1e-12 and (
                distance > 1e-12 or i > j
            ):
                contained = True
                break
        if not contained:
            keep.append(s)
    # Explicit source SRDF exclusions, with Hand/fingers represented as group9.
    pairs = {(i, i + 1) for i in range(8)}
    pairs.update(
        [
            (0, 2),
            (0, 3),
            (0, 4),
            (1, 3),
            (1, 4),
            (2, 4),
            (2, 6),
            (3, 5),
            (3, 6),
            (3, 7),
            (4, 6),
            (4, 7),
            (4, 8),
            (6, 8),
            (7, 9),
            (8, 9),
            (3, 9),
            (4, 9),
            (6, 9),
        ]
    )
    if len(keep) > 64:
        raise ValueError("sphere capacity exceeded")
    return keep, [list(v) for v in sorted(pairs)]


def main():
    generated, exclusions = generate()
    count = check_finger_cover(generated)
    print(f"Source geometry audit covered {count} finger vertices and travel samples "
          f"with {len(generated)} spheres and {len(exclusions)} exclusions.")


if __name__ == "__main__":
    main()
