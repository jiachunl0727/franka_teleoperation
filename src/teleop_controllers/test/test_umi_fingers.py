"""Verify generated attachment against measured mass and continuous sweep bounds."""
import importlib.util
import itertools
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("umi", ROOT / "scripts/generate_umi_finger_model.py")
umi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(umi)


class UmiModelTest(unittest.TestCase):
    def test_volume_integral_against_analytic_tetrahedron(self):
        # Closed unit tetrahedron: volume=1/6, CoM=(1/4,1/4,1/4),
        # unit-mass inertia diag=3/40, products=1/80.
        vertices = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype=float)
        faces = [[0,2,1], [0,1,3], [0,3,2], [1,2,3]]
        data = b" " * 80 + struct.pack("<I", len(faces))
        for face in faces:
            data += struct.pack("<12fH", *([0.] * 3), *vertices[face].ravel(), 0)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "tetra.stl"
            path.write_bytes(data)
            _, volume, com, inertia = umi.mesh_properties(path, 1.)
        self.assertAlmostEqual(volume, 1/6)
        np.testing.assert_allclose(com, [.25]*3)
        np.testing.assert_allclose(inertia, np.full((3,3), 1/80) + np.eye(3)/16)

    def test_measured_mass_retained_stock_and_outward_tip(self):
        text, _ = umi.generated()
        robot = ET.fromstring(text)
        for side in ["left", "right"]:
            self.assertIsNotNone(robot.find(f"link[@name='panda_{side}finger']"))
            link = robot.find(f"link[@name='panda_umi_{side}finger']")
            self.assertEqual(float(link.find("inertial/mass").get("value")), .005)
            self.assertIsNotNone(link.find("visual/geometry/mesh"))
            self.assertIsNotNone(link.find("collision/geometry/box"))
        settings, vertices, _, _, inertia, poses, *_ = umi.properties()
        self.assertGreater(np.linalg.eigvalsh(inertia).min(), 0)
        for rotation, _ in poses:
            np.testing.assert_allclose(rotation @ [0,0,-1], [0,0,1])
        r, t = poses[0]
        np.testing.assert_allclose(r @ settings["mesh_mount_xyz_m"] + t,
                                   settings["stock_finger_mount_xyz_m"])

    def test_continuous_box_cover_and_both_opening_sweeps(self):
        settings, vertices, _, _, _, poses, lower, upper, spheres = umi.properties()
        # Independent nearest-centre tests at every cell's corners, covering
        # the full cell by convexity (not merely discrete opening samples).
        half = (upper-lower)/np.array([2,12,4])
        for centre in [lower+(np.array(index)*2+1)*half
                       for index in itertools.product(range(1),range(6),range(2))]:
            for sign in itertools.product([-1,1], repeat=3):
                self.assertLessEqual(np.linalg.norm(half*np.array(sign)), spheres[0]["radius_m"]+1e-12)
        for sign, (r,t) in zip([1,-1],poses):
            for opening in np.linspace(0,.04,17):
                points = vertices @ r.T+t+[0,sign*opening,.0584]
                self.assertTrue((points >= lower).all() and (points <= upper).all())

    def test_all_shipped_profiles_use_the_same_moving_finger_and_body_volumes(self):
        config = ROOT / "src/cbf_safety_filter/config"
        paths = [config / "single_panda/geometry_panda_umi.yaml", config / "dual_panda/geometry.left.yaml",
                 config / "dual_panda/geometry.right.yaml"]
        geometry = [yaml.safe_load(path.read_text()) for path in paths]
        expected = geometry[0]["body_volumes"]
        for document in geometry:
            volumes = document["body_volumes"]
            self.assertEqual(volumes, expected)
            self.assertEqual(len(volumes), 15)
            self.assertEqual(sum("end" in volume for volume in volumes), 12)
            self.assertEqual(sum("opening_axis" in volume for volume in volumes), 4)
            self.assertNotIn("body_spheres", document)
            self.assertNotIn("capsules", document)
            self.assertIn([9, 10], document["self_exclusions"])
            self.assertFalse(any(b == 10 and a < 9 for a, b in document["self_exclusions"]))


if __name__ == "__main__":
    unittest.main()
