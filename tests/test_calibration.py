"""Synthetic calibration regression tests; no camera, GUI, serial or model required."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("calibration", ROOT / "calibration/calibrate.py")
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.settings = cal.load_yaml(ROOT / "config/calibration.yaml")
        self.size = (1440, 1080)
        self.k = np.array([[1800., 0, 720], [0, 1810., 540], [0, 0, 1.]])
        self.d = np.array([-.035, .08, .0003, -.0002, 0.])
        self.rng = np.random.default_rng(42)

    def intrinsic_views(self):
        objects = cal.board_points(self.settings)
        views = []
        for i in range(24):
            rv = self.rng.uniform(-.5, .5, (3, 1))
            tv = np.array([[-180 + self.rng.uniform(-130, 130)],
                           [-120 + self.rng.uniform(-100, 100)], [self.rng.uniform(900, 1600)]])
            points, _ = cv2.projectPoints(objects, rv, tv, self.k, self.d)
            views.append({"path": str(i), "points": points, "pose": None})
        return views

    def test_intrinsic_recovery_and_wust_shapes(self):
        info, report = cal.calibrate_camera(self.size, self.intrinsic_views(), self.settings, "test")
        k, d = cal.read_intrinsics(info, self.size)
        np.testing.assert_allclose(k, self.k, atol=.08)
        np.testing.assert_allclose(d.ravel(), self.d, atol=.001)
        self.assertLess(report["reprojection_rms_px"], .001)
        p = np.array(info["projection_matrix"]["data"]).reshape(3, 4)
        np.testing.assert_allclose(p[:, :3], k)
        np.testing.assert_array_equal(p[:, 3], 0)
        self.assertEqual(len(info["distortion_coefficients"]["data"]), 5)

    def handeye_views(self, robotworld):
        objects = cal.board_points(self.settings, robotworld)
        r_camera_gimbal = cv2.Rodrigues(np.array([.01, -.02, .03]))[0] @ cal.CV_TO_PHYSICS
        t_camera_gimbal = np.array([[130.], [4.], [94.]])
        # Fixed world board, same physical plane for both board coordinate definitions.
        r_board_world = np.eye(3) if robotworld else cal.CV_TO_PHYSICS
        t_board_world = np.array([[1800.], [0.], [0.]])
        views = []
        for i in range(25):
            g = cv2.Rodrigues(self.rng.uniform(-.3, .3, 3))[0]
            rcw = g @ r_camera_gimbal
            tcw = g @ t_camera_gimbal
            rb = rcw.T @ r_board_world
            tb = rcw.T @ (t_board_world-tcw)
            points, _ = cv2.projectPoints(objects, cv2.Rodrigues(rb)[0], tb, self.k, self.d)
            views.append({"path": str(i), "points": points, "pose": g})
        return views, r_camera_gimbal, t_camera_gimbal

    def test_both_handeye_solvers_and_coordinate_conversion(self):
        info = cal.camera_info(self.size, self.k, self.d, "test")
        for robotworld in (False, True):
            with self.subTest(robotworld=robotworld):
                views, r, t = self.handeye_views(robotworld)
                transform, report = cal.calibrate_handeye(self.size, views, self.settings, info, robotworld)
                exported_r = np.array(transform["R"]).reshape(3, 3)
                np.testing.assert_allclose(exported_r @ cal.CV_TO_PHYSICS, r, atol=1e-4)
                np.testing.assert_allclose(transform["t"], t.ravel()/1000, atol=1e-4)
                self.assertLess(report["board_translation_rms_m"], .0001)

    def test_ideal_camera_exports_identity(self):
        transform = cal.handeye_transform(cal.CV_TO_PHYSICS, [130, 4, 94])
        np.testing.assert_array_equal(np.array(transform["R"]).reshape(3, 3), np.eye(3))
        np.testing.assert_allclose(transform["t"], [.130, .004, .094])

    def test_bad_pose_and_degenerate_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "pose.txt"
            for text in ("0 0 0 0", "nan 0 0 0", "1 0 0", "2 0 0 0", "1 0 0 0 extra"):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    cal.quaternion_matrix(path)
            path.write_text("1 0 0 0")
            np.testing.assert_array_equal(cal.quaternion_matrix(path), np.eye(3))
        with self.assertRaises(ValueError):
            cal.require_rotation_excitation([np.eye(3)] * 10)
        with self.assertRaises(ValueError):
            cal.require_rotation_excitation([cv2.Rodrigues(np.array([0., 0., a]))[0]
                                             for a in np.linspace(0, 1, 10)])

    def test_inconsistent_pose_rejected(self):
        views, _, _ = self.handeye_views(False)
        for view in views:
            view["pose"] = cv2.Rodrigues(self.rng.uniform(-.7, .7, 3))[0]
        with self.assertRaises((ValueError, cv2.error)):
            cal.calibrate_handeye(self.size, views, self.settings,
                cal.camera_info(self.size, self.k, self.d, "test"))

    def test_input_validation_and_non_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            with self.assertRaises(ValueError):
                cal.load_views(folder, self.settings)
            cv2.imencode('.png', np.full((80, 100), 255, np.uint8))[1].tofile(folder / "1.png")
            cv2.imencode('.png', np.full((81, 100), 255, np.uint8))[1].tofile(folder / "3.png")
            with self.assertRaisesRegex(ValueError, "Mixed image"):
                cal.load_views(folder, self.settings)
            with self.assertRaises(OSError):
                cal.load_views(folder, self.settings, with_pose=True)
            out = folder / "out.yaml"
            original = {"serial": {"device_name": "sentinel"}, "camera": {"type": "hik"}}
            cal.write_result(out, original, {"rms": 0.1})
            self.assertEqual(cal.load_yaml(out), original)
            previous = out.read_bytes()
            with self.assertRaises(FileExistsError):
                cal.write_result(out, {}, {})
            self.assertEqual(out.read_bytes(), previous)

    def test_full_image_cli_and_config_preservation(self):
        # Rasterized board views exercise the actual circle detector, image loading,
        # calibration, CLI and complete YAML output, including gaps in numbering.
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            images = folder / "images"
            images.mkdir()
            for i, view in enumerate(self.intrinsic_views()):
                image = np.full((self.size[1], self.size[0]), 255, np.uint8)
                for x, y in view["points"].reshape(-1, 2):
                    cv2.circle(image, (round(float(x)), round(float(y))), 9, 0, -1, cv2.LINE_AA)
                cv2.imencode('.png', image)[1].tofile(images / f"{2*i+1}.png")
            source = ROOT / "config/sentry.yaml"
            original = source.read_bytes()
            output = folder / "result.yaml"
            diagnostics = io.StringIO()
            with contextlib.redirect_stderr(diagnostics), contextlib.redirect_stdout(io.StringIO()):
                result = cal.main(["camera", str(images), "--settings", str(ROOT / "config/calibration.yaml"),
                                   "--robot-config", str(source), "--output", str(output)])
            self.assertEqual(result, 0, diagnostics.getvalue())
            self.assertEqual(source.read_bytes(), original)
            config = cal.load_yaml(output)
            expected = cal.load_yaml(source)
            self.assertEqual(config["serial"], expected["serial"])
            self.assertEqual(config["tf"], expected["tf"])
            self.assertEqual(config["camera"]["hik_camera"], expected["camera"]["hik_camera"])
            self.assertEqual(config["camera"]["hik_camera"]["trigger_mode"], "Off")
            self.assertIs(config["serial"]["enable"], True)
            self.assertEqual(config["camera"]["camera_info"]["image_width"], self.size[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
