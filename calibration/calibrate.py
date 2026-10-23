#!/usr/bin/env python3
"""Tongji circle-grid calibration adapted to Awakening YAML (see LICENSE-TongjiSuperPower).

Source: sp_vision_25/calibration, bd9f5e798fa3c6dd3b483ae6627796afb41c608d.
Offline Python port: capture uses the existing C++ Awakening camera/serial drivers.
"""
import argparse
import copy
import math
from pathlib import Path
import re
import sys

import cv2
import numpy as np
import yaml


# src/tasks/base/common.hpp: camera CV (right/down/forward) -> physics (forward/left/up).
CV_TO_PHYSICS = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


class ConfigLoader(yaml.SafeLoader):
    """Preserve camera SDK enum strings such as Off/On (PyYAML's YAML 1.1 booleans)."""
    yaml_implicit_resolvers = {
        key: [(tag, regex) for tag, regex in values if tag != "tag:yaml.org,2002:bool"]
        for key, values in yaml.SafeLoader.yaml_implicit_resolvers.items()
    }


ConfigLoader.add_implicit_resolver("tag:yaml.org,2002:bool",
    re.compile(r"^(?:true|True|TRUE|false|False|FALSE)$"), list("tTfF"))


def load_yaml(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        result = yaml.load(stream, Loader=ConfigLoader)
    if not isinstance(result, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return result


def camera_node(document, key):
    node = document
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ValueError(f"Missing camera config: {key}")
        node = node[part]
    if not isinstance(node, dict):
        raise ValueError(f"Camera config must be a mapping: {key}")
    return node


def rotation(value, name="rotation"):
    value = np.asarray(value, dtype=np.float64).reshape(3, 3)
    if (not np.isfinite(value).all() or
            not np.allclose(value.T @ value, np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(value), 1., atol=1e-5)):
        raise ValueError(f"Invalid SO(3) matrix: {name}")
    return value


def quaternion_matrix(path):
    values = Path(path).read_text(encoding="utf-8-sig").split()
    if len(values) != 4:
        raise ValueError(f"Expected exactly w x y z: {path}")
    q = np.array([float(x) for x in values], dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or not 0.99 <= norm <= 1.01:
        raise ValueError(f"Invalid unit quaternion: {path}")
    w, x, y, z = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def board_settings(settings):
    cols, rows = int(settings["pattern_cols"]), int(settings["pattern_rows"])
    distance = float(settings["center_distance_mm"])
    if cols < 2 or rows < 2 or not math.isfinite(distance) or distance <= 0:
        raise ValueError("Invalid circle grid dimensions/distance")
    return cols, rows, distance


def board_points(settings, robotworld=False):
    cols, rows, distance = board_settings(settings)
    # Keep Tongji's board frame for each solver; all object coordinates are mm.
    if robotworld:
        points = [(0, (-j + .5*cols)*distance, (-i + .5*rows)*distance)
                  for i in range(rows) for j in range(cols)]
    else:
        points = [(j*distance, i*distance, 0) for i in range(rows) for j in range(cols)]
    return np.array(points, dtype=np.float32)


def load_views(folder, settings, with_pose=False, preview=False):
    folder = Path(folder)
    if not folder.is_dir():
        raise ValueError(f"Image directory does not exist: {folder}")
    paths = sorted((p for p in folder.iterdir() if p.suffix.lower() in
                    {".png", ".jpg", ".jpeg", ".bmp"}), key=lambda p: p.name)
    if not paths:
        raise ValueError("No calibration images")
    cols, rows, _ = board_settings(settings)
    mount = rotation(settings.get("R_gimbal2imubody", np.eye(3)))
    size, views, stems = None, [], set()
    for path in paths:
        if path.stem in stems:
            raise ValueError(f"Duplicate image stem (ambiguous pose): {path.stem}")
        stems.add(path.stem)
        # imread on Windows cannot reliably open non-ASCII paths.
        encoded = np.fromfile(path, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE) if encoded.size else None
        if image is None:
            raise ValueError(f"Unreadable image: {path}")
        this_size = (image.shape[1], image.shape[0])
        if size is not None and size != this_size:
            raise ValueError(f"Mixed image resolutions: {path}")
        size = this_size
        pose = None
        if with_pose:
            pose = mount.T @ quaternion_matrix(path.with_suffix(".txt")) @ mount
        ok, points = cv2.findCirclesGrid(image, (cols, rows), flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        print(f"{'success' if ok else 'skip'}: {path}", file=sys.stderr)
        if preview:
            drawing = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(drawing, (cols, rows), points, ok)
            cv2.imshow("Circle grid: any key next, q abort", cv2.resize(drawing, None, fx=.5, fy=.5))
            if cv2.waitKey(0) & 255 == ord('q'):
                raise ValueError("Calibration cancelled")
        if ok:
            views.append({"path": str(path), "points": points, "pose": pose})
    minimum = max(3, int(settings.get("min_views", 10)))
    if len(views) < minimum:
        raise ValueError(f"Need at least {minimum} detected views, got {len(views)}")
    return size, views


def matrix_node(value):
    value = np.asarray(value, dtype=np.float64)
    if value.ndim != 2 or not np.isfinite(value).all():
        raise ValueError("Invalid matrix for YAML output")
    return {"rows": value.shape[0], "cols": value.shape[1], "data": value.ravel().tolist()}


def camera_info(size, k, d, name):
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(d, dtype=np.float64).reshape(1, 5)
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError("Invalid camera intrinsics")
    return {"image_width": int(size[0]), "image_height": int(size[1]), "camera_name": name,
            "camera_matrix": matrix_node(k), "distortion_model": "plumb_bob",
            "distortion_coefficients": matrix_node(d),
            "rectification_matrix": matrix_node(np.eye(3)),
            # Monocular unrectified camera: P = [K | 0]; no invented stereo baseline.
            "projection_matrix": matrix_node(np.column_stack((k, np.zeros(3))))}


def read_intrinsics(info, size):
    if (int(info["image_width"]), int(info["image_height"])) != size:
        raise ValueError("Calibration image size differs from camera_info; recalibrate intrinsics")
    def read(name, shape):
        node = info[name]
        if (int(node["rows"]), int(node["cols"])) != shape:
            raise ValueError(f"Invalid dimensions: {name}")
        value = np.array(node["data"], dtype=np.float64).reshape(shape)
        if not np.isfinite(value).all():
            raise ValueError(f"Non-finite {name}")
        return value
    k, d = read("camera_matrix", (3, 3)), read("distortion_coefficients", (1, 5))
    if info.get("distortion_model") != "plumb_bob":
        raise ValueError("Only plumb_bob distortion is supported")
    camera_info(size, k, d, "validation")
    return k, d


def reprojection(objects, views, rvecs, tvecs, k, d):
    errors = []
    for view, rv, tv in zip(views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objects, rv, tv, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum(
            (view["points"].reshape(-1, 2) - projected.reshape(-1, 2))**2, axis=1)))))
    return float(np.sqrt(np.mean(np.square(errors)))), errors


def check_limit(value, settings, name, default):
    limit = float(settings.get(name, default))
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(value) or value > limit:
        raise ValueError(f"Calibration rejected: {name}: {value:.6g} > {limit:.6g}")


def calibrate_camera(size, views, settings, name):
    objects = board_points(settings)
    _, k, d, rvecs, tvecs = cv2.calibrateCamera(
        [objects for _ in views], [v["points"] for v in views], size, None, None,
        flags=cv2.CALIB_FIX_K3,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, np.finfo(float).eps))
    rms, errors = reprojection(objects, views, rvecs, tvecs, k, d)
    check_limit(rms, settings, "max_reprojection_rms_px", 2.)
    return camera_info(size, k, d, name), {"reprojection_rms_px": rms, "per_view_rms_px": errors}


def require_rotation_excitation(rotations):
    axes = np.array([cv2.Rodrigues(rotations[0].T @ r)[0].ravel() for r in rotations[1:]])
    singular = np.linalg.svd(axes, compute_uv=False)
    if len(singular) < 2 or singular[1] < math.radians(5):
        raise ValueError("Degenerate poses: rotate around at least two axes (yaw AND pitch)")


def handeye_transform(r_camera_to_gimbal, t_mm):
    r = rotation(r_camera_to_gimbal) @ CV_TO_PHYSICS.T
    t = np.asarray(t_mm, dtype=np.float64).reshape(3) / 1000.
    if not np.isfinite(t).all():
        raise ValueError("Non-finite hand-eye translation")
    return {"R": r.ravel().tolist(), "t": t.tolist()}


def calibrate_handeye(size, views, settings, info, robotworld=False):
    k, d = read_intrinsics(info, size)
    rg = [rotation(v["pose"]) for v in views]
    require_rotation_excitation(rg)
    objects = board_points(settings, robotworld)
    rvecs, tvecs = [], []
    for view in views:
        ok, rv, tv = cv2.solvePnP(objects, view["points"], k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok or not np.isfinite(rv).all() or not np.isfinite(tv).all():
            raise ValueError(f"PnP failed: {view['path']}")
        transformed = objects @ cv2.Rodrigues(rv)[0].T + tv.reshape(3)
        if np.any(transformed[:, 2] <= 0):
            raise ValueError(f"Board behind camera: {view['path']}")
        rvecs.append(rv)
        tvecs.append(tv)
    rms, errors = reprojection(objects, views, rvecs, tvecs, k, d)
    check_limit(rms, settings, "max_reprojection_rms_px", 2.)
    zeros = [np.zeros((3, 1)) for _ in views]
    if robotworld:
        _, _, r_gimbal_to_camera, t_gimbal_to_camera = cv2.calibrateRobotWorldHandEye(
            rvecs, tvecs, [r.T for r in rg], zeros, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
        r = r_gimbal_to_camera.T
        t = -r @ t_gimbal_to_camera
    else:
        r, t = cv2.calibrateHandEye(rg, zeros, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_TSAI)
    rotation(r)
    # The board and gimbal origin must stay fixed in world during acquisition.
    world_r = [g @ r @ cv2.Rodrigues(rv)[0] for g, rv in zip(rg, rvecs)]
    world_t = np.array([(g @ (r @ tv + t)).ravel()/1000. for g, tv in zip(rg, tvecs)])
    u, _, vt = np.linalg.svd(np.sum(world_r, axis=0))
    mean_r = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    angle_rms = float(np.sqrt(np.mean([np.linalg.norm(cv2.Rodrigues(mean_r.T @ a)[0])**2
                                      for a in world_r])) * 180. / np.pi)
    translation_rms = float(np.sqrt(np.mean(np.sum((world_t-world_t.mean(axis=0))**2, axis=1))))
    check_limit(angle_rms, settings, "max_handeye_rotation_rms_deg", 3.)
    check_limit(translation_rms, settings, "max_handeye_translation_rms_m", .02)
    return handeye_transform(r, t), {"reprojection_rms_px": rms, "per_view_rms_px": errors,
        "board_rotation_rms_deg": angle_rms, "board_translation_rms_m": translation_rms,
        "board_position_world_m": world_t.mean(axis=0).tolist()}


def write_result(path, document, report):
    # Exclusive creation: never overwrite the robot config or an earlier calibration.
    path = Path(path)
    text = "# Calibration result; inspect quality and hardware before deployment.\n"
    text += "# " + yaml.safe_dump(report, allow_unicode=True, sort_keys=False).replace("\n", "\n# ")
    text += "\n" + yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["camera", "handeye", "robotworld-handeye"])
    parser.add_argument("input_folder", type=Path)
    parser.add_argument("--settings", type=Path, default=Path("config/calibration.yaml"))
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--camera-key", default="camera", help="e.g. camera or armor_omni.camera0")
    parser.add_argument("--output", type=Path, required=True, help="new complete robot YAML; refuses overwrite")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"Output already exists: {args.output}")
        settings = load_yaml(args.settings)
        document = copy.deepcopy(load_yaml(args.robot_config))
        node = camera_node(document, args.camera_key)
        size, views = load_views(args.input_folder, settings, args.mode != "camera", args.preview)
        if args.mode == "camera":
            node["camera_info"], report = calibrate_camera(size, views, settings,
                node.get("camera_info", {}).get("camera_name", args.camera_key))
        else:
            if args.camera_key != "camera":
                raise ValueError("Hand-eye export targets the main camera tf.camera_in_gimbal only")
            transform, report = calibrate_handeye(size, views, settings, node["camera_info"],
                                                  args.mode == "robotworld-handeye")
            document.setdefault("tf", {})["camera_in_gimbal"] = transform
        report.update(mode=args.mode, accepted_views=len(views), images=[v["path"] for v in views])
        write_result(args.output, document, report)
        print(yaml.safe_dump(report, sort_keys=False), file=sys.stderr)
        print(f"Written: {args.output}")
        return 0
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, cv2.error, np.linalg.LinAlgError) as error:
        print(f"Calibration failed: {error}", file=sys.stderr)
        return 1
    finally:
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
