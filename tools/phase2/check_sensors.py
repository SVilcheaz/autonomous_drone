#!/usr/bin/env python3
"""Short sensor-contract check, not the Jetson stereo-depth acceptance test."""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import struct
import time
import zlib

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def write_image(path, msg):
    if msg.encoding != "rgb8":
        raise ValueError(f"Expected rgb8, got {msg.encoding}")
    width = msg.width * 3
    raw = b"".join(b"\0" + bytes(msg.data[row * msg.step:row * msg.step + width]) for row in range(msg.height))
    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", msg.width, msg.height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results/local_streams.json")
    parser.add_argument("--scope", default="Jetson sensor contract only; no depth/flight acceptance")
    parser.add_argument("--max-age-s", type=float, default=0.5)
    parser.add_argument("--max-receipt-gap-s", type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("phase2_local_contract_check")
    samples, last, frames = defaultdict(list), {}, defaultdict(set)
    arrivals = defaultdict(list)
    images = {"left": {}, "right": {}}
    transforms = {}
    subscriptions = []
    def callback(key, msg):
        stamp = stamp_ns(msg.clock if key == "clock" else msg.header.stamp)
        samples[key].append(stamp)
        arrivals[key].append(time.monotonic())
        last[key] = msg
        if key != "clock":
            frames[key].add(msg.header.frame_id)
        if key in images:
            images[key][stamp] = msg
            if len(images[key]) > 5:
                images[key].pop(next(iter(images[key])))
    for key, topic, kind in (
        ("left", "/stereo/left/image_rect", Image), ("right", "/stereo/right/image_rect", Image),
        ("left_info", "/stereo/left/camera_info", CameraInfo), ("right_info", "/stereo/right/camera_info", CameraInfo),
        ("imu", "/imu/data", Imu), ("clock", "/clock", Clock)):
        subscriptions.append(node.create_subscription(kind, topic, lambda msg, key=key: callback(key, msg), qos_profile_sensor_data))
    def tf_callback(msg):
        for transform in msg.transforms:
            t, q = transform.transform.translation, transform.transform.rotation
            transforms[transform.child_frame_id] = {"parent": transform.header.frame_id,
                "translation": [t.x, t.y, t.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]}
    static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", tf_callback, static_qos))
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    duration = time.monotonic() - start
    report = {"scope": args.scope, "wall_seconds": duration,
              "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")), "streams": {}, "static_transforms": transforms,
              "clock_publisher_count": len(node.get_publishers_info_by_topic("/clock"))}
    errors = []
    if report["clock_publisher_count"] != 1:
        errors.append("expected exactly one ROS /clock publisher")
    for key in ("left", "right", "left_info", "right_info", "imu", "clock"):
        stamps = samples[key]
        advancing = len(stamps) > 1 and stamps[-1] > stamps[0]
        hz = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]) if advancing else 0
        report["streams"][key] = {"count": len(stamps), "advancing": advancing, "hz_sim_time": hz,
            "hz_wall_time": len(stamps) / duration, "frame_ids": sorted(frames[key]),
            "first_stamp_ns": stamps[0] if stamps else None, "last_stamp_ns": stamps[-1] if stamps else None}
        received = arrivals[key]
        receipt_age = time.monotonic() - received[-1] if received else None
        source_age = (samples["clock"][-1] - stamps[-1]) * 1e-9 if samples["clock"] and stamps else None
        report["streams"][key].update({
            "hz_receipt_after_discovery": (len(received) - 1) / (received[-1] - received[0]) if len(received) > 1 else 0,
            "last_receipt_age_wall_s": receipt_age,
            "last_capture_age_sim_s": source_age,
            "nonadvancing_stamp_count": sum(b <= a for a, b in zip(stamps, stamps[1:])),
            "max_receipt_gap_wall_s": max((b - a for a, b in zip(received, received[1:])), default=0),
        })
        if not advancing:
            errors.append(f"{key}: no advancing messages")
        if receipt_age is None or receipt_age > args.max_receipt_gap_s:
            errors.append(f"{key}: delivery stopped or never started")
        if report["streams"][key]["max_receipt_gap_wall_s"] > args.max_receipt_gap_s:
            errors.append(f"{key}: interrupted delivery during the sample")
        if source_age is None or source_age > args.max_age_s or source_age < -0.1:
            errors.append(f"{key}: capture stamp is stale or inconsistent with /clock")
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            errors.append(f"{key}: repeated or backward acquisition stamps")
    for side in ("left", "right"):
        if side not in last or f"{side}_info" not in last:
            continue
        image, info = last[side], last[f"{side}_info"]
        expected_frame = f"stereo_{side}_optical_frame"
        camera = {"width": info.width, "height": info.height, "encoding": image.encoding,
                  "frame_id": info.header.frame_id, "k": list(info.k), "p": list(info.p), "r": list(info.r), "d": list(info.d)}
        report[f"{side}_camera"] = camera
        if (image.width, image.height, info.width, info.height, image.encoding) != (640, 480, 640, 480, "rgb8"):
            errors.append(f"{side}: dimensions/encoding mismatch")
        if frames[side] != {expected_frame} or frames[f"{side}_info"] != {expected_frame}:
            errors.append(f"{side}: unexpected optical frame")
        if not math.isclose(info.k[0], 400, abs_tol=1e-5) or not math.isclose(info.k[4], 400, abs_tol=1e-5):
            errors.append(f"{side}: wrong focal length")
        expected_tx = 0 if side == "left" else -48
        if not math.isclose(info.p[3], expected_tx, abs_tol=1e-5):
            errors.append(f"{side}: projection Tx {info.p[3]} != {expected_tx}")
        expected_k = [400, 0, 320, 0, 400, 240, 0, 0, 1]
        expected_p = [400, 0, 320, expected_tx, 0, 400, 240, 0, 0, 0, 1, 0]
        identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        if (not all(math.isclose(a, b, abs_tol=1e-5) for a, b in zip(info.k, expected_k))
                or not all(math.isclose(a, b, abs_tol=1e-5) for a, b in zip(info.p, expected_p))
                or not all(math.isclose(a, b, abs_tol=1e-5) for a, b in zip(info.r, identity))
                or any(abs(x) > 1e-8 for x in info.d)):
            errors.append(f"{side}: unexpected rectified calibration")
        if image.step < image.width * 3 or len(image.data) != image.step * image.height:
            errors.append(f"{side}: malformed RGB image layout")
        tf = transforms.get(expected_frame)
        if not tf or tf["parent"] != "base_link":
            errors.append(f"{side}: missing static mounting transform")
        else:
            expected_translation = [0.25, 0.06 if side == "left" else -0.06, 0.14]
            expected_quaternion = [-0.5, 0.5, -0.5, 0.5]
            dot = sum(a * b for a, b in zip(tf["quaternion_xyzw"], expected_quaternion))
            if (not all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(tf["translation"], expected_translation))
                    or not math.isclose(abs(dot), 1, abs_tol=1e-6)):
                errors.append(f"{side}: mounting transform differs from saved FLU/optical calibration")
    left_stamps, right_stamps = set(samples["left"]), set(samples["right"])
    denominator = min(len(left_stamps), len(right_stamps))
    matched = len(left_stamps & right_stamps)
    info_left, info_right = set(samples["left_info"]), set(samples["right_info"])
    info_denominator = min(len(info_left), len(info_right))
    info_matched = len(info_left & info_right)
    report["stereo_sync"] = {"exact_stamp_pairs": matched, "fraction_of_smaller_stream": matched / denominator if denominator else 0,
        "camera_info_exact_stamp_pairs": info_matched, "camera_info_match_fraction": info_matched / info_denominator if info_denominator else 0,
        "four_way_exact_stamp_sets": len(left_stamps & right_stamps & info_left & info_right),
        "note": "Images use best-effort QoS; asymmetric delivery losses reduce available pairs without changing capture synchronization."}
    if (matched < 2 or not denominator or matched / denominator < 0.8
            or not info_denominator or info_matched / info_denominator < 0.98
            or len(left_stamps & right_stamps & info_left & info_right) < 2):
        errors.append("stereo acquisition timestamps do not match")
    if "imu" in last:
        imu = last["imu"]
        report["imu_sample"] = {"acceleration_m_s2": [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z],
                                "angular_velocity_rad_s": [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]}
        if frames["imu"] != {"base_link"}:
            errors.append("unexpected IMU frame")
    common = images["left"].keys() & images["right"].keys()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if common:
        selected = max(common)
        report["snapshot_stamp_ns"] = selected
        for side in ("left", "right"):
            write_image(args.output.parent / f"{args.output.stem}_{side}.png", images[side][selected])
    report["errors"] = errors
    report["passed"] = not errors
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"report": str(args.output), "passed": report["passed"], "errors": errors,
        "received_sim_hz": {key: round(value["hz_sim_time"], 2) for key, value in report["streams"].items()},
        "stereo_sync": report["stereo_sync"]}))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(bool(errors))


if __name__ == "__main__":
    main()
