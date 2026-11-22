#!/usr/bin/env python3
"""Generate a separate sensor model and validation worlds from pinned PX4 assets."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import struct
import subprocess
import xml.etree.ElementTree as ET
import zlib

PIN = "b12d17f63241e7c875a0394e7b52064ca3a34946"
ROOT = Path(__file__).resolve().parent
MOUNT_X, MOUNT_Z, BASELINE, FOCAL = 0.25, 0.14, 0.12, 400.0


def text(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def save_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def png(path):
    """Deterministic nonrepeating multiscale texture; no external image library."""
    size = 1024
    rng = random.Random(20261115)
    cells = [[rng.randrange(35, 220) for _ in range(128)] for _ in range(128)]
    coarse = [[rng.randrange(-30, 31) for _ in range(16)] for _ in range(16)]
    data = bytearray()
    for y in range(size):
        data.append(0)
        for x in range(size):
            value = max(5, min(250, cells[y // 8][x // 8] + coarse[y // 64][x // 64]))
            data.extend((value, value, value))
    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(data, 9)) + chunk(b"IEND", b""))


def material(visual, textured=False, color="0.7 0.7 0.7 1"):
    mat = ET.SubElement(visual, "material")
    text(mat, "ambient", color)
    text(mat, "diffuse", color)
    text(mat, "specular", "0 0 0 1")
    if textured:
        metal = ET.SubElement(ET.SubElement(mat, "pbr"), "metal")
        text(metal, "albedo_map", "model://phase2_target/materials/textures/nonrepeating.png")
        text(metal, "metalness", 0)
        text(metal, "roughness", 1)


def box(world, name, pose, size, textured=False, color="0.7 0.7 0.7 1"):
    model = ET.SubElement(world, "model", name=name)
    text(model, "static", "true")
    text(model, "pose", pose)
    link = ET.SubElement(model, "link", name="link")
    for tag in ("visual", "collision"):
        item = ET.SubElement(link, tag, name=tag)
        text(ET.SubElement(ET.SubElement(item, "geometry"), "box"), "size", size)
        if tag == "visual":
            material(item, textured, color)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4", type=Path, default=Path.home() / "PX4-Autopilot")
    args = parser.parse_args()
    px4 = args.px4.resolve()
    revision = subprocess.check_output(["git", "-C", str(px4), "rev-parse", "HEAD"], text=True).strip()
    if revision != PIN:
        raise SystemExit(f"Unexpected PX4 revision {revision}; expected {PIN}. No upstream files changed.")
    vendor = px4 / "Tools/simulation/gz"
    source_paths = {"base": vendor / "models/x500_base/model.sdf",
                    "motors": vendor / "models/x500/model.sdf",
                    "existing_default_world": vendor / "worlds/default.sdf"}
    sources = {key: hashlib.sha256(path.read_bytes()).hexdigest() for key, path in source_paths.items()}
    base = ET.parse(source_paths["base"]).getroot().find("model")
    motors = ET.parse(source_paths["motors"]).getroot().find("model")
    includes = motors.findall("include")
    if len(includes) != 1 or includes[0].findtext("uri") != "model://x500_base":
        raise SystemExit("Unexpected x500 composition; inspect it before generating an overlay.")
    model = copy.deepcopy(base)
    model.set("name", "x500_phase2")
    for child in motors:
        if child.tag != "include":
            model.append(copy.deepcopy(child))
    body = model.find("link[@name='base_link']")
    imu = body.find("sensor[@type='imu']")
    if imu is None:
        raise SystemExit("Expected existing x500 IMU is missing.")
    for side, y, tx in (("left", BASELINE / 2, 0), ("right", -BASELINE / 2, -FOCAL * BASELINE)):
        sensor = ET.SubElement(body, "sensor", name=f"phase2_{side}_camera", type="camera")
        text(sensor, "pose", f"{MOUNT_X} {y} {MOUNT_Z} 0 0 0")
        text(sensor, "gz_frame_id", f"stereo_{side}_optical_frame")
        text(sensor, "topic", f"/phase2/stereo/{side}/image")
        text(sensor, "always_on", "true")
        text(sensor, "update_rate", 20)
        camera = ET.SubElement(sensor, "camera")
        text(camera, "camera_info_topic", f"/phase2/stereo/{side}/camera_info")
        text(camera, "optical_frame_id", f"stereo_{side}_optical_frame")
        text(camera, "horizontal_fov", 2 * math.atan(640 / (2 * FOCAL)))
        image = ET.SubElement(camera, "image")
        for key, value in {"width": 640, "height": 480, "format": "R8G8B8", "anti_aliasing": 0}.items():
            text(image, key, value)
        clip = ET.SubElement(camera, "clip")
        text(clip, "near", 0.1)
        text(clip, "far", 30)
        lens = ET.SubElement(camera, "lens")
        intrinsics = ET.SubElement(lens, "intrinsics")
        for key, value in {"fx": FOCAL, "fy": FOCAL, "cx": 320, "cy": 240, "s": 0}.items():
            text(intrinsics, key, value)
        projection = ET.SubElement(lens, "projection")
        for key, value in {"p_fx": FOCAL, "p_fy": FOCAL, "p_cx": 320, "p_cy": 240, "tx": tx, "ty": 0}.items():
            text(projection, key, value)
    sdf = ET.Element("sdf", version="1.10")
    sdf.append(model)
    save_xml(sdf, ROOT / "models/x500_phase2/model.sdf")
    config = ET.Element("model")
    for key, value in {"name": "x500_phase2", "version": "1.0", "description": "Pinned x500 physics/motors with stereo sensors added to base_link."}.items():
        text(config, key, value)
    text(config, "sdf", "model.sdf").set("version", "1.10")
    save_xml(config, ROOT / "models/x500_phase2/model.config")
    target_config = ET.Element("model")
    text(target_config, "name", "phase2_target")
    text(target_config, "version", "1.0")
    text(target_config, "sdf", "model.sdf").set("version", "1.10")
    save_xml(target_config, ROOT / "models/phase2_target/model.config")
    target_sdf = ET.fromstring('<sdf version="1.10"><model name="phase2_target"><static>true</static><link name="link"/></model></sdf>')
    save_xml(target_sdf, ROOT / "models/phase2_target/model.sdf")
    png(ROOT / "models/phase2_target/materials/textures/nonrepeating.png")
    for distance in (1, 2, 4):
        world_sdf = ET.Element("sdf", version="1.10")
        world = ET.SubElement(world_sdf, "world", name="phase2_validation")
        physics = ET.SubElement(world, "physics", name="phase2", type="ignored")
        text(physics, "max_step_size", 0.002)
        text(physics, "real_time_factor", 1)
        text(world, "gravity", "0 0 -9.8")
        text(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
        default_world = ET.parse(source_paths["existing_default_world"]).getroot().find("world")
        for tag in ("spherical_coordinates", "atmosphere"):
            inherited = default_world.find(tag)
            if inherited is not None:
                world.append(copy.deepcopy(inherited))
        for plugin in ("Physics", "UserCommands", "SceneBroadcaster", "Imu", "AirPressure", "NavSat", "Magnetometer", "Sensors"):
            slug = {"UserCommands": "user-commands", "SceneBroadcaster": "scene-broadcaster", "AirPressure": "air-pressure"}.get(plugin, plugin.lower())
            p = ET.SubElement(world, "plugin", filename=f"gz-sim-{slug}-system", name=f"gz::sim::systems::{plugin}")
            if plugin == "Sensors":
                text(p, "render_engine", "ogre2")
        scene = ET.SubElement(world, "scene")
        text(scene, "ambient", "0.6 0.6 0.6 1")
        text(scene, "background", "0.65 0.7 0.8 1")
        text(scene, "shadows", "false")
        light = ET.SubElement(world, "light", name="sun", type="directional")
        text(light, "pose", "0 0 10 0 0 0")
        text(light, "direction", "-0.5 0.2 -0.8")
        text(light, "diffuse", "0.8 0.8 0.8 1")
        text(light, "specular", "0 0 0 1")
        box(world, "ground", "0 0 -0.05 0 0 0", "30 30 0.1", textured=True)
        # Front face, not box centre, is at mount_x + distance.
        box(world, "textured_wall", f"{MOUNT_X + distance + 0.05} 0 1.5 0 0 0", "0.1 8 3", textured=True, color="1 1 1 1")
        box(world, "occluding_edge", f"{MOUNT_X + distance - 0.35} 0.45 0.6 0 0 0", "0.1 0.12 1.2", textured=True)
        box(world, "low_texture_patch", f"{MOUNT_X + distance - 0.005} -0.4 0.6 0 0 0", "0.01 0.45 0.45")
        include = ET.SubElement(world, "include")
        text(include, "uri", "model://x500_phase2")
        text(include, "name", "x500_0")
        text(include, "pose", "0 0 0.24 0 0 0")
        save_xml(world_sdf, ROOT / f"worlds/phase2_validation_{distance}m.sdf")
    calibration = {
        "px4_revision": revision,
        "gazebo_models_revision": subprocess.check_output(["git", "-C", str(vendor), "rev-parse", "HEAD"], text=True).strip(),
        "source_sha256": sources,
        "model": "x500_0", "world": "phase2_validation", "image_size": [640, 480], "image_rate_hz": 20,
        "fx": FOCAL, "fy": FOCAL, "cx": 320, "cy": 240, "baseline_m": BASELINE,
        "distortion_k1_k2_p1_p2_k3": [0, 0, 0, 0, 0], "image_noise": "none (initial ideal pinhole validation rig)",
        "left_projection_tx": 0, "right_projection_tx": -FOCAL * BASELINE,
        "body_frame": "base_link", "body_axes": "FLU", "optical_axes": "x right, y down, z forward",
        "left_pose_body": [MOUNT_X, BASELINE / 2, MOUNT_Z, -math.pi / 2, 0, -math.pi / 2],
        "right_pose_body": [MOUNT_X, -BASELINE / 2, MOUNT_Z, -math.pi / 2, 0, -math.pi / 2],
        "imu_frame": "base_link", "imu_rate_hz": float(imu.findtext("update_rate")),
        "imu_noise_per_sample": {child.tag: {axis.tag: float(axis.findtext("noise/stddev")) for axis in child} for child in imu.find("imu")},
        "imu_note": "Original colocated FLU IMU reused unchanged; specific force includes gravity response. Rates are in simulation time.",
        "physics_max_step_s": 0.002, "texture_seed": 20261115,
        "wall_face_x": {str(d): MOUNT_X + d for d in (1, 2, 4)},
        "note": "Generated configuration, not a claim of runtime validation. No PX4/vendor files modified; no flight commanded."
    }
    (ROOT / "calibration.json").write_text(json.dumps(calibration, indent=2) + "\n")
    print(json.dumps({"assets": str(ROOT), "px4_revision": revision, "reused_imu_hz": calibration["imu_rate_hz"], "baseline_m": BASELINE}))


if __name__ == "__main__":
    main()
