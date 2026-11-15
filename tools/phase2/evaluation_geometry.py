"""Independent pinhole ray/box reference for the saved Phase 2 worlds."""
import xml.etree.ElementTree as ET

import numpy as np


def quaternion_matrix(xyzw):
    q = np.asarray(xyzw, dtype=float)
    if not np.all(np.isfinite(q)) or abs(np.linalg.norm(q) - 1) > 1e-4:
        raise ValueError('invalid unit quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def read_boxes(path):
    boxes = []
    for model in ET.parse(path).getroot().find('world').findall('model'):
        if model.findtext('static') != 'true':
            raise ValueError('reference surfaces must be static')
        visual = model.find('link/visual')
        size = np.fromstring(visual.findtext('geometry/box/size'), sep=' ')
        pose = np.fromstring(model.findtext('pose'), sep=' ')
        if size.size != 3 or pose.size != 6 or np.any(size <= 0):
            raise ValueError('malformed reference box')
        boxes.append(dict(name=model.attrib['name'], position=pose[:3],
                          rotation=rpy_matrix(pose[3:]), size=size))
    if {b['name'] for b in boxes} != {
            'ground', 'textured_wall', 'occluding_edge', 'low_texture_patch'}:
        raise ValueError('unexpected validation world surfaces')
    return boxes


def pose_from_json(pose):
    p, q = pose['position'], pose['orientation']
    position = np.array([float(p.get(k, 0)) for k in 'xyz'])
    if not np.all(np.isfinite(position)):
        raise ValueError('nonfinite reference position')
    return position, quaternion_matrix([float(q.get(k, 0)) for k in 'xyzw'])


def pose_delta(a, b):
    position = float(np.linalg.norm(a[0] - b[0]))
    rotation = float(np.arccos(np.clip((np.trace(a[1].T @ b[1])-1)/2, -1, 1)))
    return position, rotation


def cast_boxes(origin, directions, boxes):
    """Ray parameter is optical Z, since input optical directions have z=1."""
    depth = np.full(directions.shape[:-1], np.inf)
    label = np.full(depth.shape, -1, dtype=np.int16)
    for i, box in enumerate(boxes):
        o = (origin - box['position']) @ box['rotation']
        d = directions @ box['rotation']
        parallel = np.abs(d) < 1e-12
        outside = np.any(parallel & (np.abs(o) > box['size']/2), axis=-1)
        a = np.divide(-box['size']/2-o, d, out=np.full_like(d, -np.inf), where=~parallel)
        b = np.divide(box['size']/2-o, d, out=np.full_like(d, np.inf), where=~parallel)
        near, far = np.min([a, b], axis=0).max(axis=-1), np.max([a, b], axis=0).min(axis=-1)
        hit = np.where(near > 0, near, far)
        visible = ~outside & (far >= np.maximum(near, 0)) & (hit > 0) & (hit < depth)
        depth[visible], label[visible] = hit[visible], i
    return depth, label


def erode(mask, radius):
    result = mask.copy()
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            shifted = np.zeros_like(mask)
            y0, y1 = max(0, dy), min(mask.shape[0], mask.shape[0]+dy)
            x0, x1 = max(0, dx), min(mask.shape[1], mask.shape[1]+dx)
            shifted[y0:y1, x0:x1] = mask[y0-dy:y1-dy, x0-dx:x1-dx]
            result &= shifted
    return result


def reference_raster(body_pose, calibration, boxes, criteria):
    width, height = calibration['image_size']
    v, u = np.indices((height, width))
    fx, fy, cx, cy = (calibration[k] for k in ('fx', 'fy', 'cx', 'cy'))
    rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u)), axis=-1)
    body_p, body_r = body_pose
    left, right = (np.array(calibration[k]) for k in ('left_pose_body', 'right_pose_body'))
    rotation = body_r @ rpy_matrix(left[3:])
    origin = body_p + body_r @ left[:3]
    depth, labels = cast_boxes(origin, rays @ rotation.T, boxes)
    patch_id = next(i for i, b in enumerate(boxes) if b['name'] == 'low_texture_patch')
    wall_id = next(i for i, b in enumerate(boxes) if b['name'] == 'textured_wall')
    masks = {'low_texture': erode(labels == patch_id, criteria['low_texture_interior_erosion_px'])}
    x, y, w, h = criteria['textured_roi_xywh']
    masks['textured_roi'] = np.zeros_like(labels, dtype=bool)
    masks['textured_roi'][y:y+h, x:x+w] = True
    if not np.all(labels[masks['textured_roi']] == wall_id):
        raise ValueError('predeclared textured ROI does not exclusively view the wall at this pose')
    discontinuity = np.zeros_like(labels, dtype=bool)
    discontinuity[:, 1:] |= labels[:, 1:] != labels[:, :-1]
    discontinuity[1:, :] |= labels[1:, :] != labels[:-1, :]
    masks['edges'] = ~erode(~discontinuity, criteria['edge_band_radius_px'])
    # Right-camera visibility of each left-camera surface point. This excludes
    # pixels outside the right image; those are reported in a separate mask.
    points = rays * np.where(np.isfinite(depth), depth, 1)[..., None]
    right_rays = points.copy()
    right_rays[..., 0] -= calibration['baseline_m']
    right_u = fx * right_rays[..., 0] / right_rays[..., 2] + cx
    right_rays /= right_rays[..., 2, None]
    right_depth, _ = cast_boxes(body_p + body_r @ right[:3], right_rays @ rotation.T, boxes)
    in_right = (right_u >= 0) & (right_u < width)
    masks['outside_right_fov'] = ~in_right & np.isfinite(depth)
    masks['right_occluded'] = in_right & np.isfinite(depth) & (right_depth < depth-.005)
    return depth, labels, masks, origin, rotation


def region_metrics(observed, expected, mask):
    selected = observed[:, mask] if observed.ndim == 3 else observed[mask]
    valid = np.isfinite(selected)
    truth = np.broadcast_to(expected[mask], selected.shape)
    errors = np.abs(selected[valid] - truth[valid])
    return dict(mask_pixels=int(mask.sum()), valid_fraction=float(valid.mean()) if valid.size else None,
                median_absolute_error_m=float(np.median(errors)) if errors.size else None,
                p95_absolute_error_m=float(np.percentile(errors, 95)) if errors.size else None)
