#!/usr/bin/env python3
"""Render saved scientific evaluation arrays; no live subscriptions or truth use."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def panel(data, low, high):
    values = np.nan_to_num(np.clip((data-low)/(high-low), 0, 1), nan=0, posinf=1)
    rgb = cv2.applyColorMap((values*255).astype(np.uint8), cv2.COLORMAP_TURBO)
    rgb[~np.isfinite(data)] = (80, 80, 80)
    return rgb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    report = json.loads(args.directory.joinpath('report.json').read_text())
    sample = report['samples'][0]
    arrays = np.load(args.directory/sample['path'])
    metadata = sample['image_metadata']['left']
    left = arrays['left_bytes'].reshape(metadata['height'], metadata['step'])
    if metadata['encoding'] != 'rgb8':
        raise ValueError('plot expects the saved RGB8 rig')
    left = left[:, :metadata['width']*3].reshape(metadata['height'], metadata['width'], 3)
    left = cv2.cvtColor(left, cv2.COLOR_RGB2BGR)
    observed, expected = arrays['depth_m'], arrays['expected_optical_z_m']
    error = np.abs(observed-expected)
    panels = [left, panel(expected, .75, 5), panel(observed, .75, 5), panel(error, 0, .4)]
    titles = ['Left image: fixed wall ROI (red)', 'Reference optical Z: 0.75 to 5 m',
              'Stereo optical Z: 0.75 to 5 m', 'Absolute error: 0 to 0.4 m']
    x, y, w, h = json.loads(args.directory.joinpath('criteria.json').read_text())['textured_roi_xywh']
    cv2.rectangle(panels[0], (x, y), (x+w-1, y+h-1), (0, 0, 255), 2)
    for name, color in [('low_texture', (0, 255, 0)), ('right_occluded', (255, 255, 0))]:
        contours, _ = cv2.findContours(arrays['mask_'+name].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(panels[0], contours, -1, color, 1)
    decorated = []
    for title, pixels in zip(titles, panels):
        canvas = cv2.copyMakeBorder(pixels, 32, 24, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        cv2.putText(canvas, title, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (15, 15, 15), 1)
        cv2.putText(canvas, 'Gray = unknown / no reference intersection', (8, canvas.shape[0]-7), cv2.FONT_HERSHEY_SIMPLEX, .45, (15, 15, 15), 1)
        decorated.append(canvas)
    result = np.vstack((np.hstack(decorated[:2]), np.hstack(decorated[2:])))
    destination = args.directory/'sample_overview.png'
    if not cv2.imwrite(str(destination), result):
        raise RuntimeError('failed to save plot')
    print(destination)


if __name__ == '__main__':
    main()
