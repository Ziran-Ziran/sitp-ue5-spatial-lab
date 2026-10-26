"""Draw projected landmark positions on one RGB frame, to check the camera model."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np


def quat_to_matrix(quat):
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


session = Path(sys.argv[1])
frame_index = int(sys.argv[2]) if len(sys.argv) > 2 else 30
output = sys.argv[3] if len(sys.argv) > 3 else "tmp/projection.png"

landmarks = np.array(json.loads((session / "landmarks.json").read_text(encoding="utf-8"))["points"], dtype=float)
poses = {json.loads(l)["i"]: json.loads(l) for l in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()}
pose = poses[frame_index]

image = cv2.imread(str(session / "frames" / f"frame_{frame_index:06d}.png"))
rotation = quat_to_matrix(pose["quat"])
centre = np.array(pose["loc"], dtype=float)

drawn = 0
for point in landmarks:
    camera = rotation.T @ (point - centre)
    if camera[0] <= 10.0:
        continue
    u = pose["cx"] + pose["fx"] * camera[1] / camera[0]
    v = pose["cy"] - pose["fy"] * camera[2] / camera[0]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < pose["w"] and 0 <= vi < pose["h"]):
        continue
    cv2.circle(image, (ui, vi), 12, (0, 0, 255), 3)
    cv2.putText(image, f"{camera[0]/100:.1f}m", (ui + 14, vi), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    drawn += 1

cv2.imwrite(output, image)
print(f"drew {drawn} landmarks on frame {frame_index} -> {output}")
