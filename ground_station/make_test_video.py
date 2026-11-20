#!/usr/bin/env python3
"""Make a synthetic 'fly-past' video (pole, rusty bracket, a tree, sky) so you can
test the ground station without the drone:  python make_test_video.py test.mp4"""
import sys
import cv2
import numpy as np

out = sys.argv[1] if len(sys.argv) > 1 else "test_flight.mp4"
W, H, N = 960, 540, 240
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), 20, (W, H))
rng = np.random.default_rng(1)
for i in range(N):
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (215, 190, 160)                                   # sky
    img[int(H * .8):] = (90, 110, 120)                         # ground
    off = int(W * 1.2 - i * (W * 1.6 / N))                     # scene scrolls left
    # pole + cross-arm + conductors
    cv2.rectangle(img, (off, 120), (off + 26, H), (95, 95, 100), -1)
    cv2.rectangle(img, (off - 160, 130), (off + 186, 148), (110, 110, 115), -1)
    for dx in (-140, 13, 166):
        pts = np.array([[x, 150 + int(0.00012 * (x - off - dx) ** 2)] for x in range(0, W, 8)], np.int32)
        cv2.polylines(img, [pts], False, (40, 40, 40), 2)
    # rusty bracket (orange-brown blotches) on the cross-arm
    for _ in range(30):
        cx = off - 120 + int(rng.integers(0, 70)); cy = 136 + int(rng.integers(-10, 25))
        col = (int(rng.integers(20, 50)), int(rng.integers(70, 110)), int(rng.integers(150, 200)))
        cv2.circle(img, (cx, cy), int(rng.integers(4, 10)), col, -1)
    # tree growing into the line
    t = off + 420
    cv2.rectangle(img, (t - 8, 300), (t + 8, H), (40, 60, 90), -1)
    for _ in range(40):
        cv2.circle(img, (t + int(rng.integers(-110, 110)), 230 + int(rng.integers(-80, 80))),
                   int(rng.integers(18, 40)), (40, int(rng.integers(130, 170)), 50), -1)
    noise = rng.integers(-6, 6, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    vw.write(img)
vw.release()
print("wrote", out)
