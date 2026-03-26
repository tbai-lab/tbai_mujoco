#!/usr/bin/env python3
"""Repeatedly stand up and sit down — using tbai_sdk over Zenoh.

Usage: python stand_go2.py [--store_images] [--store_depth] [--hold S] [--ramp S] [--save_every N]
"""

import argparse
import math
import os
import struct
import sys
from collections import deque

from tbai_sdk import Publisher, PollingSubscriber, Rate
from tbai_sdk.messages import (
    ImgFrame,
    LowState,
    MotorCommand,
    MotorCommands,
    PointCloud2,
)

try:
    from PIL import Image
except ImportError:
    Image = None

NUM_MOTORS = 12
DT = 0.002  # 500 Hz
RAMP_TAU = 1.2  # tanh time constant for transitions

# Actuator order: FR(hip,thigh,calf), FL, RR, RL
STAND_UP = [
     0.00572,  0.60881, -1.21763,
    -0.00572,  0.60881, -1.21763,
     0.00572,  0.60881, -1.21763,
    -0.00572,  0.60881, -1.21763,
]

STAND_DOWN = [
     0.04735,  1.22187, -2.44375,
    -0.04735,  1.22187, -2.44375,
     0.04735,  1.22187, -2.44375,
    -0.04735,  1.22187, -2.44375,
]


def depth_to_rgb(norm: float) -> tuple[int, int, int]:
    """Turbo-ish colormap: maps 0..1 -> RGB."""
    norm = max(0.0, min(1.0, norm))
    if norm < 0.25:
        t = norm / 0.25
        return 0, int(t * 255), 255
    elif norm < 0.5:
        t = (norm - 0.25) / 0.25
        return 0, 255, int((1.0 - t) * 255)
    elif norm < 0.75:
        t = (norm - 0.5) / 0.25
        return int(t * 255), 255, 0
    else:
        t = (norm - 0.75) / 0.25
        return 255, int((1.0 - t) * 255), 0


def pointcloud_to_depth_image(
    pc: PointCloud2,
    img_w: int,
    img_h: int,
    fov_y_deg: float,
    max_depth: float,
) -> bytes | None:
    """Reconstruct a depth heatmap from a PointCloud2 message. Returns PNG bytes or None."""
    if Image is None:
        return None
    if not pc.data or pc.point_step < 12:
        return None

    fovy_rad = fov_y_deg * math.pi / 180.0
    fy = img_h / (2.0 * math.tan(fovy_rad / 2.0))
    fx = fy
    cx = img_w / 2.0
    cy = img_h / 2.0

    # Depth buffer
    depth_map = [max_depth] * (img_w * img_h)

    num_points = pc.width * pc.height
    data = bytes(pc.data)
    for i in range(num_points):
        offset = i * pc.point_step
        x, y, z = struct.unpack_from("<fff", data, offset)
        if z <= 0.0:
            continue
        u = int(x * fx / z + cx)
        v = int(y * fy / z + cy)
        if 0 <= u < img_w and 0 <= v < img_h:
            idx = v * img_w + u
            if z < depth_map[idx]:
                depth_map[idx] = z

    # BFS nearest-neighbor fill
    visited = [False] * (img_w * img_h)
    queue: deque[int] = deque()
    for i in range(img_w * img_h):
        if depth_map[i] < max_depth:
            visited[i] = True
            queue.append(i)

    dx = (-1, 1, 0, 0)
    dy = (0, 0, -1, 1)
    while queue:
        idx = queue.popleft()
        py, px = divmod(idx, img_w)
        for d in range(4):
            nx, ny = px + dx[d], py + dy[d]
            if 0 <= nx < img_w and 0 <= ny < img_h:
                nidx = ny * img_w + nx
                if not visited[nidx]:
                    visited[nidx] = True
                    depth_map[nidx] = depth_map[idx]
                    queue.append(nidx)

    # Render heatmap
    rgb = bytearray(img_w * img_h * 3)
    for i in range(img_w * img_h):
        r, g, b = depth_to_rgb(depth_map[i] / max_depth)
        rgb[i * 3] = r
        rgb[i * 3 + 1] = g
        rgb[i * 3 + 2] = b

    img = Image.frombytes("RGB", (img_w, img_h), bytes(rgb))
    import io
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def main():
    parser = argparse.ArgumentParser(description="Stand Go2 — repeatedly stand up and sit down")
    parser.add_argument("--hold", type=float, default=2.0, help="Hold duration in seconds (default: 2.0)")
    parser.add_argument("--ramp", type=float, default=3.0 * RAMP_TAU, help="Ramp duration in seconds (default: 3.6)")
    parser.add_argument("--store_images", action="store_true", help="Save RGB images to images/")
    parser.add_argument("--store_depth", action="store_true", help="Save depth heatmaps to depth/")
    parser.add_argument("--save_every", type=int, default=10, help="Save every Nth frame (default: 10)")
    args = parser.parse_args()

    hold_duration = args.hold
    ramp_duration = args.ramp
    save_every_n = max(1, args.save_every)
    store_images = args.store_images
    store_depth = args.store_depth

    tau = ramp_duration / 3.0
    half_cycle = ramp_duration + hold_duration
    full_cycle = 2.0 * half_cycle

    if store_images:
        os.makedirs("images", exist_ok=True)
    if store_depth:
        os.makedirs("depth", exist_ok=True)

    print(
        f"Stand Go2 — hold={hold_duration:.1f}s  ramp={ramp_duration:.1f}s  "
        f"cycle={full_cycle:.1f}s"
    )
    input("Press ENTER to start...")

    cmd_pub = Publisher(MotorCommands, "rt/lowcmd")
    state_sub = PollingSubscriber(LowState, "rt/lowstate")
    image_sub = PollingSubscriber(ImgFrame, "rt/camera/image")
    pc_sub = PollingSubscriber(PointCloud2, "rt/pointcloud")

    cmd = MotorCommands(commands=[MotorCommand() for _ in range(NUM_MOTORS)])

    t = 0.0
    last_saved_img = 0
    last_saved_pc = 0
    saved_images = 0
    saved_depths = 0
    rate = Rate(1.0 / DT)

    # Depth heatmap parameters (must match config.yaml depth camera settings)
    DEPTH_W = 320
    DEPTH_H = 240
    DEPTH_FOV_Y = 87.0
    DEPTH_MAX = 2.5

    while True:
        t += DT

        cycle_t = t % full_cycle
        standing_up = cycle_t < half_cycle

        if standing_up:
            src, dst = STAND_DOWN, STAND_UP
            phase = math.tanh(cycle_t / tau)
        else:
            src, dst = STAND_UP, STAND_DOWN
            phase = math.tanh((cycle_t - half_cycle) / tau)

        for i in range(NUM_MOTORS):
            cmd.commands[i].q = (1.0 - phase) * src[i] + phase * dst[i]
            cmd.commands[i].dq = 0.0
            cmd.commands[i].kp = 50.0
            cmd.commands[i].kd = 3.5
            cmd.commands[i].tau = 0.0

        cmd_pub.publish(cmd)

        if store_images:
            img_count = image_sub.message_count()
            if img_count > 0 and img_count >= last_saved_img + save_every_n:
                img = image_sub.get()
                if img is not None and Image is not None:
                    pil_img = Image.frombytes("RGB", (img.width, img.height), bytes(img.data))
                    filename = f"images/{saved_images:06d}.png"
                    pil_img.save(filename)
                    print(f"[{t:6.1f}s] Saved {filename}")
                    last_saved_img = img_count
                    saved_images += 1

        if store_depth:
            pc_count = pc_sub.message_count()
            if pc_count > 0 and pc_count >= last_saved_pc + save_every_n:
                pc = pc_sub.get()
                if pc is not None:
                    png_data = pointcloud_to_depth_image(pc, DEPTH_W, DEPTH_H, DEPTH_FOV_Y, DEPTH_MAX)
                    if png_data is not None:
                        filename = f"depth/{saved_depths:06d}.png"
                        with open(filename, "wb") as f:
                            f.write(png_data)
                        print(f"[{t:6.1f}s] Saved {filename} ({pc.width} pts → {len(png_data)} bytes)")
                        last_saved_pc = pc_count
                        saved_depths += 1

        # Print state once per second
        step = int(t / DT)
        if step % 500 == 0:
            st = state_sub.get()
            if st is not None:
                label = "UP  " if standing_up else "DOWN"
                q0 = st.motor_states[0].q
                q1 = st.motor_states[1].q
                q2 = st.motor_states[2].q
                print(f"[{t:6.1f}s] {label}  q0={q0:.3f} q1={q1:.3f} q2={q2:.3f}")

        rate.sleep()


if __name__ == "__main__":
    main()
