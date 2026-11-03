#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl_bridge.py — 强化学习着陆策略（PPO）与飞控 SIL 的 UDP 联合仿真桥

策略与观测/动作映射与箭载机实机程序
`D:/电火箭制导控制/箭载机/Projects/Electric_Rocket/main.py` 保持一致：
  观测 obs = clip(env_state - TARGET_STATE, OBS_LOW, OBS_HIGH)
  env_state = [x(天), y(东), z(北), vx, vy, vz]
  动作 a∈[-1,1] → ux = AMIN + (a0+1)/2*(AMAX-AMIN), uy = a1*HORIZ_GAIN, uz = a2*HORIZ_GAIN
  下发加速度 = (ux - GRAVITY, uy, uz)

协议（与 SIL 一致）：
  收：0xAA + 18×float + 0x55（74 B，含 [5]pu [4]pe [3]pn [8]vu [7]ve [6]vn）
  发：0xAA + 3×float(U,E,N) + 0x55（14 B）

用法：
  py rl_bridge.py --model "D:/电火箭制导控制/箭载机/Projects/Electric_Rocket/ElectricRocket_policy_PPO" \
      --target 1.5 0 0 --duration 45 --out tmp/rl_joint.csv
"""
import argparse
import os
import socket
import struct
import time

import numpy as np

GRAVITY = 9.81
AMIN, AMAX = 7.81, 11.81
HORIZ_GAIN = 2.0
OBS_LOW = np.array([-20.0, -40.0, -40.0, -10.0, -10.0, -10.0], dtype=np.float32)
OBS_HIGH = np.array([20.0, 40.0, 40.0, 10.0, 10.0, 10.0], dtype=np.float32)


def transform_action(action):
    a = np.clip(action, -1.0, 1.0)
    ux = AMIN + (a[0] + 1.0) / 2.0 * (AMAX - AMIN)
    uy = a[1] * HORIZ_GAIN
    uz = a[2] * HORIZ_GAIN
    return float(ux), float(uy), float(uz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--rx-port", type=int, default=7002)
    ap.add_argument("--tx-port", type=int, default=7001)
    ap.add_argument("--target", type=float, nargs=3, default=[1.5, 0.0, 0.0],
                    metavar=("UP", "EAST", "NORTH"))
    ap.add_argument("--duration", type=float, default=45.0)
    ap.add_argument("--out", default="tmp/rl_joint.csv")
    a = ap.parse_args()

    from stable_baselines3 import PPO  # 延迟导入，避免无 sb3 环境无法跑其他仿真

    model = PPO.load(a.model)
    # 观测是 6 维（位置 3 + 速度 3），目标速度恒为 0
    target = np.concatenate([np.asarray(a.target, dtype=np.float32),
                             np.zeros(3, dtype=np.float32)])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", a.rx_port))
    sock.settimeout(0.5)
    dst = ("127.0.0.1", a.tx_port)

    rows, frames, cmds = [], 0, 0
    t0 = time.time()
    print(f"[rl] model={a.model} target={a.target} rx={a.rx_port} tx={a.tx_port}")

    while time.time() - t0 < a.duration:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        if len(data) < 74 or data[0] != 0xAA or data[73] != 0x55:
            continue
        f = struct.unpack("<18f", data[1:73])
        frames += 1

        env_state = np.array([f[5], f[4], f[3], f[8], f[7], f[6]], dtype=np.float32)
        obs = np.clip(env_state - target, OBS_LOW, OBS_HIGH)
        action, _ = model.predict(obs, deterministic=True)
        ux, uy, uz = transform_action(action)
        acc = (ux - GRAVITY, uy, uz)

        frame = b"\xaa" + struct.pack("<3f", *acc) + b"\x55"
        sock.sendto(frame, dst)
        cmds += 1
        rows.append([f[2], *env_state, *acc])

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as fh:
        fh.write("t_sil,up,e,n,vup,ve,vn,a_up,a_e,a_n\n")
        for r in rows:
            fh.write(",".join(f"{v:.6f}" for v in r) + "\n")
    print(f"[rl] frames={frames} cmds={cmds} log={a.out}")
    if rows:
        last = rows[-1]
        print(f"[rl] final pos=({last[1]:.2f},{last[2]:.2f},{last[3]:.2f}) target={a.target}")


if __name__ == "__main__":
    main()
