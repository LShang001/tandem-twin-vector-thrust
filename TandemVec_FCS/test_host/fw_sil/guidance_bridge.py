#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
guidance_bridge.py — 制导算法（SOCP 轨迹规划 + 跟踪）与飞控 SIL 的 UDP 联合仿真桥

协议（与飞控 Serial5 私有帧一致）：
  状态上行（SIL → 本脚本）：0xAA + 18×float + 0x55（74 B）
      [0]auto [1]fire [2]t [3]pn [4]pe [5]pu(Up+) [6]vn [7]ve [8]vu
      [9]roll [10]pitch [11]yaw [12]gx [13]gy [14]gz [15]acc_u [16]acc_e [17]acc_n
  指令下行（本脚本 → SIL）：0xAA + 3×float(U,E,N) + 0x55（14 B）

用法：
  py guidance_bridge.py --gnc-dir "D:/电火箭制导控制/vtvl-gnc/latte-panda/tests" \
      --target 10 3 0 --duration 60 --out tmp/guidance_joint.csv

终端前馈：SOCP 只约束终端 jerk（a_N=a_{N-1}），不保证 a_N=g；t>tf 若继续用
规划末端 app_acc 当前馈，会形成 kp·Δr=a_ff 的假平衡（实测停在 1.21 m）。
本脚本在 t>=tf 后把前馈归零，只留 PD，详见
docs/04-数学建模/代码理论核查/SOCP制导联合仿真-2026-11-03.md。

"""
import argparse
import os
import socket
import struct
import sys
import time

import numpy as np


def load_gnc(gnc_dir):
    sys.path.insert(0, gnc_dir)
    import vtvl_sim  # noqa: E402

    return vtvl_sim


class Bridge:
    def __init__(self, gnc, rx_port, tx_port, target, kp=1.2, kd=0.9):
        self.gs = gnc
        self.g0 = gnc.g0
        self.target = np.asarray(target, dtype=float)  # [up, east, north]
        self.kp, self.kd = kp, kd

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", rx_port))
        self.sock.settimeout(0.5)
        self.dst = ("127.0.0.1", tx_port)

        self.plan = None
        self.t0 = None
        self.last_cmd_t = -1.0
        self.rows = []
        self.frames = 0
        self.cmds = 0

    # ---------- 协议 ----------
    def recv_state(self):
        try:
            data, _ = self.sock.recvfrom(256)
        except socket.timeout:
            return None
        if len(data) < 74 or data[0] != 0xAA or data[73] != 0x55:
            return None
        f = struct.unpack("<18f", data[1:73])
        return {
            "auto": f[0], "fire": f[1], "t": f[2],
            "r": np.array([f[5], f[4], f[3]], dtype=float),   # [up, east, north]
            "v": np.array([f[8], f[7], f[6]], dtype=float),
            "att": (f[9], f[10], f[11]),
            "gyro": (f[12], f[13], f[14]),
            "acc": (f[15], f[16], f[17]),
        }

    def send_cmd(self, acc_u, acc_e, acc_n):
        frame = b"\xaa" + struct.pack("<3f", acc_u, acc_e, acc_n) + b"\x55"
        self.sock.sendto(frame, self.dst)
        self.cmds += 1

    # ---------- 制导 ----------
    def plan_once(self, state):
        r_i = np.clip(state["r"], -50, 50)
        v_i = np.clip(state["v"], -5, 5)
        if r_i[0] < 0.5:
            # 制导侧规划器位置下限 -0.5 m；还没爬升到 0.5 m 以上时先不规划
            return False
        self.t_plan_t0 = state["t"]
        # 自由终端时间搜索（二分）在部分初始状态下单次可达数十秒，会拖死实时闭环；
        # 这里改为「按距离/速度给候选 t_f，逐个求解取第一个可行解」，单次约 0.13 s。
        dist = float(np.linalg.norm(self.target - r_i))
        v_mag = float(np.linalg.norm(v_i))
        base = max(3.0, (dist + 0.5 * v_mag) / 2.5)
        plan = None
        for tf in (base, base * 1.4, base * 2.0, 4.0, 6.0, 8.0, 12.0):
            tf = float(np.clip(tf, 2.0, 20.0))
            plan = self.gs.solve_trajplan(r_i, v_i, self.target, np.zeros(3),
                                          t_i=0.0, t_f=tf)
            if plan is not None:
                break
        if plan is None:
            print("[bridge] trajplan FAILED (all candidate t_f infeasible)")
            return False
        self.plan = plan
        print(f"[bridge] trajplan OK tf={plan['tf']:.2f}s "
              f"f_opt={plan.get('f_opt', float('nan')):.4f} "
              f"from r={np.round(r_i, 2)} v={np.round(v_i, 2)} to {self.target}")
        return True

    def command(self, state):
        if self.plan is None:
            return None
        t_rel = state["t"] - self.t_plan_t0 if hasattr(self, "t_plan_t0") else 0.0
        times = self.plan["time"]
        idx = int(np.searchsorted(times, t_rel))
        idx = min(max(idx, 0), len(times) - 1)
        r_ref = self.plan["position"][idx]
        v_ref = self.plan["velocity"][idx]
        # SOCP 只约束终端 jerk（a_N=a_{N-1}），不约束 a_N=g；实测规划末端
        # app_acc=11.24（悬停应为 9.79），t>tf 继续用它作前馈会形成
        # kp*Δr=a_ff 的假平衡（SIL 实测停在 1.21 m）。规划结束后前馈归零，
        # 只留位置/速度反馈把终端误差拉回 0。
        if t_rel >= float(times[-1]):
            a_ff = np.zeros(3)
        else:
            a_ff = self.plan["app_acc"][idx] - np.array([self.g0, 0.0, 0.0])

        r_err = r_ref - state["r"]
        v_err = v_ref - state["v"]
        a_cmd = a_ff + self.kp * r_err + self.kd * v_err

        # 水平/垂直限幅（与制导侧 ±1 m/s² 保守量级一致，可调）
        a_cmd = np.clip(a_cmd, -6.0, 6.0)
        return a_cmd, r_ref, v_ref, float(t_rel)

    def run(self, duration, out_csv):
        t_start = time.time()
        while time.time() - t_start < duration:
            state = self.recv_state()
            if state is None:
                continue
            self.frames += 1

            if self.plan is None:
                if self.plan_once(state):
                    self.t_plan_t0 = state["t"]
                else:
                    continue

            now = time.time()
            if self.plan is not None and (now - self.last_cmd_t) >= 0.02:
                self.last_cmd_t = now
                res = self.command(state)
                if res is not None:
                    a_cmd, r_ref, v_ref, t_rel = res
                    self.send_cmd(a_cmd[0], a_cmd[1], a_cmd[2])
                    self.rows.append([
                        state["t"], t_rel,
                        state["r"][0], state["r"][1], state["r"][2],
                        state["v"][0], state["v"][1], state["v"][2],
                        r_ref[0], r_ref[1], r_ref[2],
                        a_cmd[0], a_cmd[1], a_cmd[2],
                    ])

        os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
        with open(out_csv, "w", encoding="utf-8") as fh:
            fh.write("t_sil,t_rel,r_up,r_e,r_n,v_up,v_e,v_n,"
                     "rref_up,rref_e,rref_n,a_up,a_e,a_n\n")
            for row in self.rows:
                fh.write(",".join(f"{v:.6f}" for v in row) + "\n")
        print(f"[bridge] frames={self.frames} cmds={self.cmds} rows={len(self.rows)}")
        print(f"[bridge] log -> {out_csv}")
        if self.rows:
            last = self.rows[-1]
            print(f"[bridge] final pos = ({last[2]:.2f}, {last[3]:.2f}, {last[4]:.2f}) "
                  f"target = {self.target}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gnc-dir", required=True)
    ap.add_argument("--rx-port", type=int, default=7002, help="SIL 回传状态的端口")
    ap.add_argument("--tx-port", type=int, default=7001, help="发给 SIL 指令的端口")
    ap.add_argument("--target", type=float, nargs=3, default=[10.0, 3.0, 0.0],
                    metavar=("UP", "EAST", "NORTH"))
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--kp", type=float, default=1.2)
    ap.add_argument("--kd", type=float, default=0.9)
    ap.add_argument("--out", default="tmp/guidance_joint.csv")
    a = ap.parse_args()

    gnc = load_gnc(a.gnc_dir)
    br = Bridge(gnc, a.rx_port, a.tx_port, a.target, kp=a.kp, kd=a.kd)
    print(f"[bridge] listening 127.0.0.1:{a.rx_port}, sending to :{a.tx_port}, "
          f"target={a.target}")
    br.run(a.duration, a.out)


if __name__ == "__main__":
    main()
