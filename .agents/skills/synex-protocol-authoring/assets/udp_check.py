#!/usr/bin/env python3
"""本地 UDP 收帧自检：验证 udp_ano_sim.py 发出的 ANO 帧长度与双校验。"""

import argparse
import socket
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=19000)
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(args.timeout)
    print(f"监听 127.0.0.1:{args.port}，等 {args.count} 帧", flush=True)

    got = 0
    bad = 0
    lengths = {}
    while got < args.count:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            print(f"超时：只收到 {got} 帧", flush=True)
            break
        got += 1
        lengths[len(data)] = lengths.get(len(data), 0) + 1
        s_calc = 0
        a_calc = 0
        for b in data[:-2]:
            s_calc = (s_calc + b) & 0xFF
            a_calc = (a_calc + s_calc) & 0xFF
        ok = (s_calc == data[-2]) and (a_calc == data[-1])
        if not ok:
            bad += 1
        if got <= 3 or not ok:
            print(f"帧{got}: len={len(data)} head={data[0]:02X}{data[1]:02X} "
                  f"func=0x{data[3]:02X} len字段={int.from_bytes(data[4:6], 'little')} "
                  f"校验={'OK' if ok else 'FAIL'} {data.hex(' ').upper()}", flush=True)

    print(f"收到 {got} 帧，帧长分布 {lengths}，校验失败 {bad}", flush=True)
    sock.close()
    return 0 if bad == 0 and got > 0 else 1


if __name__ == "__main__":
    sys.exit(main())