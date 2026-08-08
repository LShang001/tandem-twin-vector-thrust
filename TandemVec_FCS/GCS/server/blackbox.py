# -*- coding: utf-8 -*-
"""
blackbox.py — W25N01GV 黑匣子下载与解析（移植自 tools/flash_export.py，v2 I/P 差分帧 + S/E 飞行段）

帧格式 v2（与固件 lib/W25N01GV/W25N01GV_log.h 一致，小端，CRC16-CCITT poly 0x1021 init 0xFFFF）:
  I: magic(2)+type(1)=0x49+seq(2)+t_ms(4)+22xfloat(88)+crc16(2) = 99B
  P: magic(2)+type(1)=0x50+seq(2)+dt_ms(2)+21xint16(42)+crc16(2) = 51B
  S: magic(2)+type(1)=0x53+seg(2)+t_ms(4)+通道名表 224B(\\0结尾)+crc16(2) = 236B 定长
  E: magic(2)+type(1)=0x45+seg(2)+dur_ms(4)+frames(4)+crc16(2) = 15B

列名自描述：S 帧携带通道名表，解析以 S 帧为准（不硬编码）。
"""
import struct
import time

MAGIC = b'\xAA\x55'
TYPE_I = 0x49
TYPE_P = 0x50
TYPE_S = 0x53
TYPE_E = 0x45
IFRAME_SIZE = 99
PFRAME_SIZE = 51
EFRAME_SIZE = 15
SFRAME_MAX = 236
PAYLOAD_LEN = 88          # 22 floats
DELTA_SCALE = 100

DEFAULT_COLS = ("seq,t_ms,roll_deg,pitch_deg,heading_deg,"
                "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
                "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
                "vel_n_ms,vel_e_ms,vel_d_ms,"
                "rel_n_m,rel_e_m,rel_d_m,"
                "tvc1_deg,tvc2_deg,valve_ctrl,p1,p2")


def crc16(data: bytes) -> int:
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF) — 与固件一致"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def extract_frames(data: bytes):
    """magic 同步切帧，返回 (type, frame_bytes) 列表（CRC 校验通过）"""
    frames = []
    i = 0
    n = len(data)
    while i < n - 2:
        if data[i:i + 2] == MAGIC:
            t = data[i + 2] if i + 2 < n else 0
            if t == TYPE_I and i + IFRAME_SIZE <= n:
                f = data[i:i + IFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_I, f))
                    i += IFRAME_SIZE
                    continue
            if t == TYPE_P and i + PFRAME_SIZE <= n:
                f = data[i:i + PFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_P, f))
                    i += PFRAME_SIZE
                    continue
            if t == TYPE_E and i + EFRAME_SIZE <= n:
                f = data[i:i + EFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_E, f))
                    i += EFRAME_SIZE
                    continue
            if t == TYPE_S and i + SFRAME_MAX <= n:
                f = data[i:i + SFRAME_MAX]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_S, f))
                    i += SFRAME_MAX
                    continue
            i += 1
        else:
            i += 1
    return frames


def decode_frames(frames):
    """I/P 差分还原 + S/E 段切分。返回 (rows, segments, columns)
    rows: [(seq, t_ms, vals[22], seg)]；segments: [{num,start_tms,dur_ms,frames,columns}]"""
    rows = []
    N_F = PAYLOAD_LEN // 4
    ref = [0.0] * N_F
    ref_t = 0
    cur_seg = 0
    columns = None
    default_cols = DEFAULT_COLS.split(',')
    segments = []

    for typ, f in frames:
        if typ == TYPE_S:
            seg_num = struct.unpack('<H', f[3:5])[0]
            t_ms = struct.unpack('<I', f[5:9])[0]
            names_raw = f[9:].split(b'\x00')[0]
            names = [x for x in names_raw.decode('ascii', errors='replace').split(',') if x]
            if names:
                columns = names
            cur_seg = seg_num
            segments.append({'num': seg_num, 'start_tms': t_ms,
                             'dur_ms': 0, 'frames': 0, 'columns': list(names)})
        elif typ == TYPE_E:
            seg_num = struct.unpack('<H', f[3:5])[0]
            dur_ms = struct.unpack('<I', f[5:9])[0]
            frames_cnt = struct.unpack('<I', f[9:13])[0]
            for sg in segments:
                if sg['num'] == seg_num:
                    sg['dur_ms'] = dur_ms
                    sg['frames'] = frames_cnt
        elif typ == TYPE_I:
            t_ms = struct.unpack('<I', f[5:9])[0]
            vals = list(struct.unpack(f'<{N_F}f', f[9:9 + PAYLOAD_LEN]))
            ref = vals
            ref_t = t_ms
            rows.append((0, t_ms, vals, cur_seg))
        elif typ == TYPE_P:
            dt = struct.unpack('<h', f[5:7])[0]
            t_ms = ref_t + dt
            # ★ P 帧实际携带 21 个增量（第 22 槽被帧尾 CRC 占用，见固件
            #   buildPFrame 2026-08-08 修正）——末通道（p2）保持最近参考值。
            #   原 22 增量解析会令 p2 被 CRC 字节污染（tools/flash_export.py 同步修正）
            deltas = struct.unpack(f'<{N_F - 1}h', f[7:7 + (N_F - 1) * 2])
            vals = [ref[i] + deltas[i] / DELTA_SCALE for i in range(N_F - 1)]
            vals.append(ref[N_F - 1])
            ref = vals
            ref_t = t_ms
            rows.append((0, t_ms, vals, cur_seg))

    for sg in segments:
        sg['frames'] = max(sg['frames'], sum(1 for r in rows if r[3] == sg['num']))

    if columns is None:
        columns = default_cols
    return rows, segments, columns


class DbgSession:
    """DBG 控制台会话：进入调试模式 → 发命令 → 收文本/原始字节。

    串口已处于 AnoCom 模式；进入 DBG 后固件停止遥测（互斥，固件行为）。
    导出流程（flash export <start> <count>）：
      1. 发命令后先收到 "[DBG] export start=.. count=.." 文本行
      2. 随后 count×2048B 原始页字节（无分隔）
    """

    def __init__(self, write_fn):
        self._write = write_fn
        self._rx = bytearray()
        self._export_state = 'idle'   # idle | waiting_header | collecting
        self._export_buf = bytearray()
        self._export_total = 0        # 期望总页数
        self._export_done = None      # 回调 on_export(bytes)

    # ---- 模式切换 ----
    def enter(self):
        self._write(b'DBG\n')

    def exit(self):
        self._write(b'exit\n')

    def send_cmd(self, line: str):
        self._write((line + '\n').encode('ascii', errors='ignore'))

    def start_export(self, start_page: int, count: int, on_done):
        self._export_state = 'waiting_header'
        self._export_buf = bytearray()
        self._export_total = count
        self._export_done = on_done
        self.send_cmd(f'flash export {start_page} {count}')

    # ---- 字节流入（由串口回调驱动）----
    def feed(self, chunk: bytes):
        if not chunk:
            return
        if self._export_state == 'idle':
            self._rx.extend(chunk)
            return
        if self._export_state == 'waiting_header':
            # 等待 "[DBG] export start=.. count=..\n" 行尾——必须精确匹配该行，
            # 否则 findseg 等前一命令的迟到文本行（也含 \n）会被误当行尾，
            # 导致页数据起点错位、收集长度永远对不齐而超时
            idx = 0
            pos = -1
            while True:
                nl = chunk.find(b'\n', idx)
                if nl < 0:
                    break
                if b'export start=' in chunk[idx:nl]:
                    pos = nl
                    break
                idx = nl + 1
            if pos < 0:
                return
            rest = chunk[pos + 1:]
            self._export_state = 'collecting'
            if rest:
                self.feed(rest)
            return
        # collecting：累计 count×2048B 原始页字节
        self._export_buf.extend(chunk)
        if len(self._export_buf) >= self._export_total * 2048:
            done = self._export_done
            self._export_done = None
            self._export_state = 'idle'
            if done:
                done(bytes(self._export_buf[:self._export_total * 2048]))
            self._export_buf = bytearray()

    def drain_text(self) -> str:
        """取出已收集的文本输出（按行切分，保留未完整行）"""
        out = self._rx.decode('utf-8', errors='replace')
        self._rx.clear()
        return out

    def discard_all(self):
        self._rx.clear()
