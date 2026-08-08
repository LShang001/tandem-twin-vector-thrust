# -*- coding: utf-8 -*-
"""黑匣子 I/P/S/E 帧解析测试 — 与固件 W25N01GV_log.h / tools/flash_export.py 对齐"""
import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'server'))

import blackbox as bb

MAGIC = bb.MAGIC
N_F = 13  # float 通道数（★2026-08-09 固件通道表裁剪 22→13）


def mk_frame(typ: int, body: bytes) -> bytes:
    """构造完整帧：magic + type + body + crc16"""
    f = MAGIC + bytes([typ]) + body
    return f + struct.pack('<H', bb.crc16(f))


def mk_s_frame(seg: int, t_ms: int, names: list) -> bytes:
    """固件 buildSFrame 布局：236B 固定（头 9B + 名表≤224B + 1B 预留 + CRC 2B）"""
    f = bytearray(236)
    f[0:2] = MAGIC
    f[2] = bb.TYPE_S
    f[3:5] = struct.pack('<H', seg)
    f[5:9] = struct.pack('<I', t_ms)
    nb = ','.join(names).encode('ascii')
    f[9:9 + len(nb)] = nb
    crc = bb.crc16(f[:234])
    f[234] = crc & 0xFF
    f[235] = (crc >> 8) & 0xFF
    return bytes(f)


def mk_i_frame(seq: int, t_ms: int, vals: list) -> bytes:
    assert len(vals) == N_F
    body = struct.pack('<H', seq) + struct.pack('<I', t_ms) + struct.pack(f'<{N_F}f', *vals)
    return mk_frame(bb.TYPE_I, body)


def mk_p_frame(seq: int, dt_ms: int, deltas: list) -> bytes:
    assert len(deltas) == N_F - 1
    body = struct.pack('<H', seq) + struct.pack('<h', dt_ms) + struct.pack(f'<{N_F - 1}h', *deltas)
    return mk_frame(bb.TYPE_P, body)


def mk_e_frame(seg: int, dur_ms: int, frames: int) -> bytes:
    body = struct.pack('<H', seg) + struct.pack('<I', dur_ms) + struct.pack('<I', frames)
    return mk_frame(bb.TYPE_E, body)


def test_crc16_reference():
    """CRC16-CCITT 对照已知向量"""
    # poly 0x1021 init 0xFFFF：空串校验值
    assert bb.crc16(b'') == 0xFFFF
    # "123456789" 的 CRC-16/CCITT-FALSE 已知值 0x29B1
    assert bb.crc16(b'123456789') == 0x29B1


def test_extract_frames_sync():
    """magic 同步切帧：前缀垃圾 + 各类型帧 + 尾部垃圾"""
    vals = [1.5 + i for i in range(N_F)]
    sf_names = [f'c{i}' for i in range(N_F)]   # S 帧名数须与帧通道数一致
    stream = (b'\x00\xFF' * 3
              + mk_s_frame(2, 1000, sf_names)
              + mk_i_frame(0, 1000, vals)
              + mk_p_frame(1, 5, [10] * (N_F - 1))
              + mk_e_frame(2, 5000, 100)
              + b'\xAA\x55\x49\x00')   # 不完整帧尾部
    frames = bb.extract_frames(stream)
    types = [t for t, _ in frames]
    assert types == [bb.TYPE_S, bb.TYPE_I, bb.TYPE_P, bb.TYPE_E]


def test_extract_frames_corrupt_crc_dropped():
    """CRC 错误的帧被丢弃（不进入帧列表）"""
    good = mk_i_frame(0, 1000, [0.0] * N_F)
    bad = bytearray(good)
    bad[9] ^= 0xFF
    frames = bb.extract_frames(bytes(bad) + good)
    assert len(frames) == 1 and frames[0][0] == bb.TYPE_I


def test_decode_single_segment():
    """I + P 差分还原 + S/E 段元数据"""
    cols = ['t_ms', 'roll_deg', 'pitch_deg', 'heading_deg'] + [f'ch{i}' for i in range(4, N_F)]
    stream = (
        mk_s_frame(1, 5000, cols)
        + mk_i_frame(0, 5000, [5000, 10.0, -5.0, 90.0] + [0.0] * (N_F - 4))
        # P 帧差分：deltas[0]=t_ms 通道（0），deltas[1]=roll 通道（+0.1° ×100 = 10）
        + mk_p_frame(1, 5, [0, 10] + [0] * (N_F - 3))
        + mk_p_frame(2, 5, [0, 5] + [0] * (N_F - 3))
        + mk_e_frame(1, 10000, 3)
    )
    rows, segments, columns = bb.decode_frames(bb.extract_frames(stream))
    assert columns == cols
    assert len(rows) == 3
    assert rows[0][1] == 5000 and abs(rows[0][2][1] - 10.0) < 1e-6
    assert rows[1][1] == 5005 and abs(rows[1][2][1] - 10.1) < 1e-6
    assert rows[2][1] == 5010 and abs(rows[2][2][1] - 10.15) < 1e-6   # 10.1 + 0.05 差分累积
    # P 帧 21 增量：末通道（p2）不差分，保持 I 帧参考值
    assert rows[1][2][N_F - 1] == rows[0][2][N_F - 1]
    assert rows[2][2][N_F - 1] == rows[0][2][N_F - 1]
    assert len(segments) == 1
    assert segments[0]['num'] == 1 and segments[0]['dur_ms'] == 10000
    assert segments[0]['frames'] >= 3


def test_decode_missing_s_frame_uses_default_cols():
    """无 S 帧时回退默认 13 列（当前固件通道表）"""
    frames = bb.extract_frames(mk_i_frame(0, 1000, [1.0] * N_F))
    rows, segments, columns = bb.decode_frames(frames)
    assert columns == bb.DEFAULT_COLS.split(',')
    assert len(columns) == N_F and len(rows) == 1


def test_s_frame_prescan_sets_channel_count():
    """全量预扫描：S 帧给出权威通道数（旧 22 通道数据也能解析）"""
    old22 = [f'ch{i}' for i in range(22)]
    # 22 通道 I 帧（绕过 mk_i_frame 的 13 通道断言）
    body = struct.pack('<H', 0) + struct.pack('<I', 9000) + struct.pack('<22f', *([9000, 1.0] + [0.0] * 20))
    i22 = mk_frame(bb.TYPE_I, body)
    stream = mk_s_frame(3, 9000, old22) + i22
    frames = bb.extract_frames(stream)
    rows, segments, columns = bb.decode_frames(frames)
    assert len(columns) == 22 and len(rows) == 1
    assert rows[0][2][0] == 9000


def test_dbg_session_export_strips_header():
    """DBG export：剥离 "[DBG] export start=.. count=.." 文本行，按 2048B/页收集"""
    out = []
    sess = bb.DbgSession(lambda b: None)
    sess.start_export(10, 2, out.append)
    # 模拟一次读出：文本行 + 1.5 页
    chunk = b'[DBG] export start=10 count=2\n' + bytes(range(256)) * 12
    sess.feed(chunk)
    assert out == []      # 尚未收满 2 页
    # 第二段补足
    sess.feed(bytes(2048))
    assert len(out) == 1 and len(out[0]) == 2 * 2048
    # export 结束后文本恢复收集
    sess.feed(b'[DBG] done\n')
    assert 'done' in sess.drain_text()
