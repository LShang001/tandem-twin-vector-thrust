#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 增量构建器
#
# 为什么需要它：全链路目标要一次链接 10+ 个固件源文件（含 Eigen 模板重度实例化
# 的 state_data.cpp 与 1139 行的 sensor_imu.cpp），全量编译单次超过 5 分钟。
# 每改一行桩就重编整条链，迭代成本不可接受——这是本文件存在的原因。
#
# 做法：每个源文件编译成独立 .o 带 -MMD 依赖文件；下次按"源文件或任一依赖头
# 比 .o 新"判断是否重编，只重编受影响的单元，然后重新链接。
#
# 用法：
#   bash build_incremental.sh <输出名> [编译选项通过 CXXFLAGS 传入] <源文件...>
# 环境变量：
#   BIN_DIR  产物与对象缓存目录（默认系统临时目录，ASCII 路径）
#   CXXFLAGS 额外编译选项
#   OBJ_NAME 对象目录名（同一 BIN_DIR 下多目标隔离）
#
# 退出码：0 成功；2 参数错误；其他为编译/链接失败。
# ============================================================
set -u

if [ $# -lt 2 ]; then
  echo "用法：bash build_incremental.sh <输出名> <源文件...>" >&2
  exit 2
fi

TARGET_NAME="$1"
shift
SOURCES=("$@")

BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"
OBJ_NAME="${OBJ_NAME:-$TARGET_NAME}"
OBJ_DIR="$BIN_DIR/obj-$OBJ_NAME"
mkdir -p "$OBJ_DIR"

CXX="${CXX:-g++}"
CXXFLAGS="${CXXFLAGS:--std=c++17 -O0 -g}"

# 源文件路径 → 对象文件名（把分隔符换成下划线，避免建子目录）
obj_for() {
  local s="$1"
  printf '%s/%s.o' "$OBJ_DIR" "$(printf '%s' "$s" | tr '/\\.' '___')"
}

built=0
cached=0
objects=()
to_build=()          # 待编源文件
to_build_obj=()      # 对应对象路径
to_build_hash=()     # 对应组合哈希

# ★重编判据用内容哈希，不用 `-nt` 时间戳比较。
# 原因（实测）：源文件在 /mnt/d（Windows 文件系统）、对象在 ext4 /tmp，
# 跨文件系统的时间戳精度与时区不一致，`[ "$src" -nt "$obj" ]` 会恒为真，
# 导致每次调用都全量重编（单轮 5–10 分钟），改一行也等这么久。
# 现象上还更隐蔽：有时又恒为假，改了代码却复用旧对象、测的是旧二进制。
hash_for() {
  printf '%s/.hash_%s' "$OBJ_DIR" "$(printf '%s' "$1" | tr '/\\.' '___')"
}

for src in "${SOURCES[@]}"; do
  obj="$(obj_for "$src")"
  dep="${obj%.o}.d"
  hfile="$(hash_for "$src")"
  objects+=("$obj")

  # 组合哈希 = 编译选项 + 源文件 + 全部依赖头（从上次生成的 .d 里取）
  # ★必须把 CXXFLAGS 算进去：多个构建脚本共用同一个 OBJ_DIR，但宏不同
  #   （-DFW_SIL_REAL_NAV / -DBFS_EKF_BARO_ALTITUDE_UPDATE / -DFW_SIL_REAL_IMU…）。
  #   只哈希源文件时，切换配置会复用按另一套宏编出来的 .o——编译不报错，
  #   运行时行为却对不上，表现为"同样的输入、同样的代码，结果时好时坏"。
  #   这类假绿/假失败比编译错误难查得多，因此编译选项必须进哈希。
  cur_hash="$(
    {
      printf 'CXXFLAGS:%s\n' "$CXXFLAGS"
      md5sum "$src" 2>/dev/null
      if [ -f "$dep" ]; then
        # shellcheck disable=SC2046
        for d in $(sed -e 's/^[^:]*://' -e 's/\\$//' -e 's/\\ /\x01/g' "$dep" | tr ' ' '\n' | tr '\001' ' '); do
          [ -f "$d" ] && md5sum "$d" 2>/dev/null
        done
      fi
    } | md5sum | cut -d' ' -f1
  )"

  if [ -f "$obj" ] && [ -f "$hfile" ] && [ "$(cat "$hfile")" = "$cur_hash" ]; then
    cached=$((cached + 1))
  else
    to_build+=("$src")
    to_build_obj+=("$obj")
    to_build_hash+=("$cur_hash")
  fi
done

# ---- 并行编译（单核串行编 35 个文件要 13 分钟，24 核并行约 1–2 分钟）----
if [ ${#to_build[@]} -gt 0 ]; then
  JOBS="${BUILD_JOBS:-8}"
  log_dir="$OBJ_DIR/.logs"
  mkdir -p "$log_dir"

  compile_one() {
    local idx="$1"
    local src="${to_build[$idx]}"
    local obj="${to_build_obj[$idx]}"
    echo "  [compile] $src" >&2
    # shellcheck disable=SC2086
    $CXX $CXXFLAGS $INC -MMD -MP -c "$src" -o "$obj" \
      > "$log_dir/$idx.log" 2>&1
  }

  running=0
  pids=()
  failed=0
  for i in "${!to_build[@]}"; do
    compile_one "$i" &
    pids+=($!)
    running=$((running + 1))
    if [ "$running" -ge "$JOBS" ]; then
      if ! wait "${pids[0]}"; then failed=1; fi
      pids=("${pids[@]:1}")
      running=$((running - 1))
    fi
  done
  for p in "${pids[@]}"; do
    if ! wait "$p"; then failed=1; fi
  done

  if [ "$failed" -ne 0 ]; then
    echo "编译失败：详情见 $log_dir/*.log" >&2
    for i in "${!to_build[@]}"; do
      if [ -s "$log_dir/$i.log" ]; then
        echo "--- ${to_build[$i]} ---" >&2
        tail -20 "$log_dir/$i.log" >&2
      fi
    done
    exit 1
  fi

  for i in "${!to_build[@]}"; do
    printf '%s' "${to_build_hash[$i]}" > "$(hash_for "${to_build[$i]}")"
  done
  built=${#to_build[@]}
fi

echo "  [objects] 重编 $built 个，复用 $cached 个" >&2

OUT="$BIN_DIR/$TARGET_NAME"
echo "  [link] $TARGET_NAME" >&2
# shellcheck disable=SC2086
if ! $CXX $CXXFLAGS "${objects[@]}" -o "$OUT"; then
  echo "链接失败：$TARGET_NAME" >&2
  exit 1
fi

# MinGW/WSL 下可能带 .exe
if [ -f "$OUT.exe" ] && [ ! -x "$OUT" ]; then OUT="$OUT.exe"; fi
echo "$OUT"