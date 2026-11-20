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

for src in "${SOURCES[@]}"; do
  obj="$(obj_for "$src")"
  dep="${obj%.o}.d"
  objects+=("$obj")

  need=1
  if [ -f "$obj" ]; then
    need=0
    # 源文件比对象新 → 重编
    if [ "$src" -nt "$obj" ]; then
      need=1
    # 任一依赖头比对象新 → 重编
    elif [ -f "$dep" ]; then
      # .d 内容形如：obj: src.h1 src.h2 ...
      # shellcheck disable=SC2046
      for d in $(sed -e 's/^[^:]*://' -e 's/\\$//' -e 's/\\ /\x01/g' "$dep" | tr ' ' '\n' | tr '\001' ' '); do
        if [ -f "$d" ] && [ "$d" -nt "$obj" ]; then
          need=1
          break
        fi
      done
    fi
  fi

  if [ "$need" -eq 1 ]; then
    echo "  [compile] $src" >&2
    # shellcheck disable=SC2086
    if ! $CXX $CXXFLAGS $INC -MMD -MP -c "$src" -o "$obj"; then
      echo "编译失败：$src" >&2
      exit 1
    fi
    built=$((built + 1))
  else
    cached=$((cached + 1))
  fi
done

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