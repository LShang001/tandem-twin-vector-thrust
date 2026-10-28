#!/usr/bin/env bash
# 构建固件算法宿主直调共享库（fw_algo）。源码就是固件 Ekf15State 本身，
# 不复制任何滤波公式；Python 侧通过 ctypes 加载。
#
# 用法：bash test_host/fw_algo/build_fw_algo.sh [--print-path]
# 输出：${BIN_DIR:-${TMPDIR:-/tmp}/tv_fw_algo}/fw_algo.dll（Linux 为 libfw_algo.so）
# 说明：默认输出到 ASCII 临时目录——MinGW 链接器无法在中文路径下写目标文件。
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fw_algo}"
CXX="${CXX:-g++}"
STD="${STD:-c++17}"
OPT="${OPT:--O2}"
mkdir -p "$OUT_DIR"

INCS="-I$ROOT/include -I$ROOT/src -I$ROOT/lib/eigen/src -I$ROOT/lib/navigation-main/src"
INCS="$INCS -I$ROOT/lib/units/src -I$ROOT/lib/ICM42688/src -I$ROOT/test_host"
# 标量 Eigen：与固件/SIL 同口径，杜绝多核向量化造成的边际样本双吸引子。
# EKF_NAV_TELEMETRY 只增加只读协方差访问器（全仓仅此一处守卫），不改滤波数学。
FLAGS="-std=$STD $OPT -D_USE_MATH_DEFINES -DEKF_NAV_TELEMETRY"
FLAGS="$FLAGS -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE"
SRCS="$ROOT/test_host/fw_algo/fw_algo_api.cpp
$ROOT/lib/units/src/convang.cpp
$ROOT/lib/units/src/convangacc.cpp
$ROOT/lib/units/src/convangvel.cpp"

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    LIB="$OUT_DIR/fw_algo.dll"
    # 自包含 MinGW 运行库：Python 进程 PATH 上没有 libstdc++/libgcc。
    $CXX $FLAGS $INCS $SRCS -shared -static -o "$LIB" \
      -Wl,--out-implib,"$OUT_DIR/libfw_algo.a"
    ;;
  *)
    LIB="$OUT_DIR/libfw_algo.so"
    $CXX $FLAGS $INCS $SRCS -shared -fPIC -o "$LIB"
    ;;
esac

if [ "${1:-}" = "--print-path" ]; then
  echo "$LIB"
else
  echo "built: $LIB"
fi
