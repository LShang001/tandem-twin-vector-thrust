#!/usr/bin/env bash
# 构建固件核心宿主直调共享库（fw_algo）：**固件源码的唯一宿主构建入口**。
#
# 同一份固件源码既编进 SIL 可执行（fw_real.exe），也编进本共享库，
# 因此"仿真里跑的算法"与"固件里烧的算法"是同一份 C++ 源，不是复刻。
#
# 库内含四组 ABI：
#   1) fw_algo_api.h  —— 15 态 ESKF 直调；
#   2) fw_ctrl_api.cpp —— TandemVec 分配 / 线性推力分配；
#   3) fw_gnc_api.h   —— **控制链**（runGNCExecutive）+ 能力查询 + 参数注册表；
#   4) fw_nav_api.h   —— **导航步进**（handleNavigationSystem，含解析器/门控/重锚）。
#
# 用法：bash test_host/fw_algo/build_fw_algo.sh [--print-path] [--clean]
# 环境：BIN_DIR（默认 /tmp/tv_fw_algo）、EXTRA_CXXFLAGS（追加宏/优化）
# 输出：$BIN_DIR/fw_algo.dll（Windows）/ libfw_algo.so（其它）
# 说明：默认输出到 ASCII 临时目录——MinGW 链接器无法在中文路径下写目标文件。
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fw_algo}"
OBJ_DIR="$OUT_DIR/obj"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
STD="${STD:-c++17}"
OPT="${OPT:--O2}"
mkdir -p "$OBJ_DIR"
PRINT_PATH=0
for arg in "$@"; do
  case "$arg" in
    --clean) rm -rf "$OBJ_DIR"; mkdir -p "$OBJ_DIR" ;;
    --print-path) PRINT_PATH=1 ;;
    *) echo "[fwcore] 未知参数: $arg" >&2; exit 2 ;;
  esac
done

# ---- 固件核心源清单（唯一事实源：core_sources.sh）-------------------------
# 与 SIL 共用同一份清单；本脚本只追加 ABI 封装与参数注册表（DLL 特有）。
. "$HERE/core_sources.sh"
CORE_SOURCES="$FW_CORE_SOURCES src/ano_params.cpp"
CORE_C_SOURCES="$FW_CORE_C_SOURCES"
# 漂移检查：SIL 构建脚本必须列出同一批核心源，否则"仿真跑的算法"与"固件烧的算法"
# 会分叉。新增模块后忘记同步 SIL 时，这里直接失败而不是静默构建。
for src in $FW_CORE_SOURCES; do
  grep -q "$src" "$HERE/../fw_sil/build_incremental.sh" || {
    echo "[fwcore] 源清单漂移：fw_sil/build_incremental.sh 缺少 $src" >&2
    exit 3
  }
done
API_SOURCES="
test_host/fw_algo/fw_algo_api.cpp
test_host/fw_algo/fw_ctrl_api.cpp
test_host/fw_algo/fw_gnc_api.cpp
test_host/fw_algo/fw_nav_api.cpp
"

INCS="-I$ROOT/lib/ublox-main/src -I$ROOT/test_host/fw_sil/stub -I$ROOT/include"
INCS="$INCS -I$ROOT/src -I$ROOT/test_host/stub -I$ROOT/lib/navigation-main/src"
INCS="$INCS -I$ROOT/lib/units/src -I$ROOT/lib/AnoComProtocol -I$ROOT/lib/MAVLink"
INCS="$INCS -I$ROOT/lib/eigen/src"
# 导航 ABI 复用 SIL 的 UBX 帧生成（sil_sensor_model.h::BuildNavPvt/EOE），
# 不在这里再抄一份 NAV-PVT 布局。
INCS="$INCS -I$ROOT/test_host/fw_sil"
# 与 SIL 构建同口径：lib 下两层目录都进头文件搜索路径（ICM42688/src、MagSensor 等）。
for d in $(cd "$ROOT" && find lib -maxdepth 2 -type d); do INCS="$INCS -I$ROOT/$d"; done

# 与 SIL 同口径的宏：TANDEMVEC_FIRMWARE（可变参数实例）、FW_SIL_REAL_NAV（SIL 研究
# 接缝，控制/导航里# ifdef 的行为必须与 SIL 一致）、ARDUINO=300（Arduino 桩）。
# 标量 Eigen：与固件/SIL 一致，杜绝宿主向量化造成的边际样本双吸引子。
BUILD_ID="${FW_BUILD_ID:-$(cd "$ROOT" && git rev-parse --short HEAD 2>/dev/null || echo dev)}"
FLAGS="-std=$STD $OPT -D_USE_MATH_DEFINES -DEKF_NAV_TELEMETRY"
FLAGS="$FLAGS -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DARDUINO=300"
FLAGS="$FLAGS -DBFS_EKF_BARO_ALTITUDE_UPDATE"
FLAGS="$FLAGS -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE"
FLAGS="$FLAGS ${EXTRA_CXXFLAGS:-}"

obj_name() { printf '%s' "$1" | tr '/' '_' | tr -d '-'; }

# 编译选项不参与 mtime 依赖：选项变化必须整目录重建。
STAMP="$OBJ_DIR/.flags"
if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" != "$FLAGS" ]; then
  echo "[fwcore] flags changed -> rebuild all objects"
  rm -rf "$OBJ_DIR"
  mkdir -p "$OBJ_DIR"
fi
printf '%s\n' "$FLAGS" > "$STAMP"

# 构建号只影响 fw_gnc_api.cpp 的 caps.build_id：单独记录、不参与全局 flags 比对，
# 否则每次 git commit（哈希变化）都会触发全量重建（实测约 4 min）。
ID_STAMP="$OBJ_DIR/.build_id"
API_REBUILD=0
if [ ! -f "$ID_STAMP" ] || [ "$(cat "$ID_STAMP")" != "$BUILD_ID" ]; then
  API_REBUILD=1
  printf '%s\n' "$BUILD_ID" > "$ID_STAMP"
fi

OBJS=""
for src in $CORE_SOURCES $API_SOURCES; do
  obj="$OBJ_DIR/$(obj_name "$src").o"
  force=0
  extra=""
  if [ "$src" = "test_host/fw_algo/fw_gnc_api.cpp" ]; then
    extra="-DFW_BUILD_ID=\"$BUILD_ID\""
    [ "$API_REBUILD" = "1" ] && force=1
  fi
  if [ "$force" = "1" ] || [ ! -f "$obj" ] || [ "$ROOT/$src" -nt "$obj" ] || [ -n "$(find "$ROOT/include" "$ROOT/src" "$ROOT/lib/navigation-main/src" "$ROOT/test_host/fw_algo" -newer "$obj" -name '*.h' 2>/dev/null | head -1)" ]; then
    echo "[fwcore] $src"
    $CXX $FLAGS $INCS $extra -c "$ROOT/$src" -o "$obj"
  fi
  OBJS="$OBJS $obj"
done
for src in $CORE_C_SOURCES; do
  obj="$OBJ_DIR/$(obj_name "$src").o"
  if [ ! -f "$obj" ] || [ "$ROOT/$src" -nt "$obj" ]; then
    echo "[fwcore] $src"
    $CC -std=c11 -O2 -I"$ROOT/lib/ublox-main/src" -c "$ROOT/$src" -o "$obj"
  fi
  OBJS="$OBJS $obj"
done

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    LIB="$OUT_DIR/fw_algo.dll"
    # 自包含 MinGW 运行库：Python 进程 PATH 上没有 libstdc++/libgcc。
    $CXX $OBJS -shared -static -o "$LIB" -Wl,--out-implib,"$OUT_DIR/libfw_algo.a"
    ;;
  *)
    LIB="$OUT_DIR/libfw_algo.so"
    $CXX $OBJS -shared -fPIC -o "$LIB"
    ;;
esac

if [ "$PRINT_PATH" = "1" ]; then
  echo "$LIB"
else
  echo "built: $LIB (build_id=$BUILD_ID)"
fi
