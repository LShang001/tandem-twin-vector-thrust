#!/usr/bin/env bash
# 用 ASCII 盘符运行需要宿主 g++ 的命令（MinGW ld 不能写中文输出路径）
#
# 背景：本项目在中文路径下。实测 MinGW-W64 的 ld.exe 写非 ASCII 输出路径会报
#   `cannot open output file ...: Invalid argument`；读中文源文件、bash 重定向、
#   Python 写文件都正常。所以只有“链接产物落盘”这一步需要 ASCII 路径。
#
# 做法：把 <repo>/tmp 临时映射到一个空闲盘符（T..Z），导出 BIN_DIR=<盘符>:/<sub>，
#       命令结束后自动解除映射；中途被 kill 也会在下次运行时自愈（清理指向本仓库
#       tmp 的残留映射）。不会长期多出一个盘符。
#
# 用法（在仓库根目录）：
#   bash tools/with-ascii-bin.sh --sub runall -- bash TandemVec_FCS/test_host/run_all.sh
#   bash tools/with-ascii-bin.sh --sub matrix -- bash TandemVec_FCS/test_host/fw_sil/run_failsafe_matrix.sh
#   bash tools/with-ascii-bin.sh --sub demo -- bash -c 'echo "$BIN_DIR"; echo hi > "$BIN_DIR/x"'
#
# 选项：
#   --sub NAME   产物子目录名（默认取被调用命令的 basename）
#   --drive X    指定盘符（默认自动挑空闲的 T..Z）
#   --keep       保留映射（调试用；正常不要加，会留下盘符）
#
# 退出码：透传被调用命令；2=参数错误；4=没有可用盘符/映射失败。
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET_WIN="$(cd "$ROOT/tmp" && pwd -W 2>/dev/null || printf '%s' "$ROOT/tmp")"
LOCK_PREFIX="$ROOT/tmp/.ascii-bin-lock-"
normpath() { printf '%s' "$1" | tr '\\' '/' | tr 'A-Z' 'a-z'; }
TARGET_NORM="$(normpath "$TARGET_WIN")"
SUB=""
WANT_DRIVE=""
KEEP=0

while [ $# -gt 0 ]; do
  case "$1" in
    --sub)   SUB="${2:?--sub 需要名称}"; shift 2 ;;
    --drive) WANT_DRIVE="${2:?--drive 需要盘符}"; WANT_DRIVE="${WANT_DRIVE%:}"; shift 2 ;;
    --keep)  KEEP=1; shift ;;
    --)      shift; break ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) break ;;
  esac
done

if [ $# -eq 0 ]; then
  echo "用法: bash tools/with-ascii-bin.sh [--sub NAME] [--drive X] -- <命令...>" >&2
  exit 2
fi

# 自愈：清理指向本仓库 tmp 的残留映射（上次被 kill -9 / 中途崩溃留下）。
# 若该盘符的锁文件仍指向活着的进程，说明另一个包装器正在用，不能抢。
subst 2>/dev/null | sed -n 's/^\([A-Za-z]\):.*=> *\(.*\)$/\1|\2/p' |
while IFS='|' read -r drv path; do
  if [ "$(normpath "$path")" = "$TARGET_NORM" ]; then
    lock="${LOCK_PREFIX}${drv}"
    if [ -f "$lock" ] && kill -0 "$(cat "$lock" 2>/dev/null)" 2>/dev/null; then
      continue
    fi
    MSYS2_ARG_CONV_EXCL='*' subst "$drv:" /D >/dev/null 2>&1 || true
    rm -f "$lock"
  fi
done

# 选盘符：优先指定值，否则 T..Z 里第一个未占用的。
DRIVE=""
if [ -n "$WANT_DRIVE" ]; then
  if [ -e "$WANT_DRIVE:/" ]; then
    echo "[ascii-bin] 盘符 $WANT_DRIVE: 已被占用（subst 或物理盘），换一个" >&2
  else
    DRIVE="$WANT_DRIVE"
  fi
fi
if [ -z "$DRIVE" ]; then
  for d in T U V W X Y Z; do
    if [ ! -e "$d:/" ]; then DRIVE="$d"; break; fi
  done
fi
if [ -z "$DRIVE" ]; then
  echo "[ascii-bin] T..Z 都已被占用，无法建立 ASCII 映射" >&2
  exit 4
fi

if ! MSYS2_ARG_CONV_EXCL='*' subst "$DRIVE:" "$TARGET_WIN" >/dev/null 2>&1; then
  echo "[ascii-bin] subst $DRIVE: -> $TARGET_WIN 失败" >&2
  exit 4
fi
# 锁文件：让并发的另一个包装器知道这个盘符有人正在用（自愈时跳过）。
printf '%s\n' "$$" > "${LOCK_PREFIX}${DRIVE}"

if [ -z "$SUB" ]; then
  SUB="$(basename "${1%%/}")"
  SUB="${SUB:-bin}"
fi
export BIN_DIR="$DRIVE:/$SUB"
mkdir -p "$BIN_DIR"

cleanup() {
  rm -f "${LOCK_PREFIX}${DRIVE}"
  if [ "$KEEP" = "0" ]; then
    MSYS2_ARG_CONV_EXCL='*' subst "$DRIVE:" /D >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[ascii-bin] $DRIVE: -> $TARGET_WIN；BIN_DIR=$BIN_DIR" >&2
set +e
"$@"
rc=$?
set -e
exit "$rc"
