#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
#  平板独立运行地面站：Termux 一键安装 + 启动脚本
#
#  用途：没有电脑时，用 Android 平板自己跑 GCS 后端，
#        并通过 OTG 插的 USB 数传与飞控通信（或走局域网 UDP）。
#
#  用法（Termux 里，任意目录）：
#      bash termux-setup.sh               # 只装依赖并自检
#      bash termux-setup.sh --run         # 装完起服务（平板浏览器开 127.0.0.1:8091）
#      bash termux-setup.sh --usb         # 列出 USB 设备让你选，然后授权并接上数传
#
#  服务起在 127.0.0.1（仅本机）：平板自己就是后端，浏览器访问 localhost 即
#  secure context，PWA 的 Service Worker 也能生效。
#
#  为什么必须用 pkg 装 numpy：Termux 的 aarch64 没有 numpy 预编译 wheel，
#  pip 会去源码编译（几十分钟且常失败），而频谱/统计依赖 numpy。
#
#  为什么串口必须走 termux-usb：Android 上普通进程打不开 /dev/ttyUSB0，
#  必须由 termux-usb 经 USB Host API 授权后，把设备以 **fd** 交给本进程。
#  termux-usb 调用 -e 指定的命令时，会把 fd 号作为**最后一个参数**追加。
# ============================================================================
set -e

GCS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${GCS_PORT:-8091}"
MODE="${1:-}"

# ---------------------------------------------------------------- 被 termux-usb 回调
# termux-usb -e <cmd> 会把 fd 号作为最后一个参数追加，所以这里收到的是纯数字。
if [[ "$MODE" =~ ^[0-9]+$ ]]; then
  FD="$MODE"
  echo "==> 收到 USB 设备 fd=$FD，启动后端并自动接上该链路"
  cd "$GCS_DIR"
  # 先起后端，再用 HTTP 调试接口触发连接（等着后端就绪，最多 15 s）
  python -m uvicorn server.main:app --host 127.0.0.1 --port "$PORT" &
  UV_PID=$!
  for _ in $(seq 1 60); do
    sleep 0.25
    if curl -fsS "http://127.0.0.1:$PORT/api/status" >/dev/null 2>&1; then break; fi
  done
  curl -sS -X POST "http://127.0.0.1:$PORT/api/cmd" \
       -H 'Content-Type: application/json' \
       -d "{\"cmd\":\"connect\",\"port\":\"fd:$FD\",\"protocol\":\"anocom\"}" \
    && echo \
    || echo "!! 自动连接失败：用界面「连接」或 POST /api/cmd 手动触发"
  echo "==> 浏览器打开 http://127.0.0.1:$PORT/"
  command -v termux-open-url >/dev/null 2>&1 && termux-open-url "http://127.0.0.1:$PORT/" || true
  wait $UV_PID
  exit 0
fi

# ---------------------------------------------------------------- 安装依赖
echo "==> 安装依赖（首次较慢）"
pkg update -y >/dev/null 2>&1 || true
pkg install -y python python-numpy termux-api curl >/dev/null 2>&1 || {
  echo "!! pkg 安装失败：检查网络，或先 termux-change-repo 换源"; exit 1;
}
python -m pip install --quiet --upgrade pip >/dev/null 2>&1 || true
python -m pip install --quiet pyserial fastapi 'uvicorn[standard]' >/dev/null 2>&1 || {
  echo "!! pip 安装失败：检查网络"; exit 1;
}

echo "==> 依赖自检"
python - <<'PY'
import importlib, sys
need = ['fastapi', 'uvicorn', 'serial', 'numpy']
print('    Python', sys.version.split()[0])
bad = []
for m in need:
    ok = importlib.util.find_spec(m) is not None
    print(f'    {m:10s}', 'OK' if ok else 'MISSING')
    if not ok:
        bad.append(m)
if bad:
    raise SystemExit(f'缺少：{bad}')
PY

# 平板会杀后台，长任务前加唤醒锁
command -v termux-wake-lock >/dev/null 2>&1 && termux-wake-lock || true

# ---------------------------------------------------------------- 按模式执行
case "$MODE" in
  --usb)
    echo "==> 可用 USB 设备："
    termux-usb -l || { echo "!! termux-usb 不可用（需要装 termux-api 并在系统里授予 USB 权限）"; exit 1; }
    echo
    if [ -z "${USB_DEV:-}" ]; then
      echo "请指定设备后重跑，例如："
      echo "    USB_DEV=/dev/bus/usb/001/002 bash $0 --usb"
      exit 0
    fi
    echo "==> 用 termux-usb 授权 $USB_DEV（系统会弹权限框，点允许）"
    exec termux-usb -r -e "$0" "$USB_DEV"
    ;;
  --run)
    cd "$GCS_DIR"
    echo "==> 启动后端：http://127.0.0.1:$PORT/"
    command -v termux-open-url >/dev/null 2>&1 && termux-open-url "http://127.0.0.1:$PORT/" || true
    exec python -m uvicorn server.main:app --host 127.0.0.1 --port "$PORT"
    ;;
  *)
    echo "==> 依赖就绪。用法："
    echo "    bash $0 --run     # 起服务"
    echo "    bash $0 --usb     # 接 OTG USB 数传"
    ;;
esac
