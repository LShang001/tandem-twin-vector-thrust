#!/usr/bin/env bash
# ============================================================================
#  构建 UniGNC USB-UDP 桥 APK（不依赖 Gradle）
#
#  为什么不用 Gradle：本机只装了 SDK 工具、没有 Gradle 发行版，而打这个
#  APK 只需要 aapt2 / javac / d8 / apksigner —— 全在 build-tools 里。
#  少一层构建系统，也就少一堆下载依赖与版本对齐问题。
#
#  依赖：
#    - JDK 17（javac / keytool / jar）
#    - Android SDK：build-tools（aapt2/d8/apksigner）+ platforms/android-34/android.jar
#    - usb-serial-for-android 源码（脚本会自动 clone 到 build/ 下）
#
#  用法：
#    bash build-apk.sh              # 构建，产物在 dist/unignc-usb-bridge.apk
#    bash build-apk.sh --clean      # 先清再构建
#
#  ★ 产物是 **debug 自签** APK（keystore 密码 android），够装平板测；正式发布
#    需要换成你自己的密钥并走 apksigner --v1/--v2 全签。
# ============================================================================
set -eo pipefail   # 不开 -u：MSYS 下 USER 等变量可能未定义
# ★ 禁止 MSYS 自动改写传给 Windows 程序的参数：它会把 U:/x 转成 /u/x，
#   aapt2 拿到 /u/x 会当成无效路径，报错却是"missing required flag -o"，极难定位。
export MSYS2_ARG_CONV_EXCL='*'

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ★ Windows 上的 aapt2 / javac / d8 处理不了含中文的路径（项目里 MinGW 也踩过同样的坑）。
#   做法：把 android 目录 subst 到一个空闲 ASCII 盘符，全部构建都用该盘符下的路径。
#   脚本退出时自动取消映射，不留残留。
ROOT_WIN="$(cygpath -w "$HERE" 2>/dev/null || echo "$HERE")"
DRIVE=""
case "$HERE" in
  *[![:print:]]*|*[^a-zA-Z0-9/._\ :-]*)      # 含非 ASCII 字符
    for L in U V W X Y Z Q R; do
      if [ ! -d "/$L" ] && [ ! -d "$L:/" ]; then
        case "$(subst 2>/dev/null | grep -i "^$L:" || true)" in "") DRIVE="$L:"; break ;; esac
      fi
    done
    ;;
esac
if [ -n "$DRIVE" ]; then
  subst "$DRIVE" "$ROOT_WIN" >/dev/null 2>&1 || { echo "!! subst $DRIVE 失败"; exit 1; }
  echo "==> 中文路径 → 映射 $DRIVE 到本项目 android 目录（构建结束自动取消）"
  trap 'subst "$DRIVE" /D >/dev/null 2>&1 || true' EXIT
  BASE="$DRIVE"
else
  BASE="$ROOT_WIN"
fi

PROJ="$BASE/usb-udp-bridge"
BUILD="$BASE/build"
DIST="$BASE/dist"
PKG="com.unignc.usbbridge"
APK_NAME="unignc-usb-bridge"

# ---- SDK 定位（Windows 的 LOCALAPPDATA 在 MSYS 里是常规路径） ----
SDK="${ANDROID_SDK_ROOT:-${ANDROID_HOME:-}}"
if [ -z "$SDK" ]; then
  for cand in "$LOCALAPPDATA/Android/Sdk" "$HOME/Android/Sdk" "/c/Users/${USERNAME:-12631}/AppData/Local/Android/Sdk"; do
    [ -d "$cand" ] && SDK="$cand" && break
  done
fi
[ -n "$SDK" ] && [ -d "$SDK" ] || { echo "!! 找不到 Android SDK：设 ANDROID_SDK_ROOT"; exit 1; }
# Windows 下这些环境变量是 C:\... 风格，MSYS 的 ls/find 不认，统一转成 /c/... 形式
# -m = 混合形式 C:/Users/...：MSYS 的 ls 认它，Windows 的 aapt2 也认它（-w 的反斜杠 MSYS 不认）
if command -v cygpath >/dev/null 2>&1; then SDK="$(cygpath -m "$SDK")"; fi

BT="$(ls -d "$SDK"/build-tools/* 2>/dev/null | sort -V | tail -1)"
PLATFORM="$(ls -d "$SDK"/platforms/android-* 2>/dev/null | sort -V | tail -1)"
JAR="$PLATFORM/android.jar"
for tool in aapt2 d8 apksigner; do
  [ -x "$BT/$tool" ] || [ -x "$BT/$tool.bat" ] || { echo "!! build-tools 缺 $tool（$BT）"; exit 1; }
done
[ -f "$JAR" ] || { echo "!! 缺 android.jar（$JAR）"; exit 1; }

# build-tools 在 Windows 上多是 .bat，MSYS 里直接调名字即可
AAPT2="$BT/aapt2"
D8="$BT/d8"
APKSIGNER="$BT/apksigner"

echo "==> SDK : $SDK"
echo "==> 工具: $BT"
echo "==> 平台: $PLATFORM"

if [ "${1:-}" = "--clean" ]; then rm -rf "$BUILD" "$DIST"; fi
mkdir -p "$BUILD" "$DIST"

# ---- 1) 取 usb-serial-for-android 源码（纯 Java，无资源依赖） ----
LIB="$BUILD/usb-serial-for-android"
if [ ! -d "$LIB" ]; then
  echo "==> 克隆 usb-serial-for-android"
  git clone --depth 1 --quiet https://github.com/mik3y/usb-serial-for-android "$LIB"
fi
LIB_SRC="$LIB/usbSerialForAndroid/src/main/java"
[ -d "$LIB_SRC" ] || { echo "!! 库源码路径不对：$LIB_SRC"; exit 1; }

# ---- 2) 资源：aapt2 compile → link（生成 R.java + 资源包） ----
echo "==> 编译资源"
rm -rf "$BUILD/res" "$BUILD/gen" "$BUILD/classes" "$BUILD/dex"
mkdir -p "$BUILD/res" "$BUILD/gen" "$BUILD/classes" "$BUILD/dex"
# aapt2 compile 只接受目录或单个文件，目录要求结构 res/<type>[-qualifier]/name.ext
"$AAPT2" compile --dir "$PROJ/res" -o "$BUILD/res/resources.zip"

echo "==> 链接资源 + 生成 R.java"
"$AAPT2" link \
  -o "$BUILD/app.ap_" \
  -I "$JAR" \
  --manifest "$PROJ/AndroidManifest.xml" \
  "$BUILD/res/resources.zip"
# 不生成 R.java：本 App 界面全部代码构建、不引用任何资源 id，
# 资源只有 manifest 里引用的 @xml/usb_device_filter（由 aapt2 直接打进包）

# ---- 3) javac：自己的代码 + 库源码 + R.java ----
echo "==> 编译 Java"
find "$LIB_SRC" -name '*.java' > "$BUILD/sources.txt"
# 补两个 Gradle 会自动生成、手动构建里缺失的桩：
#   androidx.annotation.IntDef（库用它做静态检查）
#   com.hoho.android.usbserial.BuildConfig（库引用了 DEBUG）
mkdir -p "$PROJ/../stubs/androidx/annotation" "$PROJ/../stubs/com/hoho/android/usbserial"
if [ ! -f "$PROJ/../stubs/androidx/annotation/IntDef.java" ]; then echo "!! 缺 IntDef 桩"; exit 1; fi
find "$PROJ/../stubs" -name '*.java' >> "$BUILD/sources.txt"
find "$PROJ/src" -name '*.java' >> "$BUILD/sources.txt"
: # 无 R.java
echo "    源文件 $(wc -l < "$BUILD/sources.txt") 个"
# core-lambda-stubs.jar 提供 LambdaMetafactory 等桩：Android 的 lambda 由 d8 desugar，
# 但 javac 编译期必须有这些符号，否则报「找不到 metafactory」。Gradle 会自动带上它。
LAMBDA_STUBS="$BT/core-lambda-stubs.jar"
BCP="$JAR"
[ -f "$LAMBDA_STUBS" ] && BCP="$JAR;$LAMBDA_STUBS"
javac -encoding UTF-8 -nowarn -source 8 -target 8 \
  -bootclasspath "$BCP" -classpath "$BCP" \
  -d "$BUILD/classes" @"$BUILD/sources.txt" 2>&1 | grep -v "bootstrap class path" || true
[ -d "$BUILD/classes/com/unignc" ] || { echo "!! 编译失败（没有产出类）"; exit 1; }

# ---- 4) d8 转 dex ----
echo "==> 转 dex"
find "$BUILD/classes" -name '*.class' > "$BUILD/classes.txt"
"$D8" --lib "$JAR" --min-api 21 --output "$BUILD/dex" @"$BUILD/classes.txt"

# ---- 5) 打包 ----
echo "==> 组装 APK"
OUT="$DIST/$APK_NAME.apk"
cp "$BUILD/app.ap_" "$OUT"
(cd "$BUILD/dex" && zip -q -X "$OUT" classes.dex)

# ---- 6) 签名（首次自动生成 debug keystore） ----
KS="$BUILD/debug.jks"
if [ ! -f "$KS" ]; then
  echo "==> 生成签名用 keystore"
  keytool -genkeypair -keystore "$KS" -alias unignc -keyalg RSA -keysize 2048 \
    -validity 10000 -storepass android -keypass android \
    -dname "CN=UniGNC GCS, OU=Dev, O=UniGNC, C=CN" >/dev/null 2>&1
fi
echo "==> 签名"
"$APKSIGNER" sign --ks "$KS" --ks-pass pass:android --key-pass pass:android \
  --v1-signing-enabled true --v2-signing-enabled true "$OUT"
"$APKSIGNER" verify "$OUT" >/dev/null && echo "==> 签名校验通过"

SIZE=$(stat -c %s "$OUT" 2>/dev/null || stat -f %z "$OUT")
echo
echo "==> 完成：$OUT（$((SIZE/1024)) KB）"
echo "    安装：adb install -r \"$OUT\""
echo "    用法：平板插数传 → 打开 App → 填地面站 IP 与 UDP 端口 → 开始转发"
