/**
 * @file    video.mjs
 * @module  GCS/前端 视频流
 * @brief   多源视频：MJPEG / HLS / 本机摄像头，可切换、可记住
 * @details 三种源各有硬约束，这里按可靠性排序：
 *
 *          ① **MJPEG**（multipart/x-mixed-replace）——浏览器 `<img src>` 原生
 *             就能播，不需要任何解码库。图传设备、树莓派摄像头、ffmpeg 转出的
 *             mjpeg 都走这条。**首选**，因为依赖最少、坏得最少。
 *
 *          ② **HLS**（.m3u8）——需要 hls.js（Safari 原生支持）。适合 RTSP 经
 *             ffmpeg 转码后的场景。hls.js 已放本地 vendor，无网也能加载。
 *
 *          ③ **本机摄像头**（getUserMedia）——不需要任何服务端，用于把手机/
 *             平板的摄像头当观察视角。注意它要求 secure context
 *             （https 或 localhost），局域网 http 下会被浏览器直接拒绝。
 *
 *          **RTSP 不能直接播**：浏览器不解 RTSP，必须先转码成 HLS 或 MJPEG。
 *          这一条是浏览器的硬限制，不是实现取舍——页面上要把这句话写给用户看，
 *          否则会以为"填个 rtsp:// 就能播"。
 *
 *          源地址记在 localStorage：这是显示偏好，与布局同理。
 * @theory  N/A
 * @frame   N/A
 * @units   fps / 分辨率
 * @note    MJPEG 的 <img> 元素在断流时会静默保持最后一帧——必须用 onerror
 *          与超时检测来暴露"实际已断"的事实，否则看起来像画面卡住。
 * @ref     GCS-001 §上位机、RFC 2046 multipart/x-mixed-replace
 */

import { bus } from './main.mjs';

const $ = (id) => document.getElementById(id);
const STORE_KEY = 'unignc.gcs.video.v1';

const video = {
  mode: 'mjpeg',
  url: '',
  hls: null,
  stream: null,
  activated: false,
  lastFrameAt: 0,
  watchdog: null,
};

function savePrefs() {
  try {
    localStorage.setItem(STORE_KEY, JSON.stringify({
      mode: video.mode, url: video.url,
    }));
  } catch (e) { /* 忽略 */ }
}

function loadPrefs() {
  try {
    const raw = localStorage.getItem(STORE_KEY);
    if (!raw) return;
    const p = JSON.parse(raw);
    if (p.mode) video.mode = p.mode;
    if (p.url) video.url = p.url;
  } catch (e) { /* 坏数据当没存过 */ }
}

function setStatus(text, cls = '') {
  const el = $('videoStatus');
  if (!el) return;
  el.textContent = text;
  el.className = `video-status ${cls}`;
}

function stopAll() {
  if (video.hls) {
    try { video.hls.destroy(); } catch (e) { /* 忽略 */ }
    video.hls = null;
  }
  if (video.stream) {
    try { video.stream.getTracks().forEach((t) => t.stop()); } catch (e) { /* 忽略 */ }
    video.stream = null;
  }
  if (video.watchdog) {
    clearInterval(video.watchdog);
    video.watchdog = null;
  }
  const img = $('videoImg');
  if (img) { img.removeAttribute('src'); img.style.display = 'none'; }
  const vid = $('videoEl');
  if (vid) {
    try { vid.pause(); } catch (e) { /* 忽略 */ }
    vid.removeAttribute('src');
    vid.style.display = 'none';
  }
}

/** MJPEG：靠 <img> 的 onload 逐帧计数判断是否真的在动 */
function startMjpeg(url) {
  const img = $('videoImg');
  if (!img) return;
  img.style.display = 'block';
  video.mjpegError = false;
  img.onerror = () => {
    video.mjpegError = true;
    setStatus('MJPEG 流断开（源不可达或不是 multipart 流）', 'bad');
  };
  img.src = url;
  setStatus('MJPEG 正在打开…', '');

  // 判活方式：**不能靠 onload 计数**——multipart/x-mixed-replace 流在浏览器里
  // 只在首帧触发一次 onload，之后换帧不会再次触发，用它判断会误报"无新帧"
  // （实测踩过）。正确判据是：没有触发 onerror 且有 naturalWidth，
  // 就说明流还活着；真的断了浏览器会触发 onerror。
  video.watchdog = setInterval(() => {
    if (video.mjpegError) return;
    if (img.naturalWidth === 0) {
      setStatus('MJPEG 无画面（源可能不是 multipart/x-mixed-replace）', 'bad');
      return;
    }
    setStatus(`MJPEG 播放中 · ${img.naturalWidth}×${img.naturalHeight}`, 'ok');
  }, 1500);
}

function startHls(url) {
  const vid = $('videoEl');
  if (!vid) return;
  vid.style.display = 'block';

  if (vid.canPlayType('application/vnd.apple.mpegurl')) {
    // Safari 原生支持 HLS
    vid.src = url;
    vid.play().catch(() => setStatus('自动播放被拦截，请点画面', 'warn'));
    setStatus('HLS（原生）播放中…', 'ok');
    return;
  }
  const Hls = window.Hls;
  if (!Hls || !Hls.isSupported()) {
    setStatus('HLS 不可用：hls.js 未加载且浏览器不支持原生 HLS', 'bad');
    return;
  }
  video.hls = new Hls({ lowLatencyMode: true });
  video.hls.loadSource(url);
  video.hls.attachMedia(vid);
  video.hls.on(Hls.Events.ERROR, (_e, data) => {
    if (data && data.fatal) setStatus(`HLS 致命错误：${data.type}`, 'bad');
  });
  video.hls.on(Hls.Events.MANIFEST_PARSED, () => {
    vid.play().catch(() => setStatus('自动播放被拦截，请点画面', 'warn'));
    setStatus('HLS 播放中…', 'ok');
  });
}

async function startCamera() {
  const vid = $('videoEl');
  if (!vid) return;
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    setStatus('本机摄像头不可用：需要 https 或 localhost（secure context）', 'bad');
    return;
  }
  try {
    video.stream = await navigator.mediaDevices.getUserMedia({
      video: { width: { ideal: 1280 }, height: { ideal: 720 } }, audio: false,
    });
    vid.style.display = 'block';
    vid.srcObject = video.stream;
    await vid.play();
    const track = video.stream.getVideoTracks()[0];
    const s = track ? track.getSettings() : {};
    setStatus(`摄像头 ${s.width || '?'}×${s.height || '?'}`, 'ok');
  } catch (e) {
    setStatus(`摄像头打开失败：${(e && e.message) || e}`, 'bad');
  }
}

function start() {
  stopAll();
  const sel = $('videoMode');
  const input = $('videoUrl');
  video.mode = sel ? sel.value : 'mjpeg';
  video.url = input ? input.value.trim() : '';
  savePrefs();

  if (video.mode === 'camera') { startCamera(); return; }
  if (!video.url) { setStatus('请填写视频源地址', 'warn'); return; }
  if (video.mode === 'mjpeg') startMjpeg(video.url);
  else if (video.mode === 'hls') startHls(video.url);
}

export function activate() {
  if (video.activated) return;
  video.activated = true;

  loadPrefs();
  const sel = $('videoMode');
  const input = $('videoUrl');
  if (sel) sel.value = video.mode;
  if (input) input.value = video.url;

  const btn = $('videoStart');
  if (btn) btn.addEventListener('click', start);
  const stop = $('videoStop');
  if (stop) {
    stop.addEventListener('click', () => { stopAll(); setStatus('已停止', ''); });
  }
  const test = $('videoTest');
  if (test) {
    test.addEventListener('click', () => {
      if (sel) sel.value = 'mjpeg';
      if (input) input.value = '/video/test.mjpg';
      video.mode = 'mjpeg'; video.url = '/video/test.mjpg';
      savePrefs();
      start();
    });
  }
  if (sel) {
    sel.addEventListener('change', () => {
      const isCam = sel.value === 'camera';
      if (input) {
        input.disabled = isCam;
        input.placeholder = isCam ? '（本机摄像头无需地址）'
          : (sel.value === 'hls' ? 'http://…/stream.m3u8' : 'http://…/stream.mjpg');
      }
    });
  }

  window.addEventListener('beforeunload', stopAll);
  setStatus('未启动');
}
