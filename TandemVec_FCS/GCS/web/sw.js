/**
 * @file    sw.js
 * @module  GCS/前端 Service Worker
 * @brief   PWA 资源缓存，让地面站装到主屏幕后仍能离线打开界面
 * @details 策略选择：**network-first**，不是 cache-first。
 *          原因：这套界面还在高频迭代，cache-first 会让用户拿到几天前的旧页面
 *          却以为是新版本（后端 hello 握手能发现版本不一致，但页面本身仍是旧的，
 *          反而更难排查）。network-first 保证有网时永远拿最新，断网才回落缓存。
 *
 *          注意：Service Worker 只在 secure context 生效——即 https:// 或
 *          localhost。局域网 http://192.168.x.x:8091 不是 secure context，
 *          浏览器会直接拒绝注册。此时「添加到主屏幕」仍可用（靠 manifest +
 *          display:standalone 生成独立窗口），只是没有离线缓存能力。
 *          要在局域网里获得完整 PWA（含离线），需要给后端加 HTTPS 或走反向代理。
 *
 * @theory  N/A
 * @frame   N/A
 * @units   N/A
 * @note    缓存名单变化时要提升 CACHE 版本号，否则旧缓存不会清理。
 * @test    有网：断网前加载一次 → 断网刷新仍能看到界面；改文件后刷新应拿到新内容
 * @see     web/manifest.webmanifest、web/index.html 的注册代码
 */

const CACHE = 'unignc-gcs-v1';

// 只预缓存外壳。业务数据（/ws、/api）一律不进缓存——
// 把实时遥测缓存下来会导致「断网后仍显示旧数据」，比白屏更危险。
const SHELL = [
  './',
  './index.html',
  './css/style.css',
  './src/main.mjs',
  './src/widget.mjs',
  './src/dashboard.mjs',
  './src/view3d.mjs',
  './src/scope.mjs',
  './src/gnss.mjs',
  './src/inspector.mjs',
  './src/blackbox.mjs',
  './src/params.mjs',
  './src/console.mjs',
  './manifest.webmanifest',
  './icon-192.png',
  './icon-512.png',
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE)
      .then((c) => c.addAll(SHELL))
      .catch(() => {
        // 单个资源 404 不应让整个安装失败（否则永远装不上）
      })
      .then(() => self.skipWaiting()),
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)),
      ))
      .then(() => self.clients.claim()),
  );
});

self.addEventListener('fetch', (event) => {
  const req = event.request;
  if (req.method !== 'GET') return;

  const url = new URL(req.url);
  // 实时链路与状态接口不缓存
  if (url.pathname.startsWith('/ws') || url.pathname.startsWith('/api')) return;

  event.respondWith(
    fetch(req)
      .then((resp) => {
        // 只缓存同源成功响应
        if (resp.ok && url.origin === self.location.origin) {
          const copy = resp.clone();
          caches.open(CACHE).then((c) => c.put(req, copy)).catch(() => {});
        }
        return resp;
      })
      .catch(() => caches.match(req).then((hit) => hit || caches.match('./index.html'))),
  );
});
