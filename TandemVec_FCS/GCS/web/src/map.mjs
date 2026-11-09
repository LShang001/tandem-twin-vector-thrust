/**
 * @file    map.mjs
 * @module  GCS/前端地图与航点
 * @brief   地图视图：航点编辑（点选/拖动/删除）+ 飞行器实时位置 + 航点表
 * @details 底图选型：**OpenStreetMap 瓦片**，不用高德/腾讯。
 *          理由：国内厂商瓦片是 GCJ-02 坐标系，与飞控的 WGS-84 差一个非线性
 *          偏移（几十米量级），图上标的点和实际飞的点对不上——对地面站来说
 *          这是致命错误，不是"看起来差点"。宁可底图慢一点也要坐标对。
 *
 *          离线策略：Leaflet 库文件放在 vendor/ 本地加载（无网也能起界面），
 *          只有瓦片需要网络。无网时地图是灰底，但航点编辑、飞行器位置、
 *          航点表全部照常工作——飞行现场经常没网，不能因为底图拉不到就
 *          整个页面不能用。
 *
 *          数据流：航点存在前端（waypoints 数组），暂不上传飞控；上传/下载
 *          走 MAVLink MISSION_ITEM_INT 是下一期。飞行器位置从 telemetry
 *          快照的 gps_lat/gps_lon 取，按 50Hz 刷新标记位置。
 * @theory  N/A（只做平面显示，不做坐标变换）
 * @frame   WGS-84 经纬度；航点高度为相对起飞点（AMSL 由飞控自行处理）
 * @units   经纬度 deg、高度 m、距离 m
 * @ref     GCS-001 §上位机、MAVLink MISSION_ITEM_INT（下一期）
 */

import { bus, state } from './main.mjs';

const $ = (id) => document.getElementById(id);

// 默认视角：长沙（项目常用试飞区域），无定位时也有个合理起点
const DEFAULT_CENTER = [28.2300, 112.9400];
const DEFAULT_ZOOM = 16;

const mapState = {
  map: null,
  waypoints: [],        // [{lat, lon, alt, marker}]
  vehicle: null,        // L.marker 飞行器位置
  trail: null,          // L.polyline 飞行轨迹
  trailPts: [],
  activated: false,
  tileOk: true,
  coordEl: null,
};

/** 生成航点图标（带序号，避免用默认图钉分不清顺序） */
function wpIcon(L, idx) {
  return L.divIcon({
    className: 'wp-marker',
    html: `<div class="wp-dot">${idx + 1}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13],
  });
}

function refreshMarkers() {
  const L = window.L;
  mapState.waypoints.forEach((w, i) => {
    if (w.marker) {
      w.marker.setIcon(wpIcon(L, i));
    }
  });
  renderTable();
}

function addWaypoint(lat, lon, alt = 50) {
  const L = window.L;
  const marker = L.marker([lat, lon], {
    icon: wpIcon(L, mapState.waypoints.length),
    draggable: true,
  }).addTo(mapState.map);

  const wp = { lat, lon, alt, marker };
  marker.on('dragend', () => {
    const p = marker.getLatLng();
    wp.lat = p.lat;
    wp.lon = p.lng;
    renderTable();
  });
  marker.on('contextmenu', () => {
    // 右键删除：比"先选中再点删除"少一步，现场操作更快
    removeWaypoint(wp);
  });

  mapState.waypoints.push(wp);
  refreshMarkers();
  drawRoute();
}

function removeWaypoint(wp) {
  const i = mapState.waypoints.indexOf(wp);
  if (i < 0) return;
  mapState.map.removeLayer(wp.marker);
  mapState.waypoints.splice(i, 1);
  refreshMarkers();
  drawRoute();
}

/** 航点连线：直观看出任务航线和顺序 */
function drawRoute() {
  const L = window.L;
  if (mapState.route) mapState.map.removeLayer(mapState.route);
  if (mapState.waypoints.length < 2) { mapState.route = null; return; }
  const pts = mapState.waypoints.map((w) => [w.lat, w.lon]);
  mapState.route = L.polyline(pts, {
    color: '#3ea6ff', weight: 2, opacity: 0.85, dashArray: '6 4',
  }).addTo(mapState.map);
}

function renderTable() {
  const body = $('mapWpRows');
  const stat = $('mapWpStat');
  if (stat) stat.textContent = `${mapState.waypoints.length} 个航点`;
  if (!body) return;
  if (!mapState.waypoints.length) {
    body.innerHTML = '<tr><td colspan="5" class="insp-empty">在地图上点击即可添加航点</td></tr>';
    return;
  }
  body.innerHTML = mapState.waypoints.map((w, i) => `
    <tr>
      <td class="num">${i + 1}</td>
      <td class="num">${w.lat.toFixed(6)}</td>
      <td class="num">${w.lon.toFixed(6)}</td>
      <td class="num"><input class="wp-alt" type="number" data-i="${i}" value="${w.alt}" step="5" min="0"></td>
      <td><button class="btn small wp-del" data-i="${i}">删</button></td>
    </tr>`).join('');

  body.querySelectorAll('.wp-alt').forEach((el) => {
    el.addEventListener('change', () => {
      const i = Number(el.dataset.i);
      const v = Number(el.value);
      if (Number.isFinite(v) && mapState.waypoints[i]) mapState.waypoints[i].alt = v;
    });
  });
  body.querySelectorAll('.wp-del').forEach((el) => {
    el.addEventListener('click', () => {
      const i = Number(el.dataset.i);
      if (mapState.waypoints[i]) removeWaypoint(mapState.waypoints[i]);
    });
  });
}

function onTelemetry(s) {
  const L = window.L;
  if (!mapState.map) return;
  const lat = Number(s.gps_lat);
  const lon = Number(s.gps_lon);
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || (lat === 0 && lon === 0)) return;

  if (!mapState.vehicle) {
    mapState.vehicle = L.circleMarker([lat, lon], {
      radius: 6, color: '#29d3a2', weight: 2, fillColor: '#29d3a2', fillOpacity: 0.9,
    }).addTo(mapState.map);
  } else {
    mapState.vehicle.setLatLng([lat, lon]);
  }

  mapState.trailPts.push([lat, lon]);
  if (mapState.trailPts.length > 2000) mapState.trailPts.shift();
  if (!mapState.trail) {
    mapState.trail = L.polyline(mapState.trailPts, {
      color: '#29d3a2', weight: 1.6, opacity: 0.65,
    }).addTo(mapState.map);
  } else {
    mapState.trail.setLatLngs(mapState.trailPts);
  }
}

export function activate() {
  if (mapState.activated) {
    // 页面从隐藏变可见时 Leaflet 需要重算尺寸，否则瓦片错位
    setTimeout(() => mapState.map && mapState.map.invalidateSize(), 60);
    return;
  }
  mapState.activated = true;

  const L = window.L;
  if (!L) {
    const box = $('mapCanvas');
    if (box) box.innerHTML = '<div class="insp-empty">Leaflet 未加载（检查 vendor/leaflet-1.9.4）</div>';
    return;
  }

  const map = L.map('mapCanvas', {
    center: DEFAULT_CENTER,
    zoom: DEFAULT_ZOOM,
    zoomControl: true,
    attributionControl: true,
  });
  mapState.map = map;

  // 瓦片走后端代理：/tiles/{z}/{x}/{y}.png。
  // 后端缓存优先、未命中再向瓦片源取，用过的区域落盘后可离线复用；
  // 前端直连瓦片源则无法缓存到本机，野外断网就彻底没有底图。
  const tiles = L.tileLayer('/tiles/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors &copy; CARTO',
  });
  tiles.on('tileerror', () => {
    // 无网/被拦时只提示一次，不要刷屏；航点编辑与飞行器位置仍然可用
    if (!mapState.tileOk) return;
    mapState.tileOk = false;
    const hint = $('mapTileHint');
    if (hint) {
      hint.textContent = '底图瓦片未取到（本地无缓存且瓦片源不可达）——航点编辑与飞行器位置仍可用';
      hint.style.color = '#ffb547';
    }
  });
  tiles.addTo(map);

  map.on('click', (e) => addWaypoint(e.latlng.lat, e.latlng.lng));

  // 光标经纬度：编辑航点时看得到精确坐标
  map.on('mousemove', (e) => {
    if (mapState.coordEl) {
      mapState.coordEl.textContent =
        `${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`;
    }
  });
  mapState.coordEl = $('mapCoord');

  const clr = $('mapClearWp');
  if (clr) {
    clr.addEventListener('click', () => {
      [...mapState.waypoints].forEach(removeWaypoint);
    });
  }
  const center = $('mapCenterVehicle');
  if (center) {
    center.addEventListener('click', () => {
      const p = mapState.vehicle && mapState.vehicle.getLatLng();
      if (p) map.setView(p, map.getZoom());
    });
  }

  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail || {}));
  window.addEventListener('resize', () => map.invalidateSize());

  renderTable();
  drawRoute();
  setTimeout(() => map.invalidateSize(), 80);
}
