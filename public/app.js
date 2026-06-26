let MAPBOX_TOKEN = '';
let APP_CONFIG = {};

const COLORS = {
  lane: '#44ff88',
  connector: '#4488ff',
  center: '#ffcc00',
  eraser: '#ff8800'
};

const FIX_COLORS = {
  4: '#44ff44',  // RTK Fix — green
  5: '#ffcc00',  // RTK Float — yellow
  2: '#ff8800',  // DGPS — orange
  1: '#ff4444',  // GPS — red
  0: '#888888',  // No fix — gray
};

const GPS_TRAIL_MAX = 3000;
const SNAP_DISTANCE_PX = 20;
const CLIENT_ID_KEY = 'followrtk-client-id';
const COLLABORATOR_NAME_KEY = 'followrtk-collaborator-name';
const PASSWORD_KEY = 'followrtk-password';

let map;
let annotations = [];
let connectors = [];
let manualCenterLines = [];
let suppressedAutoCenterLineIds = [];
let eraserPoints = [];
let activeAnnotationId = null;
let activeConnectorId = null;
let activeManualCenterLineId = null;
let selectedAutoCenterLineId = null;
let currentMode = 'lane';
let pointMarkers = [];
let connectorMarkers = [];
let manualCenterLineMarkers = [];
let eraserMarkers = [];
let drawingLabelMarkers = [];
let lineCounter = 0;
let connectorCounter = 0;
let manualCenterLineCounter = 0;
let autoCenterLineRecords = new Map();
let eraserRadius = 8;

// GPS state
let gpsMarker = null;
let gpsTrail = [];
let gpsFollowing = true;
let gpsWs = null;
let gpsHzCounter = 0;
let gpsHzValue = 0;
let gpsHzInterval = null;
let gpsPointCount = 0;

// Collaboration state
let clientId = getClientId();
let collaboratorName = localStorage.getItem(COLLABORATOR_NAME_KEY) || '';
let appPassword = sessionStorage.getItem(PASSWORD_KEY) || '';
let supabaseClient = null;
let realtimeChannel = null;

function getClientId() {
  let id = localStorage.getItem(CLIENT_ID_KEY);
  if (!id) {
    id = crypto.randomUUID
      ? crypto.randomUUID()
      : `client-${Date.now()}-${Math.random().toString(16).slice(2)}`;
    localStorage.setItem(CLIENT_ID_KEY, id);
  }
  return id;
}

function setSyncStatus(message, state = 'online') {
  const el = document.getElementById('sync-status');
  if (!el) return;
  el.textContent = message;
  el.className = `sync-status ${state === 'online' ? '' : state}`.trim();
}

function syncHeaders() {
  return {
    'Content-Type': 'application/json',
    'X-FollowRTK-Client-Id': clientId,
    'X-FollowRTK-Password': appPassword
  };
}

function authHeaders() {
  return {
    'X-FollowRTK-Client-Id': clientId,
    'X-FollowRTK-Password': appPassword
  };
}

function showJoinOverlay(message = '') {
  const overlay = document.getElementById('join-overlay');
  const nameInput = document.getElementById('collaborator-name');
  const passwordInput = document.getElementById('site-password');
  const errorEl = document.getElementById('join-error');
  if (!overlay) return;
  nameInput.value = collaboratorName;
  passwordInput.value = appPassword;
  errorEl.textContent = message;
  overlay.classList.remove('hidden');
}

function hideJoinOverlay() {
  const overlay = document.getElementById('join-overlay');
  if (overlay) overlay.classList.add('hidden');
}

function renderCollaborators(presenceState = {}) {
  const list = document.getElementById('collaborator-list');
  if (!list) return;
  const people = Object.values(presenceState)
    .flat()
    .map(p => p.name)
    .filter(Boolean);
  const uniquePeople = [...new Set(people)];
  if (uniquePeople.length === 0) {
    list.innerHTML = '<p class="hint">No collaborators online.</p>';
    return;
  }
  list.innerHTML = uniquePeople
    .map(name => `<span class="collaborator-chip">${name}</span>`)
    .join('');
}

function touchAnnotation(annotation) {
  if (!annotation) return;
  annotation.updatedBy = collaboratorName;
  annotation.updatedAt = new Date().toISOString();
  if (annotation.id === activeAnnotationId) annotation.activeBy = collaboratorName;
}

function touchConnector(connector) {
  if (!connector) return;
  connector.updatedBy = collaboratorName;
  connector.updatedAt = new Date().toISOString();
  if (connector.id === activeConnectorId) connector.activeBy = collaboratorName;
}

function touchManualCenterLine(line) {
  if (!line) return;
  line.updatedBy = collaboratorName;
  line.updatedAt = new Date().toISOString();
  if (line.id === activeManualCenterLineId) line.activeBy = collaboratorName;
}

function clearActiveAnnotation() {
  if (!activeAnnotationId) return null;
  const ann = annotations.find(a => a.id === activeAnnotationId);
  if (ann) {
    ann.updatedBy = collaboratorName;
    ann.updatedAt = new Date().toISOString();
    ann.activeBy = '';
  }
  activeAnnotationId = null;
  return ann;
}

function clearActiveConnector() {
  if (!activeConnectorId) return null;
  const conn = connectors.find(c => c.id === activeConnectorId);
  if (conn) {
    conn.updatedBy = collaboratorName;
    conn.updatedAt = new Date().toISOString();
    conn.activeBy = '';
  }
  activeConnectorId = null;
  return conn;
}

function clearActiveManualCenterLine() {
  if (!activeManualCenterLineId) return null;
  const line = manualCenterLines.find(c => c.id === activeManualCenterLineId);
  if (line) {
    line.updatedBy = collaboratorName;
    line.updatedAt = new Date().toISOString();
    line.activeBy = '';
  }
  activeManualCenterLineId = null;
  return line;
}

function createRouteDocument() {
  return {
    annotations: annotations.map(ann => ({
      id: ann.id,
      name: ann.name,
      type: ann.type,
      points: ann.points.map(p => ({ lat: p.lat, lng: p.lng })),
      createdBy: ann.createdBy || '',
      updatedBy: ann.updatedBy || '',
      updatedAt: ann.updatedAt || '',
      activeBy: ann.activeBy || ''
    })),
    connectors: connectors.map(conn => ({
      id: conn.id,
      name: conn.name,
      type: conn.type,
      points: conn.points.map(p => ({ lat: p.lat, lng: p.lng })),
      createdBy: conn.createdBy || '',
      updatedBy: conn.updatedBy || '',
      updatedAt: conn.updatedAt || '',
      activeBy: conn.activeBy || ''
    })),
    manualCenterLines: manualCenterLines.map(line => ({
      id: line.id,
      name: line.name,
      type: 'manual-centerline',
      points: line.points.map(p => ({ lat: p.lat, lng: p.lng })),
      createdBy: line.createdBy || '',
      updatedBy: line.updatedBy || '',
      updatedAt: line.updatedAt || '',
      activeBy: line.activeBy || ''
    })),
    suppressedAutoCenterLineIds: [...new Set(suppressedAutoCenterLineIds)],
    eraserPoints: eraserPoints.map(ep => ({
      id: ep.id,
      lat: ep.lat,
      lng: ep.lng,
      radius: ep.radius,
      createdBy: ep.createdBy || ''
    })),
    metadata: {
      savedAt: new Date().toISOString(),
      campus: 'Stanford University',
      project: 'FollowRTK Self-Driving Golf Cart',
      updatedBy: collaboratorName
    }
  };
}

// --- Auto-save ---

let autoSaveTimer = null;

function scheduleAutoSave() {
  if (autoSaveTimer) clearTimeout(autoSaveTimer);
  autoSaveTimer = setTimeout(doAutoSave, 500);
}

async function doAutoSave() {
  const data = createRouteDocument();
  try {
    await fetch('/api/save', {
      method: 'POST',
      headers: syncHeaders(),
      body: JSON.stringify(data)
    });
  } catch (e) {}
}

function initRealtimeSync() {
  if (!APP_CONFIG.supabaseUrl || !APP_CONFIG.supabaseAnonKey) {
    setSyncStatus(APP_CONFIG.storageMode === 'supabase' ? 'Realtime not configured' : 'Local mode', 'offline');
    return;
  }

  if (!window.supabase) {
    setSyncStatus('Realtime library unavailable', 'error');
    return;
  }

  supabaseClient = window.supabase.createClient(APP_CONFIG.supabaseUrl, APP_CONFIG.supabaseAnonKey);
  realtimeChannel = supabaseClient
    .channel(`route-document-${APP_CONFIG.routeDocumentId}`)
    .on(
      'postgres_changes',
      {
        event: '*',
        schema: 'public',
        table: 'route_documents',
        filter: `id=eq.${APP_CONFIG.routeDocumentId}`
      },
      async (payload) => {
        if (payload.new?.updated_by === clientId) return;
        await loadAnnotations({ remote: true });
      }
    )
    .on('presence', { event: 'sync' }, () => {
      renderCollaborators(realtimeChannel.presenceState());
    })
    .subscribe((status) => {
      if (status === 'SUBSCRIBED') {
        setSyncStatus('Live sync connected');
        realtimeChannel.track({
          clientId,
          name: collaboratorName || 'Anonymous',
          joinedAt: new Date().toISOString()
        });
      } else if (status === 'CHANNEL_ERROR' || status === 'TIMED_OUT') {
        setSyncStatus('Live sync disconnected', 'error');
      }
    });
}

// --- Geo math ---

function toRad(degrees) {
  return degrees * Math.PI / 180;
}

function haversineMeters(a, b) {
  const R = 6371000;
  const dLat = toRad(b.lat - a.lat);
  const dLng = toRad(b.lng - a.lng);
  const sinLat = Math.sin(dLat / 2);
  const sinLng = Math.sin(dLng / 2);
  const h = sinLat * sinLat + Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * sinLng * sinLng;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

function resampleLine(points, numSamples) {
  if (points.length < 2 || numSamples < 2) return points.slice();
  const dists = [0];
  for (let i = 1; i < points.length; i++) {
    dists.push(dists[i - 1] + haversineMeters(points[i - 1], points[i]));
  }
  const totalDist = dists[dists.length - 1];
  if (totalDist === 0) return points.slice();

  const result = [];
  for (let s = 0; s < numSamples; s++) {
    const target = (s / (numSamples - 1)) * totalDist;
    let seg = 0;
    while (seg < dists.length - 2 && dists[seg + 1] < target) seg++;
    const segLen = dists[seg + 1] - dists[seg];
    const t = segLen > 0 ? (target - dists[seg]) / segLen : 0;
    result.push({
      lat: points[seg].lat + t * (points[seg + 1].lat - points[seg].lat),
      lng: points[seg].lng + t * (points[seg + 1].lng - points[seg].lng)
    });
  }
  return result;
}

function orientedLinePair(line1, line2) {
  const dSame = haversineMeters(line1[0], line2[0])
    + haversineMeters(line1[line1.length - 1], line2[line2.length - 1]);
  const dRev = haversineMeters(line1[0], line2[line2.length - 1])
    + haversineMeters(line1[line1.length - 1], line2[0]);
  return [line1, dRev < dSame ? [...line2].reverse() : line2];
}

function toLocalXY(point, refLat) {
  return {
    x: point.lng * Math.cos(toRad(refLat)) * 111320,
    y: point.lat * 110540
  };
}

function fromLocalXY(point, refLat) {
  return {
    lat: point.y / 110540,
    lng: point.x / (Math.cos(toRad(refLat)) * 111320)
  };
}

function closestPointOnPolyline(point, line, minAlong = -Infinity) {
  if (line.length < 2) return null;

  const refLat = point.lat;
  const p = toLocalXY(point, refLat);
  let alongBefore = 0;
  let best = null;

  for (let i = 0; i < line.length - 1; i++) {
    const aGeo = line[i];
    const bGeo = line[i + 1];
    const a = toLocalXY(aGeo, refLat);
    const b = toLocalXY(bGeo, refLat);
    const vx = b.x - a.x;
    const vy = b.y - a.y;
    const lenSq = vx * vx + vy * vy;
    const segLen = Math.sqrt(lenSq);
    if (segLen === 0) continue;

    const rawT = ((p.x - a.x) * vx + (p.y - a.y) * vy) / lenSq;
    const t = Math.max(0, Math.min(1, rawT));
    const along = alongBefore + segLen * t;
    alongBefore += segLen;
    if (along < minAlong) continue;

    const projected = {
      x: a.x + vx * t,
      y: a.y + vy * t
    };
    const dx = p.x - projected.x;
    const dy = p.y - projected.y;
    const distance = Math.sqrt(dx * dx + dy * dy);

    if (!best || distance < best.distance) {
      best = {
        point: fromLocalXY(projected, refLat),
        distance,
        along
      };
    }
  }

  return best;
}

function smoothCenterLine(points) {
  if (points.length < 5) return points;
  return points.map((point, idx) => {
    if (idx === 0 || idx === points.length - 1) return point;
    const prev = points[idx - 1];
    const next = points[idx + 1];
    return {
      lat: prev.lat * 0.25 + point.lat * 0.5 + next.lat * 0.25,
      lng: prev.lng * 0.25 + point.lng * 0.5 + next.lng * 0.25
    };
  });
}

function computeCenterLine(line1, line2) {
  const [l1, l2] = orientedLinePair(line1, line2);
  const len1 = lineLengthMeters(l1);
  const len2 = lineLengthMeters(l2);
  const driver = len1 >= len2 ? l1 : l2;
  const target = len1 >= len2 ? l2 : l1;
  const sampleCount = Math.max(driver.length, target.length, Math.min(80, Math.max(24, Math.ceil(Math.max(len1, len2) / 2.5))));
  const samples = resampleLine(driver, sampleCount);

  const center = [];
  let minAlong = -Infinity;
  for (const sample of samples) {
    const match = closestPointOnPolyline(sample, target, minAlong);
    if (!match || match.distance > 35) continue;
    minAlong = Math.max(minAlong, match.along - 0.5);
    center.push({
      lat: (sample.lat + match.point.lat) / 2,
      lng: (sample.lng + match.point.lng) / 2
    });
  }

  if (center.length >= 2) return smoothCenterLine(center);

  const n = Math.max(l1.length, l2.length, 20);
  const a = resampleLine(l1, n);
  const b = resampleLine(l2, n);
  return smoothCenterLine(a.map((p, i) => ({
    lat: (p.lat + b[i].lat) / 2,
    lng: (p.lng + b[i].lng) / 2
  })));
}

function lineLengthMeters(points) {
  let total = 0;
  for (let i = 1; i < points.length; i++) {
    total += haversineMeters(points[i - 1], points[i]);
  }
  return total;
}

function pairMetrics(itemA, itemB) {
  if (itemA.points.length < 2 || itemB.points.length < 2) return null;

  const dSame = haversineMeters(itemA.points[0], itemB.points[0])
    + haversineMeters(itemA.points[itemA.points.length - 1], itemB.points[itemB.points.length - 1]);
  const dRev = haversineMeters(itemA.points[0], itemB.points[itemB.points.length - 1])
    + haversineMeters(itemA.points[itemA.points.length - 1], itemB.points[0]);
  const pointsB = dRev < dSame ? [...itemB.points].reverse() : itemB.points;

  const n = Math.max(itemA.points.length, pointsB.length, 20);
  const a = resampleLine(itemA.points, n);
  const b = resampleLine(pointsB, n);
  const distances = a.map((pt, i) => haversineMeters(pt, b[i]));
  const avgDistance = distances.reduce((sum, d) => sum + d, 0) / distances.length;
  const maxDistance = Math.max(...distances);
  const lengthA = lineLengthMeters(itemA.points);
  const lengthB = lineLengthMeters(itemB.points);
  const lengthRatio = Math.max(lengthA, lengthB) / Math.max(1, Math.min(lengthA, lengthB));

  return {
    avgDistance,
    maxDistance,
    lengthRatio,
    score: avgDistance + maxDistance * 0.3 + Math.abs(lengthA - lengthB) * 0.15
  };
}

function findGeometricPairs(items, options) {
  const candidates = [];
  for (let i = 0; i < items.length; i++) {
    for (let j = i + 1; j < items.length; j++) {
      const metrics = pairMetrics(items[i], items[j]);
      if (!metrics) continue;
      if (
        metrics.avgDistance <= options.maxAvgDistance
        && metrics.maxDistance <= options.maxPointDistance
        && metrics.lengthRatio <= options.maxLengthRatio
      ) {
        candidates.push({ a: items[i], b: items[j], metrics });
      }
    }
  }

  candidates.sort((left, right) => left.metrics.score - right.metrics.score);

  const used = new Set();
  const pairs = [];
  for (const candidate of candidates) {
    if (used.has(candidate.a.id) || used.has(candidate.b.id)) continue;
    used.add(candidate.a.id);
    used.add(candidate.b.id);
    pairs.push({
      name: candidate.a.name === candidate.b.name
        ? candidate.a.name
        : `${candidate.a.name} / ${candidate.b.name}`,
      a: candidate.a,
      b: candidate.b
    });
  }
  return pairs;
}

function applyCenterLineErasure(centerPoints, erasers, defaultRadius) {
  const segments = [];
  let current = [];
  for (const pt of centerPoints) {
    let erased = false;
    for (const ep of erasers) {
      if (haversineMeters(pt, ep) < (ep.radius || defaultRadius)) {
        erased = true;
        break;
      }
    }
    if (erased) {
      if (current.length >= 2) segments.push(current);
      current = [];
    } else {
      current.push(pt);
    }
  }
  if (current.length >= 2) segments.push(current);
  return segments;
}

// --- Pair detection: pair nearby compatible boundaries ---

function findPairs() {
  return findGeometricPairs(
    annotations.filter(ann => ann.type === 'lane' && ann.points.length >= 2),
    {
      maxAvgDistance: 24,
      maxPointDistance: 45,
      maxLengthRatio: 2.75
    }
  );
}

function getAutoCenterLineId(pair) {
  return [pair.a.id, pair.b.id].sort().join('__');
}

function suppressAutoCenterLine(id) {
  if (!suppressedAutoCenterLineIds.includes(id)) {
    suppressedAutoCenterLineIds.push(id);
  }
  if (selectedAutoCenterLineId === id) selectedAutoCenterLineId = null;
}

// --- Snap to nearest lane point ---

function snapToLanePoint(clickLngLat) {
  let bestDist = Infinity;
  let bestPoint = null;

  for (const ann of annotations) {
    for (const pt of ann.points) {
      const projected = map.project([pt.lng, pt.lat]);
      const clickProjected = map.project([clickLngLat.lng, clickLngLat.lat]);
      const dx = projected.x - clickProjected.x;
      const dy = projected.y - clickProjected.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < bestDist) {
        bestDist = dist;
        bestPoint = { lat: pt.lat, lng: pt.lng };
      }
    }
  }

  if (bestDist <= SNAP_DISTANCE_PX && bestPoint) {
    return bestPoint;
  }
  return null;
}

// --- Center line rendering ---

let centerLineSources = new Set();
let manualCenterLineSources = new Set();

function clearCenterLines() {
  for (const id of centerLineSources) {
    if (map.getLayer(`layer-center-${id}`)) map.removeLayer(`layer-center-${id}`);
    if (map.getSource(`source-center-${id}`)) map.removeSource(`source-center-${id}`);
  }
  centerLineSources.clear();
}

function clearManualCenterLineLayers() {
  for (const id of manualCenterLineSources) {
    if (map.getLayer(`layer-manual-center-${id}`)) map.removeLayer(`layer-manual-center-${id}`);
    if (map.getSource(`source-manual-center-${id}`)) map.removeSource(`source-manual-center-${id}`);
  }
  manualCenterLineSources.clear();
}

function addCenterLineLayer(sourceId, features, selected = false) {
  centerLineSources.add(sourceId);
  map.addSource(`source-center-${sourceId}`, {
    type: 'geojson',
    data: { type: 'FeatureCollection', features }
  });
  map.addLayer({
    id: `layer-center-${sourceId}`,
    type: 'line',
    source: `source-center-${sourceId}`,
    paint: {
      'line-color': selected ? '#fff2a8' : COLORS.center,
      'line-width': selected ? 6 : 3,
      'line-dasharray': [4, 3],
      'line-opacity': selected ? 1 : 0.85
    },
    layout: { 'line-cap': 'round', 'line-join': 'round' }
  });
}

function updateManualCenterLine(line) {
  const sourceId = `manual-center-${line.id}`;

  if (line.points.length < 2) {
    removeManualCenterLineFromMap(line.id);
    return;
  }

  const geojson = {
    type: 'Feature',
    properties: { kind: 'manual-centerline', id: line.id },
    geometry: {
      type: 'LineString',
      coordinates: line.points.map(p => [p.lng, p.lat])
    }
  };

  if (map.getSource(`source-${sourceId}`)) {
    map.getSource(`source-${sourceId}`).setData(geojson);
  } else {
    manualCenterLineSources.add(line.id);
    map.addSource(`source-${sourceId}`, { type: 'geojson', data: geojson });
    map.addLayer({
      id: `layer-${sourceId}`,
      type: 'line',
      source: `source-${sourceId}`,
      paint: {
        'line-color': COLORS.center,
        'line-width': 4,
        'line-dasharray': [4, 3],
        'line-opacity': 0.95
      },
      layout: { 'line-cap': 'round', 'line-join': 'round' }
    });
  }
}

function removeManualCenterLineFromMap(id) {
  const layerId = `layer-manual-center-${id}`;
  const sourceId = `source-manual-center-${id}`;
  if (map.getLayer(layerId)) map.removeLayer(layerId);
  if (map.getSource(sourceId)) map.removeSource(sourceId);
  manualCenterLineSources.delete(id);
}

function renderCenterLines() {
  clearCenterLines();
  autoCenterLineRecords.clear();
  const pairs = findPairs();
  const listEl = document.getElementById('center-line-list');
  listEl.innerHTML = '';

  if (pairs.length === 0) {
    listEl.innerHTML = '<p class="hint">Draw 2 lane boundaries with the same road name to auto-generate a center line.</p>';
    return;
  }

  const laneCenterLines = [];
  let visibleCount = 0;

  pairs.forEach(pair => {
    const id = getAutoCenterLineId(pair);
    if (suppressedAutoCenterLineIds.includes(id)) return;

    const center = computeCenterLine(pair.a.points, pair.b.points);
    laneCenterLines.push(center);
    const segments = applyCenterLineErasure(center, eraserPoints, eraserRadius);

    const features = segments.map(seg => ({
      type: 'Feature',
      properties: { kind: 'auto-centerline', id },
      geometry: {
        type: 'LineString',
        coordinates: seg.map(p => [p.lng, p.lat])
      }
    }));

    const erasedCount = eraserPoints.filter(ep =>
      center.some(cp => haversineMeters(cp, ep) < (ep.radius || eraserRadius) + 5)
    ).length;
    const record = { id, name: pair.name, pair, center, erasedCount };
    autoCenterLineRecords.set(id, record);
    addCenterLineLayer(id, features, selectedAutoCenterLineId === id);

    const item = document.createElement('div');
    item.className = `center-line-item${selectedAutoCenterLineId === id ? ' selected' : ''}`;
    item.dataset.autoCenterlineId = id;
    item.innerHTML = `
      <div class="annotation-dot center"></div>
      <div class="annotation-info">
        <div class="annotation-name">${pair.name}</div>
        <div class="annotation-meta">auto | ${center.length} pts${erasedCount > 0 ? ` | ${erasedCount} eraser(s)` : ''}</div>
      </div>
      <button class="center-line-edit-btn" title="Convert to editable manual centerline">Edit</button>
      <button class="annotation-delete-btn" title="Suppress auto centerline">&times;</button>
    `;
    item.querySelector('.annotation-info').addEventListener('click', () => selectAutoCenterLine(id));
    item.querySelector('.center-line-edit-btn').addEventListener('click', (e) => {
      e.stopPropagation();
      convertAutoCenterLineToManual(id);
    });
    item.querySelector('.annotation-delete-btn').addEventListener('click', (e) => {
      e.stopPropagation();
      deleteAutoCenterLine(id);
    });
    listEl.appendChild(item);
    visibleCount++;
  });

  if (selectedAutoCenterLineId && !autoCenterLineRecords.has(selectedAutoCenterLineId)) {
    selectedAutoCenterLineId = null;
  }

  if (visibleCount === 0) {
    listEl.innerHTML = '<p class="hint">All generated center lines are hidden. Draw manual center lines where needed.</p>';
  }
}

// --- Live GPS ---

function updateGpsMarker(data) {
  if (!gpsMarker) {
    const el = document.createElement('div');
    el.className = 'gps-dot';
    gpsMarker = new mapboxgl.Marker({ element: el }).addTo(map);
  }

  gpsMarker
    .setLngLat([data.lon, data.lat])
    .getElement().style.background = FIX_COLORS[data.fix_code] || FIX_COLORS[0];
}

function initGpsTrailSource() {
  map.addSource('gps-trail', {
    type: 'geojson',
    data: { type: 'FeatureCollection', features: [] }
  });
  map.addLayer({
    id: 'gps-trail-layer',
    type: 'line',
    source: 'gps-trail',
    paint: {
      'line-color': ['get', 'color'],
      'line-width': 3,
      'line-opacity': 0.8
    },
    layout: { 'line-cap': 'round', 'line-join': 'round' }
  });
}

function updateGpsTrailOnMap() {
  if (!map.getSource('gps-trail')) return;
  const features = [];
  for (let i = 1; i < gpsTrail.length; i++) {
    const prev = gpsTrail[i - 1];
    const curr = gpsTrail[i];
    features.push({
      type: 'Feature',
      properties: { color: FIX_COLORS[curr.fix_code] || FIX_COLORS[0] },
      geometry: {
        type: 'LineString',
        coordinates: [[prev.lon, prev.lat], [curr.lon, curr.lat]]
      }
    });
  }
  map.getSource('gps-trail').setData({ type: 'FeatureCollection', features });
}

function onGpsPosition(data) {
  gpsHzCounter++;
  gpsPointCount++;
  document.getElementById('gps-point-count').textContent = `${gpsPointCount} points collected`;

  gpsTrail.push(data);
  if (gpsTrail.length > GPS_TRAIL_MAX) {
    gpsTrail = gpsTrail.slice(-GPS_TRAIL_MAX);
  }

  updateGpsMarker(data);

  if (gpsFollowing && map) {
    map.easeTo({ center: [data.lon, data.lat], duration: 80 });
  }

  updateGpsTrailOnMap();
  updateGpsPanel(data);
}

function updateGpsPanel(d) {
  const badge = document.getElementById('gps-fix-badge');
  badge.textContent = d.fix;
  badge.className = 'fix-badge';
  if (d.fix_code === 4) badge.classList.add('rtk-fix');
  else if (d.fix_code === 5) badge.classList.add('rtk-float');
  else if (d.fix_code === 2) badge.classList.add('dgps');
  else if (d.fix_code >= 1) badge.classList.add('gps');

  document.getElementById('gps-lat').textContent = d.lat.toFixed(8);
  document.getElementById('gps-lon').textContent = d.lon.toFixed(8);
  document.getElementById('gps-sats').textContent = d.sats;
  document.getElementById('gps-hdop').textContent = d.hdop.toFixed(2);
  document.getElementById('gps-alt').textContent = d.alt.toFixed(1) + 'm';
  document.getElementById('gps-hz').textContent = gpsHzValue > 0 ? `${gpsHzValue} Hz` : '';
}

function connectGpsWebSocket() {
  const statusEl = document.getElementById('gps-status');
  const gpsWsUrl = APP_CONFIG.gpsWsUrl || '';

  if (!gpsWsUrl) {
    statusEl.classList.add('disconnected');
    document.getElementById('gps-fix-badge').textContent = 'NO GPS';
    document.getElementById('gps-fix-badge').className = 'fix-badge';
    setStatus('Live sync ready. GPS feed is not configured for this site.');
    return;
  }

  gpsWs = new WebSocket(gpsWsUrl);

  gpsWs.onopen = () => {
    statusEl.classList.remove('disconnected');
    setStatus('GPS connected.');

    gpsHzInterval = setInterval(() => {
      gpsHzValue = gpsHzCounter;
      gpsHzCounter = 0;
      const hzEl = document.getElementById('gps-hz');
      if (hzEl) hzEl.textContent = gpsHzValue > 0 ? `${gpsHzValue} Hz` : '';
    }, 1000);
  };

  gpsWs.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'position') {
      onGpsPosition(msg.data);
    } else if (msg.type === 'history') {
      for (const pt of msg.data) {
        gpsTrail.push(pt);
      }
      gpsPointCount = msg.data.length;
      document.getElementById('gps-point-count').textContent = `${gpsPointCount} points collected`;
      if (gpsTrail.length > GPS_TRAIL_MAX) {
        gpsTrail = gpsTrail.slice(-GPS_TRAIL_MAX);
      }
      updateGpsTrailOnMap();
      if (gpsTrail.length > 0) {
        const last = gpsTrail[gpsTrail.length - 1];
        updateGpsMarker(last);
        updateGpsPanel(last);
        if (gpsFollowing) map.flyTo({ center: [last.lon, last.lat], zoom: 19 });
      }
    } else if (msg.type === 'saved') {
      setStatus(`Saved ${msg.count} GPS points to ${msg.path.split('/').pop()}`);
    } else if (msg.type === 'cleared') {
      gpsTrail = [];
      gpsPointCount = 0;
      document.getElementById('gps-point-count').textContent = '0 points collected';
      updateGpsTrailOnMap();
      setStatus('GPS points cleared.');
    }
  };

  gpsWs.onclose = () => {
    statusEl.classList.add('disconnected');
    clearInterval(gpsHzInterval);
    gpsHzValue = 0;
    document.getElementById('gps-hz').textContent = '';
    document.getElementById('gps-fix-badge').textContent = 'OFFLINE';
    document.getElementById('gps-fix-badge').className = 'fix-badge';
    if (APP_CONFIG.gpsWsUrl) setTimeout(connectGpsWebSocket, 2000);
  };

  gpsWs.onerror = () => {
    gpsWs.close();
  };
}

function clearGpsTrail() {
  gpsTrail = [];
  updateGpsTrailOnMap();
  setStatus('GPS trail cleared.');
}

function selectRenderedRouteAtClick(e) {
  if (!map) return false;
  const tolerance = 8;
  const lineLayers = [
    ...annotations.map(ann => `layer-${ann.id}`),
    ...connectors.map(conn => `layer-connector-${conn.id}`),
    ...[...centerLineSources].map(id => `layer-center-${id}`),
    ...manualCenterLines.map(line => `layer-manual-center-${line.id}`)
  ].filter(id => map.getLayer(id));
  if (lineLayers.length === 0) return false;

  const features = map.queryRenderedFeatures(
    [
      [e.point.x - tolerance, e.point.y - tolerance],
      [e.point.x + tolerance, e.point.y + tolerance]
    ],
    { layers: lineLayers }
  );
  const feature = features.find(f => f.properties?.kind && f.properties?.id);
  if (!feature) return false;

  if (feature.properties.kind === 'lane') {
    selectAnnotation(feature.properties.id);
    return true;
  }
  if (feature.properties.kind === 'connector') {
    selectConnector(feature.properties.id);
    return true;
  }
  if (feature.properties.kind === 'manual-centerline') {
    selectManualCenterLine(feature.properties.id);
    return true;
  }
  if (feature.properties.kind === 'auto-centerline') {
    selectAutoCenterLine(feature.properties.id);
    return true;
  }
  return false;
}

// --- Map ---

async function initMap() {
  const config = await fetch('/api/config', { headers: authHeaders() }).then(r => r.json());
  APP_CONFIG = config;
  if (config.passwordRequired && (!appPassword || !collaboratorName)) {
    showJoinOverlay();
    return;
  }

  MAPBOX_TOKEN = config.mapboxToken;
  mapboxgl.accessToken = MAPBOX_TOKEN;
  setSyncStatus(config.storageMode === 'supabase' ? 'Connecting live sync...' : 'Local mode', config.storageMode === 'supabase' ? 'online' : 'offline');

  map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/satellite-v9',
    center: [-122.1697, 37.4275],
    zoom: 16,
    maxZoom: 22
  });

  map.addControl(new mapboxgl.NavigationControl(), 'top-right');
  map.addControl(new mapboxgl.ScaleControl(), 'bottom-right');

  map.on('load', () => {
    map.on('click', onMapClick);
    initGpsTrailSource();
    connectGpsWebSocket();
    loadAnnotations().then(() => {
      initRealtimeSync();
      if (annotations.length === 0) {
        setStatus('Connecting to GPS...');
      }
    });
  });

  map.on('dragstart', () => {
    gpsFollowing = false;
    document.getElementById('btn-follow').classList.remove('active');
  });
}

function onMapClick(e) {
  const isActivelyDrawing =
    (currentMode === 'lane' && activeAnnotationId)
    || (currentMode === 'connector' && activeConnectorId)
    || (currentMode === 'centerline' && activeManualCenterLineId)
    || currentMode === 'eraser';

  if (!isActivelyDrawing && selectRenderedRouteAtClick(e)) return;

  const point = { lat: e.lngLat.lat, lng: e.lngLat.lng };

  if (currentMode === 'eraser') {
    const ep = { id: `eraser-${Date.now()}`, lat: point.lat, lng: point.lng, radius: eraserRadius, createdBy: collaboratorName };
    eraserPoints.push(ep);
    addEraserMarker(ep);
    renderCenterLines();
    scheduleAutoSave();
    setStatus(`Placed eraser (${eraserRadius}m radius). ${eraserPoints.length} total.`);
    return;
  }

  if (currentMode === 'connector') {
    if (!activeConnectorId) {
      setStatus('Click "New Connector" first.');
      return;
    }
    const conn = connectors.find(c => c.id === activeConnectorId);
    if (!conn) return;

    const snapped = snapToLanePoint(e.lngLat);
    const addedPoint = snapped || point;
    conn.points.push(addedPoint);
    touchConnector(conn);
    addConnectorMarker(addedPoint, conn.points.length - 1, conn.id, !!snapped);
    updateConnectorLine(conn);
    updateConnectorList();
    renderCenterLines();
    renderDrawingLabels();
    scheduleAutoSave();
    setStatus(snapped
      ? `Snapped to lane point (${conn.points.length} pts)`
      : `Added free point (${conn.points.length} pts) — no nearby lane point to snap to`);
    return;
  }

  if (currentMode === 'centerline') {
    if (!activeManualCenterLineId) {
      setStatus('Click "New Center" first.');
      return;
    }
    const line = manualCenterLines.find(c => c.id === activeManualCenterLineId);
    if (!line) return;

    line.points.push(point);
    touchManualCenterLine(line);
    addManualCenterLineMarker(point, line.points.length - 1, line.id);
    updateManualCenterLine(line);
    updateManualCenterLineList();
    renderDrawingLabels();
    scheduleAutoSave();
    setStatus(`Added manual center point ${line.points.length} to "${line.name}"`);
    return;
  }

  if (!activeAnnotationId) {
    setStatus('Click "New Line" first to start a new annotation.');
    return;
  }

  const annotation = annotations.find(a => a.id === activeAnnotationId);
  if (!annotation) return;

  annotation.points.push(point);
  touchAnnotation(annotation);
  addPointMarker(point, annotation.points.length - 1, annotation.id);
  updateLine(annotation);
  renderCenterLines();
  renderDrawingLabels();
  updateAnnotationList();
  scheduleAutoSave();
  setStatus(`Added point ${annotation.points.length} to "${annotation.name}"`);
}

// --- Point markers ---

function addPointMarker(point, index, annotationId) {
  const el = document.createElement('div');
  el.style.width = '12px';
  el.style.height = '12px';
  el.style.borderRadius = '50%';
  el.style.background = COLORS.lane;
  el.style.border = '2px solid white';
  el.style.cursor = 'pointer';
  el.style.boxShadow = '0 1px 4px rgba(0,0,0,0.5)';

  const marker = new mapboxgl.Marker({ element: el, draggable: true })
    .setLngLat([point.lng, point.lat])
    .addTo(map);

  marker._annotationId = annotationId;
  marker._pointIndex = index;

  el.addEventListener('click', (e) => {
    e.stopPropagation();
    selectAnnotation(annotationId);
  });

  marker.on('dragend', () => {
    const lngLat = marker.getLngLat();
    const ann = annotations.find(a => a.id === annotationId);
    if (ann && ann.points[marker._pointIndex]) {
      ann.points[marker._pointIndex] = { lat: lngLat.lat, lng: lngLat.lng };
      touchAnnotation(ann);
      updateLine(ann);
      renderCenterLines();
      renderDrawingLabels();
      scheduleAutoSave();
    }
  });

  pointMarkers.push(marker);
}

// --- Eraser markers ---

function addEraserMarker(ep) {
  const el = document.createElement('div');
  el.style.width = '18px';
  el.style.height = '18px';
  el.style.borderRadius = '50%';
  el.style.background = 'rgba(255, 136, 0, 0.6)';
  el.style.border = '2px solid #ff8800';
  el.style.cursor = 'pointer';
  el.style.display = 'flex';
  el.style.alignItems = 'center';
  el.style.justifyContent = 'center';
  el.style.fontSize = '11px';
  el.style.fontWeight = 'bold';
  el.style.color = 'white';
  el.style.boxShadow = '0 1px 4px rgba(0,0,0,0.5)';
  el.textContent = '×';
  el.title = `Eraser (${ep.radius}m) — click to delete`;

  const marker = new mapboxgl.Marker({ element: el, draggable: true })
    .setLngLat([ep.lng, ep.lat])
    .addTo(map);

  marker._eraserId = ep.id;

  el.addEventListener('click', (e) => {
    if (currentMode !== 'eraser') return;
    e.stopPropagation();
    deleteEraserPoint(ep.id);
  });

  marker.on('dragend', () => {
    const lngLat = marker.getLngLat();
    const point = eraserPoints.find(p => p.id === ep.id);
    if (point) {
      point.lat = lngLat.lat;
      point.lng = lngLat.lng;
      renderCenterLines();
      scheduleAutoSave();
    }
  });

  eraserMarkers.push(marker);
}

function deleteEraserPoint(id) {
  eraserPoints = eraserPoints.filter(p => p.id !== id);
  eraserMarkers = eraserMarkers.filter(m => {
    if (m._eraserId === id) {
      m.remove();
      return false;
    }
    return true;
  });
  renderCenterLines();
  scheduleAutoSave();
  setStatus(`Deleted eraser. ${eraserPoints.length} remaining.`);
}

function clearAllErasers() {
  eraserMarkers.forEach(m => m.remove());
  eraserMarkers = [];
  eraserPoints = [];
  renderCenterLines();
  scheduleAutoSave();
  setStatus('Cleared all erasers.');
}

// --- Connector markers ---

function addConnectorMarker(point, index, connectorId, snapped) {
  const el = document.createElement('div');
  el.style.width = snapped ? '14px' : '10px';
  el.style.height = snapped ? '14px' : '10px';
  el.style.borderRadius = '2px';
  el.style.background = COLORS.connector;
  el.style.border = '2px solid white';
  el.style.cursor = 'pointer';
  el.style.boxShadow = '0 1px 4px rgba(0,0,0,0.5)';
  if (snapped) el.style.transform = 'rotate(45deg)';

  const marker = new mapboxgl.Marker({ element: el, draggable: true })
    .setLngLat([point.lng, point.lat])
    .addTo(map);

  marker._connectorId = connectorId;
  marker._pointIndex = index;

  el.addEventListener('click', (e) => {
    e.stopPropagation();
    selectConnector(connectorId);
  });

  marker.on('dragend', () => {
    const lngLat = marker.getLngLat();
    const snappedPt = snapToLanePoint(lngLat);
    const finalPt = snappedPt || { lat: lngLat.lat, lng: lngLat.lng };
    const conn = connectors.find(c => c.id === connectorId);
    if (conn && conn.points[marker._pointIndex]) {
      conn.points[marker._pointIndex] = finalPt;
      touchConnector(conn);
      marker.setLngLat([finalPt.lng, finalPt.lat]);
      el.style.width = snappedPt ? '14px' : '10px';
      el.style.height = snappedPt ? '14px' : '10px';
      el.style.transform = snappedPt ? 'rotate(45deg)' : '';
      updateConnectorLine(conn);
      renderCenterLines();
      renderDrawingLabels();
      scheduleAutoSave();
    }
  });

  connectorMarkers.push(marker);
}

function clearConnectorMarkers(connectorId) {
  connectorMarkers = connectorMarkers.filter(m => {
    if (m._connectorId === connectorId) {
      m.remove();
      return false;
    }
    return true;
  });
}

// --- Manual centerline markers ---

function addManualCenterLineMarker(point, index, lineId) {
  const el = document.createElement('div');
  el.className = 'manual-centerline-point';
  el.title = 'Manual centerline point';

  const marker = new mapboxgl.Marker({ element: el, draggable: true })
    .setLngLat([point.lng, point.lat])
    .addTo(map);

  marker._manualCenterLineId = lineId;
  marker._pointIndex = index;

  el.addEventListener('click', (e) => {
    e.stopPropagation();
    selectManualCenterLine(lineId);
  });

  marker.on('dragend', () => {
    const lngLat = marker.getLngLat();
    const line = manualCenterLines.find(c => c.id === lineId);
    if (line && line.points[marker._pointIndex]) {
      line.points[marker._pointIndex] = { lat: lngLat.lat, lng: lngLat.lng };
      touchManualCenterLine(line);
      updateManualCenterLine(line);
      renderDrawingLabels();
      scheduleAutoSave();
    }
  });

  manualCenterLineMarkers.push(marker);
}

function clearManualCenterLineMarkers(lineId) {
  manualCenterLineMarkers = manualCenterLineMarkers.filter(m => {
    if (m._manualCenterLineId === lineId) {
      m.remove();
      return false;
    }
    return true;
  });
}

function updateConnectorLine(conn) {
  const sourceId = `connector-${conn.id}`;

  if (conn.points.length < 2) {
    if (map.getSource(sourceId)) {
      map.getSource(sourceId).setData({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: [] }
      });
    }
    return;
  }

  const coordinates = conn.points.map(p => [p.lng, p.lat]);
  const geojson = {
    type: 'Feature',
    properties: { kind: 'connector', id: conn.id },
    geometry: { type: 'LineString', coordinates }
  };

  if (map.getSource(sourceId)) {
    map.getSource(sourceId).setData(geojson);
  } else {
    map.addSource(sourceId, { type: 'geojson', data: geojson });
    map.addLayer({
      id: `layer-connector-${conn.id}`,
      type: 'line',
      source: sourceId,
      paint: {
        'line-color': COLORS.connector,
        'line-width': 3,
        'line-dasharray': [6, 4],
        'line-opacity': 0.9
      },
      layout: { 'line-cap': 'round', 'line-join': 'round' }
    });
  }
}

function removeConnectorFromMap(connectorId) {
  const layerId = `layer-connector-${connectorId}`;
  const sourceId = `connector-${connectorId}`;
  if (map.getLayer(layerId)) map.removeLayer(layerId);
  if (map.getSource(sourceId)) map.removeSource(sourceId);
}

// --- Line rendering ---

function updateLine(annotation) {
  const sourceId = `line-${annotation.id}`;

  if (annotation.points.length < 2) {
    if (map.getSource(sourceId)) {
      map.getSource(sourceId).setData({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: [] }
      });
    }
    return;
  }

  const coordinates = annotation.points.map(p => [p.lng, p.lat]);
  const geojson = {
    type: 'Feature',
    properties: { kind: 'lane', id: annotation.id },
    geometry: { type: 'LineString', coordinates }
  };

  if (map.getSource(sourceId)) {
    map.getSource(sourceId).setData(geojson);
  } else {
    map.addSource(sourceId, { type: 'geojson', data: geojson });
    map.addLayer({
      id: `layer-${annotation.id}`,
      type: 'line',
      source: sourceId,
      paint: {
        'line-color': COLORS.lane,
        'line-width': 4,
        'line-opacity': 0.9
      },
      layout: { 'line-cap': 'round', 'line-join': 'round' }
    });
  }
}

function removeLineFromMap(annotationId) {
  const layerId = `layer-${annotationId}`;
  const sourceId = `line-${annotationId}`;
  if (map.getLayer(layerId)) map.removeLayer(layerId);
  if (map.getSource(sourceId)) map.removeSource(sourceId);
}

// --- Marker management ---

function clearAllMarkers() {
  pointMarkers.forEach(m => m.remove());
  pointMarkers = [];
}

function clearDrawingLabels() {
  drawingLabelMarkers.forEach(m => m.remove());
  drawingLabelMarkers = [];
}

function addDrawingLabel(item, type) {
  if (!item.activeBy || item.points.length === 0) return;
  const last = item.points[item.points.length - 1];
  const el = document.createElement('div');
  el.className = `drawing-label ${type}`;
  el.textContent = item.activeBy;
  el.title = `${item.activeBy} is drawing ${item.name}`;

  const marker = new mapboxgl.Marker({ element: el, anchor: 'bottom-left', offset: [10, -8] })
    .setLngLat([last.lng, last.lat])
    .addTo(map);

  drawingLabelMarkers.push(marker);
}

function renderDrawingLabels() {
  clearDrawingLabels();
  annotations.forEach(ann => addDrawingLabel(ann, 'lane'));
  connectors.forEach(conn => addDrawingLabel(conn, 'connector'));
  manualCenterLines.forEach(line => addDrawingLabel(line, 'centerline'));
}

function scrollSelectedListItem(kind, id) {
  requestAnimationFrame(() => {
    let selector = `[data-annotation-id="${id}"]`;
    if (kind === 'connector') selector = `[data-connector-id="${id}"]`;
    if (kind === 'manual-centerline') selector = `[data-manual-centerline-id="${id}"]`;
    if (kind === 'auto-centerline') selector = `[data-auto-centerline-id="${id}"]`;
    const item = document.querySelector(selector);
    if (item) item.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  });
}

function clearMarkersForAnnotation(annotationId) {
  pointMarkers = pointMarkers.filter(m => {
    if (m._annotationId === annotationId) {
      m.remove();
      return false;
    }
    return true;
  });
}

function rebuildAllVisuals() {
  clearAllMarkers();
  connectorMarkers.forEach(m => m.remove());
  connectorMarkers = [];
  manualCenterLineMarkers.forEach(m => m.remove());
  manualCenterLineMarkers = [];
  eraserMarkers.forEach(m => m.remove());
  eraserMarkers = [];
  clearManualCenterLineLayers();
  clearDrawingLabels();

  annotations.forEach(ann => {
    ann.points.forEach((pt, i) => addPointMarker(pt, i, ann.id));
    updateLine(ann);
  });

  connectors.forEach(conn => {
    conn.points.forEach((pt, i) => addConnectorMarker(pt, i, conn.id, true));
    updateConnectorLine(conn);
  });

  manualCenterLines.forEach(line => {
    line.points.forEach((pt, i) => addManualCenterLineMarker(pt, i, line.id));
    updateManualCenterLine(line);
  });

  eraserPoints.forEach(ep => addEraserMarker(ep));
  renderCenterLines();
  renderDrawingLabels();
  updateManualCenterLineList();
}

// --- CRUD ---

function newLine() {
  if (currentMode === 'eraser') {
    setStatus('Switch to Lane mode to create a new line.');
    return;
  }

  const nameInput = document.getElementById('line-name');
  lineCounter++;
  const roadName = nameInput.value.trim() || `Road ${lineCounter}`;
  const clearedActive = !!(clearActiveAnnotation() || clearActiveConnector() || clearActiveManualCenterLine());
  selectedAutoCenterLineId = null;

  const annotation = {
    id: `ann-${Date.now()}-${lineCounter}`,
    name: roadName,
    type: 'lane',
    points: [],
    createdBy: collaboratorName,
    updatedBy: collaboratorName,
    updatedAt: new Date().toISOString(),
    activeBy: collaboratorName
  };

  annotations.push(annotation);
  activeAnnotationId = annotation.id;
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  renderCenterLines();
  renderDrawingLabels();
  if (clearedActive) scheduleAutoSave();
  setStatus(`Started "${roadName}". Click on the map to add points.`);
}

function finishLine() {
  if (!activeAnnotationId) {
    setStatus('No active line to finish.');
    return;
  }
  const ann = annotations.find(a => a.id === activeAnnotationId);
  const name = ann ? ann.name : '';
  clearActiveAnnotation();
  updateAnnotationList();
  renderCenterLines();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Finished "${name}". Click "New Line" to start another.`);
}

function undoPoint() {
  if (!activeAnnotationId) {
    setStatus('No active line selected.');
    return;
  }
  const annotation = annotations.find(a => a.id === activeAnnotationId);
  if (!annotation || annotation.points.length === 0) {
    setStatus('No points to undo.');
    return;
  }

  annotation.points.pop();
  touchAnnotation(annotation);
  clearMarkersForAnnotation(annotation.id);
  annotation.points.forEach((pt, i) => addPointMarker(pt, i, annotation.id));
  updateLine(annotation);
  renderCenterLines();
  renderDrawingLabels();
  updateAnnotationList();
  scheduleAutoSave();
  setStatus(`Removed last point. ${annotation.points.length} remaining.`);
}

function undoEraser() {
  if (eraserPoints.length === 0) {
    setStatus('No erasers to undo.');
    return;
  }
  const last = eraserPoints[eraserPoints.length - 1];
  deleteEraserPoint(last.id);
}

// --- Connector CRUD ---

function newConnector() {
  const nameInput = document.getElementById('connector-name');
  connectorCounter++;
  const name = nameInput.value.trim() || `Connector ${connectorCounter}`;
  const clearedActive = !!(clearActiveAnnotation() || clearActiveConnector() || clearActiveManualCenterLine());
  selectedAutoCenterLineId = null;

  const conn = {
    id: `conn-${Date.now()}-${connectorCounter}`,
    name,
    type: 'connector',
    points: [],
    createdBy: collaboratorName,
    updatedBy: collaboratorName,
    updatedAt: new Date().toISOString(),
    activeBy: collaboratorName
  };

  connectors.push(conn);
  activeConnectorId = conn.id;
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  renderCenterLines();
  renderDrawingLabels();
  if (clearedActive) scheduleAutoSave();
  setStatus(`Started connector "${name}". Click near lane points to snap.`);
}

function finishConnector() {
  if (!activeConnectorId) {
    setStatus('No active connector to finish.');
    return;
  }
  const conn = clearActiveConnector();
  updateConnectorList();
  renderCenterLines();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Finished connector "${conn ? conn.name : ''}".`);
}

function undoConnectorPoint() {
  if (!activeConnectorId) {
    setStatus('No active connector selected.');
    return;
  }
  const conn = connectors.find(c => c.id === activeConnectorId);
  if (!conn || conn.points.length === 0) {
    setStatus('No points to undo.');
    return;
  }
  conn.points.pop();
  touchConnector(conn);
  clearConnectorMarkers(conn.id);
  conn.points.forEach((pt, i) => addConnectorMarker(pt, i, conn.id, true));
  updateConnectorLine(conn);
  updateConnectorList();
  renderCenterLines();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Removed last connector point. ${conn.points.length} remaining.`);
}

function deleteConnector(id) {
  const idx = connectors.findIndex(c => c.id === id);
  if (idx === -1) return;
  const name = connectors[idx].name;
  clearConnectorMarkers(id);
  removeConnectorFromMap(id);
  connectors.splice(idx, 1);
  if (activeConnectorId === id) activeConnectorId = null;
  updateConnectorList();
  renderCenterLines();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Deleted connector "${name}".`);
}

function selectConnector(id) {
  if (activeAnnotationId) clearActiveAnnotation();
  if (activeManualCenterLineId) clearActiveManualCenterLine();
  if (activeConnectorId && activeConnectorId !== id) clearActiveConnector();
  selectedAutoCenterLineId = null;
  activeConnectorId = id;
  activeAnnotationId = null;
  activeManualCenterLineId = null;
  currentMode = 'connector';
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  document.querySelector('[data-mode="connector"]').classList.add('active');
  toggleModeUI();
  const conn = connectors.find(c => c.id === id);
  if (conn) {
    document.getElementById('connector-name').value = conn.name;
    conn.activeBy = collaboratorName;
    touchConnector(conn);
    setStatus(`Selected connector "${conn.name}" for editing.`);
  }
  renderDrawingLabels();
  renderCenterLines();
  scheduleAutoSave();
  updateConnectorList();
  updateAnnotationList();
  updateManualCenterLineList();
  scrollSelectedListItem('connector', id);
}

function updateConnectorList() {
  const list = document.getElementById('connector-list');
  const count = document.getElementById('connector-count');
  count.textContent = connectors.length;

  list.innerHTML = '';
  if (connectors.length === 0) {
    list.innerHTML = '<p class="hint">Use Connector mode to draw connections between lanes.</p>';
    return;
  }
  connectors.forEach(conn => {
    const item = document.createElement('div');
    item.className = `annotation-item${conn.id === activeConnectorId ? ' selected' : ''}`;
    item.dataset.connectorId = conn.id;
    item.innerHTML = `
      <div class="annotation-dot connector"></div>
      <div class="annotation-info" title="${conn.name}">
        <div class="annotation-name">${conn.name}</div>
        <div class="annotation-meta">${conn.points.length} pts${conn.updatedBy ? ` | ${conn.updatedBy}` : ''}</div>
      </div>
      <button class="annotation-delete-btn" title="Delete">&times;</button>
    `;
    item.querySelector('.annotation-info').addEventListener('click', () => selectConnector(conn.id));
    item.querySelector('.annotation-delete-btn').addEventListener('click', (e) => {
      e.stopPropagation();
      deleteConnector(conn.id);
    });
    list.appendChild(item);
  });
}

// --- Auto centerline selection / conversion ---

function selectAutoCenterLine(id) {
  const record = autoCenterLineRecords.get(id);
  if (!record) return;

  const clearedActive = !!(clearActiveAnnotation() || clearActiveConnector() || clearActiveManualCenterLine());
  activeAnnotationId = null;
  activeConnectorId = null;
  activeManualCenterLineId = null;
  selectedAutoCenterLineId = id;

  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  renderCenterLines();
  renderDrawingLabels();
  if (clearedActive) scheduleAutoSave();
  setStatus(`Selected auto centerline "${record.name}". Use Edit to convert it to a manual line, or delete it to hide it.`);
  scrollSelectedListItem('auto-centerline', id);
}

function deleteAutoCenterLine(id) {
  const record = autoCenterLineRecords.get(id);
  suppressAutoCenterLine(id);
  renderCenterLines();
  scheduleAutoSave();
  setStatus(`Deleted auto centerline "${record ? record.name : 'centerline'}". It will stay hidden in the shared data.`);
}

function convertAutoCenterLineToManual(id) {
  const record = autoCenterLineRecords.get(id);
  if (!record || record.center.length < 2) {
    setStatus('Could not convert this auto centerline.');
    return;
  }

  clearActiveAnnotation();
  clearActiveConnector();
  clearActiveManualCenterLine();
  suppressAutoCenterLine(id);
  selectedAutoCenterLineId = null;

  manualCenterLineCounter++;
  const line = {
    id: `center-${Date.now()}-${manualCenterLineCounter}`,
    name: `${record.name} center`,
    type: 'manual-centerline',
    points: record.center.map(p => ({ lat: p.lat, lng: p.lng })),
    createdBy: collaboratorName,
    updatedBy: collaboratorName,
    updatedAt: new Date().toISOString(),
    activeBy: collaboratorName
  };

  manualCenterLines.push(line);
  activeManualCenterLineId = line.id;
  currentMode = 'centerline';
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  document.querySelector('[data-mode="centerline"]').classList.add('active');
  toggleModeUI();
  document.getElementById('centerline-name').value = line.name;

  line.points.forEach((pt, i) => addManualCenterLineMarker(pt, i, line.id));
  updateManualCenterLine(line);
  renderCenterLines();
  renderDrawingLabels();
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  scheduleAutoSave();
  setStatus(`Converted "${record.name}" to an editable manual centerline.`);
  scrollSelectedListItem('manual-centerline', line.id);
}

// --- Manual centerline CRUD ---

function newManualCenterLine() {
  const nameInput = document.getElementById('centerline-name');
  manualCenterLineCounter++;
  const name = nameInput.value.trim() || `Manual Center ${manualCenterLineCounter}`;

  const clearedActive = !!(clearActiveAnnotation() || clearActiveConnector() || clearActiveManualCenterLine());
  selectedAutoCenterLineId = null;
  const line = {
    id: `center-${Date.now()}-${manualCenterLineCounter}`,
    name,
    type: 'manual-centerline',
    points: [],
    createdBy: collaboratorName,
    updatedBy: collaboratorName,
    updatedAt: new Date().toISOString(),
    activeBy: collaboratorName
  };

  manualCenterLines.push(line);
  activeManualCenterLineId = line.id;
  currentMode = 'centerline';
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  document.querySelector('[data-mode="centerline"]').classList.add('active');
  toggleModeUI();
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  renderCenterLines();
  renderDrawingLabels();
  if (clearedActive) scheduleAutoSave();
  setStatus(`Started manual centerline "${name}". Click on the map to add yellow center points.`);
}

function finishManualCenterLine() {
  if (!activeManualCenterLineId) {
    setStatus('No active manual centerline to finish.');
    return;
  }
  const line = clearActiveManualCenterLine();
  updateManualCenterLineList();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Finished manual centerline "${line ? line.name : ''}".`);
}

function undoManualCenterLinePoint() {
  if (!activeManualCenterLineId) {
    setStatus('No active manual centerline selected.');
    return;
  }
  const line = manualCenterLines.find(c => c.id === activeManualCenterLineId);
  if (!line || line.points.length === 0) {
    setStatus('No manual centerline points to undo.');
    return;
  }

  line.points.pop();
  touchManualCenterLine(line);
  clearManualCenterLineMarkers(line.id);
  line.points.forEach((pt, i) => addManualCenterLineMarker(pt, i, line.id));
  updateManualCenterLine(line);
  updateManualCenterLineList();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Removed last manual center point. ${line.points.length} remaining.`);
}

function deleteManualCenterLine(id) {
  const idx = manualCenterLines.findIndex(c => c.id === id);
  if (idx === -1) return;
  const name = manualCenterLines[idx].name;
  clearManualCenterLineMarkers(id);
  removeManualCenterLineFromMap(id);
  manualCenterLines.splice(idx, 1);
  if (activeManualCenterLineId === id) activeManualCenterLineId = null;
  updateManualCenterLineList();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Deleted manual centerline "${name}".`);
}

function selectManualCenterLine(id) {
  if (activeAnnotationId && activeAnnotationId !== id) clearActiveAnnotation();
  if (activeConnectorId && activeConnectorId !== id) clearActiveConnector();
  if (activeManualCenterLineId && activeManualCenterLineId !== id) clearActiveManualCenterLine();
  selectedAutoCenterLineId = null;

  activeAnnotationId = null;
  activeConnectorId = null;
  activeManualCenterLineId = id;
  currentMode = 'centerline';
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  document.querySelector('[data-mode="centerline"]').classList.add('active');
  toggleModeUI();

  const line = manualCenterLines.find(c => c.id === id);
  if (line) {
    document.getElementById('centerline-name').value = line.name;
    line.activeBy = collaboratorName;
    touchManualCenterLine(line);
    setStatus(`Selected manual centerline "${line.name}" for editing.`);
  }
  renderDrawingLabels();
  renderCenterLines();
  scheduleAutoSave();
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  scrollSelectedListItem('manual-centerline', id);
}

function updateManualCenterLineList() {
  const list = document.getElementById('manual-centerline-list');
  const count = document.getElementById('manual-centerline-count');
  if (!list || !count) return;
  count.textContent = manualCenterLines.length;

  list.innerHTML = '';
  if (manualCenterLines.length === 0) {
    list.innerHTML = '<p class="hint">Use Center mode to draw custom yellow dotted lines.</p>';
    return;
  }

  manualCenterLines.forEach(line => {
    const item = document.createElement('div');
    item.className = `annotation-item${line.id === activeManualCenterLineId ? ' selected' : ''}`;
    item.dataset.manualCenterlineId = line.id;
    item.innerHTML = `
      <div class="annotation-dot center"></div>
      <div class="annotation-info" title="${line.name}">
        <div class="annotation-name">${line.name}</div>
        <div class="annotation-meta">${line.points.length} pts${line.updatedBy ? ` | ${line.updatedBy}` : ''}</div>
      </div>
      <button class="annotation-delete-btn" title="Delete">&times;</button>
    `;
    item.querySelector('.annotation-info').addEventListener('click', () => selectManualCenterLine(line.id));
    item.querySelector('.annotation-delete-btn').addEventListener('click', (e) => {
      e.stopPropagation();
      deleteManualCenterLine(line.id);
    });
    list.appendChild(item);
  });
}

function deleteAnnotation(id) {
  const idx = annotations.findIndex(a => a.id === id);
  if (idx === -1) return;

  const name = annotations[idx].name;
  clearMarkersForAnnotation(id);
  removeLineFromMap(id);
  annotations.splice(idx, 1);

  if (activeAnnotationId === id) activeAnnotationId = null;

  updateAnnotationList();
  renderCenterLines();
  renderDrawingLabels();
  scheduleAutoSave();
  setStatus(`Deleted "${name}".`);
}

function selectAnnotation(id) {
  if (activeConnectorId) clearActiveConnector();
  if (activeManualCenterLineId) clearActiveManualCenterLine();
  if (activeAnnotationId && activeAnnotationId !== id) clearActiveAnnotation();
  selectedAutoCenterLineId = null;
  activeAnnotationId = id;
  activeConnectorId = null;
  activeManualCenterLineId = null;
  const ann = annotations.find(a => a.id === id);
  if (ann) {
    currentMode = 'lane';
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    document.querySelector('[data-mode="lane"]').classList.add('active');
    toggleModeUI();
    document.getElementById('line-name').value = ann.name;
    ann.activeBy = collaboratorName;
    touchAnnotation(ann);
    setStatus(`Selected "${ann.name}" for editing. Click to add more points.`);
  }
  renderDrawingLabels();
  renderCenterLines();
  scheduleAutoSave();
  updateAnnotationList();
  updateConnectorList();
  updateManualCenterLineList();
  scrollSelectedListItem('annotation', id);
}

function updateAnnotationList() {
  const list = document.getElementById('annotation-list');
  const count = document.getElementById('annotation-count');
  count.textContent = annotations.length;

  list.innerHTML = '';
  annotations.forEach(ann => {
    const item = document.createElement('div');
    item.className = `annotation-item${ann.id === activeAnnotationId ? ' selected' : ''}`;
    item.dataset.annotationId = ann.id;

    item.innerHTML = `
      <div class="annotation-dot lane"></div>
      <div class="annotation-info" title="${ann.name}">
        <div class="annotation-name">${ann.name}</div>
        <div class="annotation-meta">${ann.points.length} pts${ann.updatedBy ? ` | ${ann.updatedBy}` : ''}</div>
      </div>
      <button class="annotation-delete-btn" title="Delete">&times;</button>
    `;

    item.querySelector('.annotation-info').addEventListener('click', () => selectAnnotation(ann.id));
    item.querySelector('.annotation-delete-btn').addEventListener('click', (e) => {
      e.stopPropagation();
      deleteAnnotation(ann.id);
    });

    list.appendChild(item);
  });
}

// --- Save / Load ---

async function saveAnnotations() {
  const data = createRouteDocument();

  try {
    const res = await fetch('/api/save', {
      method: 'POST',
      headers: syncHeaders(),
      body: JSON.stringify(data)
    });
    const result = await res.json();
    if (result.success) {
      setStatus(`Saved ${result.count} lane(s), ${connectors.length} connector(s), ${manualCenterLines.length} manual centerline(s), ${eraserPoints.length} eraser(s) to ${result.storage}`);
    } else {
      setStatus(`Save failed: ${result.error}`);
    }
  } catch (err) {
    setStatus(`Save error: ${err.message}`);
  }
}

function downloadAnnotationsJson() {
  const data = createRouteDocument();
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const ts = new Date().toISOString().slice(0, 19).replace(/[T:]/g, '-');
  const link = document.createElement('a');
  link.href = url;
  link.download = `followrtk-annotations-${ts}.json`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
  setStatus(`Downloaded ${annotations.length} lane(s), ${connectors.length} connector(s), ${manualCenterLines.length} manual centerline(s), ${eraserPoints.length} eraser(s)`);
}

async function autoLabelLanes() {
  const candidates = annotations.filter(ann => ann.type === 'lane' && ann.points.length >= 2);
  if (candidates.length === 0) {
    setStatus('No lane boundaries with points to label.');
    return;
  }

  setStatus(`Auto-labeling ${candidates.length} lane(s)...`);
  try {
    const res = await fetch('/api/autolabel', {
      method: 'POST',
      headers: syncHeaders(),
      body: JSON.stringify(createRouteDocument())
    });
    const result = await res.json();
    if (!res.ok) throw new Error(result.error || 'Auto-label failed');

    const byId = new Map((result.suggestions || []).map(s => [s.id, s]));
    let applied = 0;
    let found = 0;
    for (const ann of annotations) {
      const suggestion = byId.get(ann.id);
      if (!suggestion?.suggestedName) continue;
      found++;
      if (!suggestion.shouldApply || ann.name === suggestion.suggestedName) continue;
      ann.name = suggestion.suggestedName;
      touchAnnotation(ann);
      applied++;
    }

    updateAnnotationList();
    renderCenterLines();
    renderDrawingLabels();
    if (applied > 0) {
      await saveAnnotations();
      setStatus(`Auto-labeled ${applied} lane(s). Found suggestions for ${found} lane(s).`);
    } else {
      setStatus(`Found suggestions for ${found} lane(s), but no generic lane names needed updates.`);
    }
  } catch (err) {
    setStatus(`Auto-label error: ${err.message}`);
  }
}

async function loadAnnotations(options = {}) {
  try {
    const res = await fetch('/api/load', { headers: authHeaders() });
    const data = await res.json();
    if (!res.ok) {
      throw new Error(data.error || 'Failed to load routes');
    }

    annotations.forEach(ann => {
      clearMarkersForAnnotation(ann.id);
      removeLineFromMap(ann.id);
    });
    connectors.forEach(conn => {
      clearConnectorMarkers(conn.id);
      removeConnectorFromMap(conn.id);
    });
    manualCenterLines.forEach(line => {
      clearManualCenterLineMarkers(line.id);
      removeManualCenterLineFromMap(line.id);
    });
    clearCenterLines();
    clearManualCenterLineLayers();
    eraserMarkers.forEach(m => m.remove());
    eraserMarkers = [];

    annotations = data.annotations || [];
    connectors = data.connectors || [];
    manualCenterLines = data.manualCenterLines || [];
    suppressedAutoCenterLineIds = data.suppressedAutoCenterLineIds || [];
    eraserPoints = data.eraserPoints || [];
    activeAnnotationId = null;
    activeConnectorId = null;
    activeManualCenterLineId = null;
    selectedAutoCenterLineId = null;

    rebuildAllVisuals();
    updateAnnotationList();
    updateConnectorList();
    updateManualCenterLineList();
    setStatus(`${options.remote ? 'Loaded collaborator changes:' : 'Loaded'} ${annotations.length} lane(s), ${connectors.length} connector(s), ${manualCenterLines.length} manual centerline(s), ${eraserPoints.length} eraser(s)`);
  } catch (err) {
    setStatus(`Load error: ${err.message}`);
    if (err.message.toLowerCase().includes('password')) {
      sessionStorage.removeItem(PASSWORD_KEY);
      appPassword = '';
      showJoinOverlay('Wrong password.');
    }
  }
}

// --- UI ---

function toggleModeUI() {
  const laneControls = document.getElementById('lane-controls');
  const connectorControls = document.getElementById('connector-controls');
  const eraserControls = document.getElementById('eraser-controls');
  const centerlineControls = document.getElementById('centerline-controls');
  laneControls.classList.add('hidden');
  connectorControls.classList.add('hidden');
  eraserControls.classList.add('hidden');
  centerlineControls.classList.add('hidden');

  if (currentMode === 'lane') laneControls.classList.remove('hidden');
  else if (currentMode === 'connector') connectorControls.classList.remove('hidden');
  else if (currentMode === 'centerline') centerlineControls.classList.remove('hidden');
  else if (currentMode === 'eraser') eraserControls.classList.remove('hidden');
}

function setStatus(msg) {
  document.getElementById('status-bar').textContent = msg;
}

// --- Event listeners ---

document.getElementById('join-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  const nameInput = document.getElementById('collaborator-name');
  const passwordInput = document.getElementById('site-password');
  const errorEl = document.getElementById('join-error');

  collaboratorName = nameInput.value.trim();
  appPassword = passwordInput.value;
  if (!collaboratorName || !appPassword) {
    errorEl.textContent = 'Enter your name and password.';
    return;
  }

  localStorage.setItem(COLLABORATOR_NAME_KEY, collaboratorName);
  sessionStorage.setItem(PASSWORD_KEY, appPassword);

  const res = await fetch('/api/load', { headers: authHeaders() });
  if (!res.ok) {
    errorEl.textContent = 'Wrong password.';
    sessionStorage.removeItem(PASSWORD_KEY);
    appPassword = '';
    return;
  }

  hideJoinOverlay();
  if (!map) initMap();
  else loadAnnotations();
});

document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    let clearedActive = false;
    const hadSelectedAutoCenterLine = !!selectedAutoCenterLineId;
    selectedAutoCenterLineId = null;
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentMode = btn.dataset.mode;
    toggleModeUI();
    if (currentMode === 'eraser') {
      clearedActive = !!(clearActiveAnnotation() || clearActiveConnector() || clearActiveManualCenterLine());
      updateAnnotationList();
      updateConnectorList();
      updateManualCenterLineList();
      setStatus('Eraser mode: click near a center line to erase a section.');
    } else if (currentMode === 'connector') {
      clearedActive = !!(clearActiveAnnotation() || clearActiveManualCenterLine());
      updateAnnotationList();
      updateManualCenterLineList();
      setStatus('Connector mode: click "New Connector" then click near lane points to snap.');
    } else if (currentMode === 'centerline') {
      clearedActive = !!(clearActiveAnnotation() || clearActiveConnector());
      updateAnnotationList();
      updateConnectorList();
      setStatus('Center mode: click "New Center" then draw the yellow dotted route directly.');
    } else {
      clearedActive = !!(clearActiveConnector() || clearActiveManualCenterLine());
      updateConnectorList();
      updateManualCenterLineList();
      setStatus('Lane mode: click "New Line" to start drawing a lane boundary.');
    }
    if (clearedActive) {
      renderDrawingLabels();
      scheduleAutoSave();
    }
    if (hadSelectedAutoCenterLine) renderCenterLines();
  });
});

document.getElementById('eraser-radius').addEventListener('input', (e) => {
  eraserRadius = parseInt(e.target.value);
  document.getElementById('eraser-radius-val').textContent = `${eraserRadius}m`;
});

document.getElementById('line-name').addEventListener('input', (e) => {
  if (!activeAnnotationId) return;
  const ann = annotations.find(a => a.id === activeAnnotationId);
  if (ann) {
    ann.name = e.target.value.trim() || ann.name;
    touchAnnotation(ann);
    updateAnnotationList();
    renderCenterLines();
    scheduleAutoSave();
  }
});

document.getElementById('connector-name').addEventListener('input', (e) => {
  if (!activeConnectorId) return;
  const conn = connectors.find(c => c.id === activeConnectorId);
  if (conn) {
    conn.name = e.target.value.trim() || conn.name;
    touchConnector(conn);
    updateConnectorList();
    scheduleAutoSave();
  }
});

document.getElementById('centerline-name').addEventListener('input', (e) => {
  if (!activeManualCenterLineId) return;
  const line = manualCenterLines.find(c => c.id === activeManualCenterLineId);
  if (line) {
    line.name = e.target.value.trim() || line.name;
    touchManualCenterLine(line);
    updateManualCenterLineList();
    scheduleAutoSave();
  }
});

document.getElementById('btn-new').addEventListener('click', newLine);
document.getElementById('btn-finish').addEventListener('click', finishLine);
document.getElementById('btn-undo').addEventListener('click', undoPoint);
document.getElementById('btn-delete').addEventListener('click', () => {
  if (activeAnnotationId) {
    deleteAnnotation(activeAnnotationId);
  } else {
    setStatus('No line selected to delete.');
  }
});
document.getElementById('btn-new-connector').addEventListener('click', newConnector);
document.getElementById('btn-finish-connector').addEventListener('click', finishConnector);
document.getElementById('btn-undo-connector').addEventListener('click', undoConnectorPoint);
document.getElementById('btn-delete-connector').addEventListener('click', () => {
  if (activeConnectorId) {
    deleteConnector(activeConnectorId);
  } else {
    setStatus('No connector selected to delete.');
  }
});
document.getElementById('btn-new-centerline').addEventListener('click', newManualCenterLine);
document.getElementById('btn-finish-centerline').addEventListener('click', finishManualCenterLine);
document.getElementById('btn-undo-centerline').addEventListener('click', undoManualCenterLinePoint);
document.getElementById('btn-delete-centerline').addEventListener('click', () => {
  if (activeManualCenterLineId) {
    deleteManualCenterLine(activeManualCenterLineId);
  } else {
    setStatus('No manual centerline selected to delete.');
  }
});
document.getElementById('btn-undo-eraser').addEventListener('click', undoEraser);
document.getElementById('btn-clear-erasers').addEventListener('click', clearAllErasers);
document.getElementById('btn-save').addEventListener('click', saveAnnotations);
document.getElementById('btn-load').addEventListener('click', loadAnnotations);
document.getElementById('btn-download-json').addEventListener('click', downloadAnnotationsJson);
document.getElementById('btn-auto-label').addEventListener('click', autoLabelLanes);

document.getElementById('btn-save-gps').addEventListener('click', () => {
  if (gpsWs && gpsWs.readyState === WebSocket.OPEN) {
    const ts = new Date().toISOString().slice(0, 19).replace(/[T:]/g, '-');
    gpsWs.send(JSON.stringify({ type: 'save', filename: `lane_points_${ts}.json` }));
  } else {
    setStatus('GPS not connected.');
  }
});

document.getElementById('btn-clear-gps').addEventListener('click', () => {
  if (gpsWs && gpsWs.readyState === WebSocket.OPEN) {
    gpsWs.send(JSON.stringify({ type: 'clear' }));
  }
});

document.getElementById('btn-follow').addEventListener('click', () => {
  gpsFollowing = !gpsFollowing;
  document.getElementById('btn-follow').classList.toggle('active', gpsFollowing);
  if (gpsFollowing && gpsTrail.length > 0) {
    const last = gpsTrail[gpsTrail.length - 1];
    map.flyTo({ center: [last.lon, last.lat], zoom: 19 });
  }
  setStatus(gpsFollowing ? 'Following GPS. Drag map to stop.' : 'GPS follow off.');
});

document.getElementById('btn-clear-trail').addEventListener('click', clearGpsTrail);

initMap();
