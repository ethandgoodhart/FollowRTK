let MAPBOX_TOKEN = '';

const COLORS = {
  lane: '#44ff88',
  center: '#ffcc00',
  eraser: '#ff8800'
};

let map;
let annotations = [];
let eraserPoints = [];
let activeAnnotationId = null;
let currentMode = 'lane';
let pointMarkers = [];
let eraserMarkers = [];
let lineCounter = 0;
let eraserRadius = 8;

// --- Geo math ---

function haversineMeters(a, b) {
  const R = 6371000;
  const toRad = d => d * Math.PI / 180;
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

function computeCenterLine(line1, line2) {
  const n = Math.max(line1.length, line2.length, 20);
  const a = resampleLine(line1, n);
  const b = resampleLine(line2, n);
  return a.map((p, i) => ({
    lat: (p.lat + b[i].lat) / 2,
    lng: (p.lng + b[i].lng) / 2
  }));
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

// --- Pair detection: group by road name, pair every 2 lanes ---

function findPairs() {
  const byName = {};
  for (const ann of annotations) {
    if (!byName[ann.name]) byName[ann.name] = [];
    byName[ann.name].push(ann);
  }
  const pairs = [];
  for (const name of Object.keys(byName)) {
    const lanes = byName[name];
    for (let i = 0; i + 1 < lanes.length; i += 2) {
      if (lanes[i].points.length >= 2 && lanes[i + 1].points.length >= 2) {
        pairs.push({ name, a: lanes[i], b: lanes[i + 1] });
      }
    }
  }
  return pairs;
}

// --- Center line rendering ---

let centerLineSources = new Set();

function clearCenterLines() {
  for (const id of centerLineSources) {
    if (map.getLayer(`layer-center-${id}`)) map.removeLayer(`layer-center-${id}`);
    if (map.getSource(`source-center-${id}`)) map.removeSource(`source-center-${id}`);
  }
  centerLineSources.clear();
}

function renderCenterLines() {
  clearCenterLines();
  const pairs = findPairs();
  const listEl = document.getElementById('center-line-list');
  listEl.innerHTML = '';

  if (pairs.length === 0) {
    listEl.innerHTML = '<p class="hint">Draw 2 lane boundaries with the same road name to auto-generate a center line.</p>';
    return;
  }

  pairs.forEach((pair, idx) => {
    const center = computeCenterLine(pair.a.points, pair.b.points);
    const segments = applyCenterLineErasure(center, eraserPoints, eraserRadius);

    const features = segments.map(seg => ({
      type: 'Feature',
      properties: {},
      geometry: {
        type: 'LineString',
        coordinates: seg.map(p => [p.lng, p.lat])
      }
    }));

    const sourceId = `pair-${idx}`;
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
        'line-color': COLORS.center,
        'line-width': 3,
        'line-dasharray': [4, 3],
        'line-opacity': 0.85
      },
      layout: { 'line-cap': 'round', 'line-join': 'round' }
    });

    const erasedCount = eraserPoints.filter(ep =>
      center.some(cp => haversineMeters(cp, ep) < (ep.radius || eraserRadius) + 5)
    ).length;

    const item = document.createElement('div');
    item.className = 'center-line-item';
    item.innerHTML = `
      <div class="annotation-dot center"></div>
      <div class="annotation-info">
        <div class="annotation-name">${pair.name}</div>
        <div class="annotation-meta">auto${erasedCount > 0 ? ` | ${erasedCount} eraser(s)` : ''}</div>
      </div>
    `;
    listEl.appendChild(item);
  });
}

// --- Map ---

async function initMap() {
  const config = await fetch('/api/config').then(r => r.json());
  MAPBOX_TOKEN = config.mapboxToken;
  mapboxgl.accessToken = MAPBOX_TOKEN;

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
    loadAnnotations().then(() => {
      if (annotations.length === 0) {
        setStatus('Click "New Line" to start annotating lane boundaries.');
      }
    });
  });
}

function onMapClick(e) {
  const point = { lat: e.lngLat.lat, lng: e.lngLat.lng };

  if (currentMode === 'eraser') {
    const ep = { id: `eraser-${Date.now()}`, lat: point.lat, lng: point.lng, radius: eraserRadius };
    eraserPoints.push(ep);
    addEraserMarker(ep);
    renderCenterLines();
    setStatus(`Placed eraser (${eraserRadius}m radius). ${eraserPoints.length} total.`);
    return;
  }

  if (!activeAnnotationId) {
    setStatus('Click "New Line" first to start a new annotation.');
    return;
  }

  const annotation = annotations.find(a => a.id === activeAnnotationId);
  if (!annotation) return;

  annotation.points.push(point);
  addPointMarker(point, annotation.points.length - 1, annotation.id);
  updateLine(annotation);
  renderCenterLines();
  updateAnnotationList();
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

  marker.on('dragend', () => {
    const lngLat = marker.getLngLat();
    const ann = annotations.find(a => a.id === annotationId);
    if (ann && ann.points[marker._pointIndex]) {
      ann.points[marker._pointIndex] = { lat: lngLat.lat, lng: lngLat.lng };
      updateLine(ann);
      renderCenterLines();
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
  setStatus(`Deleted eraser. ${eraserPoints.length} remaining.`);
}

function clearAllErasers() {
  eraserMarkers.forEach(m => m.remove());
  eraserMarkers = [];
  eraserPoints = [];
  renderCenterLines();
  setStatus('Cleared all erasers.');
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
    properties: {},
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
  eraserMarkers.forEach(m => m.remove());
  eraserMarkers = [];

  annotations.forEach(ann => {
    ann.points.forEach((pt, i) => addPointMarker(pt, i, ann.id));
    updateLine(ann);
  });

  eraserPoints.forEach(ep => addEraserMarker(ep));
  renderCenterLines();
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

  const annotation = {
    id: `ann-${Date.now()}-${lineCounter}`,
    name: roadName,
    type: 'lane',
    points: []
  };

  annotations.push(annotation);
  activeAnnotationId = annotation.id;
  updateAnnotationList();
  setStatus(`Started "${roadName}". Click on the map to add points.`);
}

function finishLine() {
  if (!activeAnnotationId) {
    setStatus('No active line to finish.');
    return;
  }
  const ann = annotations.find(a => a.id === activeAnnotationId);
  const name = ann ? ann.name : '';
  activeAnnotationId = null;
  updateAnnotationList();
  renderCenterLines();
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
  clearMarkersForAnnotation(annotation.id);
  annotation.points.forEach((pt, i) => addPointMarker(pt, i, annotation.id));
  updateLine(annotation);
  renderCenterLines();
  updateAnnotationList();
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
  setStatus(`Deleted "${name}".`);
}

function selectAnnotation(id) {
  activeAnnotationId = id;
  const ann = annotations.find(a => a.id === id);
  if (ann) {
    currentMode = 'lane';
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    document.querySelector('[data-mode="lane"]').classList.add('active');
    toggleModeUI();
    setStatus(`Selected "${ann.name}" for editing. Click to add more points.`);
  }
  updateAnnotationList();
}

function updateAnnotationList() {
  const list = document.getElementById('annotation-list');
  const count = document.getElementById('annotation-count');
  count.textContent = annotations.length;

  list.innerHTML = '';
  annotations.forEach(ann => {
    const item = document.createElement('div');
    item.className = `annotation-item${ann.id === activeAnnotationId ? ' selected' : ''}`;

    item.innerHTML = `
      <div class="annotation-dot lane"></div>
      <div class="annotation-info" title="${ann.name}">
        <div class="annotation-name">${ann.name}</div>
        <div class="annotation-meta">${ann.points.length} pts</div>
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
  const data = {
    annotations: annotations.map(ann => ({
      id: ann.id,
      name: ann.name,
      type: ann.type,
      points: ann.points.map(p => ({ lat: p.lat, lng: p.lng }))
    })),
    eraserPoints: eraserPoints.map(ep => ({
      id: ep.id,
      lat: ep.lat,
      lng: ep.lng,
      radius: ep.radius
    })),
    metadata: {
      savedAt: new Date().toISOString(),
      campus: 'Stanford University',
      project: 'FollowRTK Self-Driving Golf Cart'
    }
  };

  try {
    const res = await fetch('/api/save', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    });
    const result = await res.json();
    if (result.success) {
      setStatus(`Saved ${result.count} lane(s) + ${eraserPoints.length} eraser(s)`);
    } else {
      setStatus(`Save failed: ${result.error}`);
    }
  } catch (err) {
    setStatus(`Save error: ${err.message}`);
  }
}

async function loadAnnotations() {
  try {
    const res = await fetch('/api/load');
    const data = await res.json();

    annotations.forEach(ann => {
      clearMarkersForAnnotation(ann.id);
      removeLineFromMap(ann.id);
    });
    clearCenterLines();
    eraserMarkers.forEach(m => m.remove());
    eraserMarkers = [];

    annotations = data.annotations || [];
    eraserPoints = data.eraserPoints || [];
    activeAnnotationId = null;

    rebuildAllVisuals();
    updateAnnotationList();
    setStatus(`Loaded ${annotations.length} lane(s) + ${eraserPoints.length} eraser(s)`);
  } catch (err) {
    setStatus(`Load error: ${err.message}`);
  }
}

// --- UI ---

function toggleModeUI() {
  const laneControls = document.getElementById('lane-controls');
  const eraserControls = document.getElementById('eraser-controls');
  if (currentMode === 'eraser') {
    laneControls.classList.add('hidden');
    eraserControls.classList.remove('hidden');
  } else {
    laneControls.classList.remove('hidden');
    eraserControls.classList.add('hidden');
  }
}

function setStatus(msg) {
  document.getElementById('status-bar').textContent = msg;
}

// --- Event listeners ---

document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentMode = btn.dataset.mode;
    toggleModeUI();
    if (currentMode === 'eraser') {
      activeAnnotationId = null;
      updateAnnotationList();
      setStatus('Eraser mode: click near a center line to erase a section.');
    } else {
      setStatus('Lane mode: click "New Line" to start drawing a lane boundary.');
    }
  });
});

document.getElementById('eraser-radius').addEventListener('input', (e) => {
  eraserRadius = parseInt(e.target.value);
  document.getElementById('eraser-radius-val').textContent = `${eraserRadius}m`;
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
document.getElementById('btn-undo-eraser').addEventListener('click', undoEraser);
document.getElementById('btn-clear-erasers').addEventListener('click', clearAllErasers);
document.getElementById('btn-save').addEventListener('click', saveAnnotations);
document.getElementById('btn-load').addEventListener('click', loadAnnotations);

initMap();
