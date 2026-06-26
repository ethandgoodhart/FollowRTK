import { LatLng } from './types';

const R = 6371000;
const toRad = (d: number) => (d * Math.PI) / 180;
const toDeg = (r: number) => (r * 180) / Math.PI;

export function haversineMeters(a: LatLng, b: LatLng): number {
  const dLat = toRad(b.lat - a.lat);
  const dLng = toRad(b.lng - a.lng);
  const sinLat = Math.sin(dLat / 2);
  const sinLng = Math.sin(dLng / 2);
  const h = sinLat * sinLat + Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * sinLng * sinLng;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

export function bearing(a: LatLng, b: LatLng): number {
  const dLng = toRad(b.lng - a.lng);
  const lat1 = toRad(a.lat);
  const lat2 = toRad(b.lat);
  const y = Math.sin(dLng) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLng);
  return ((toDeg(Math.atan2(y, x)) % 360) + 360) % 360;
}

export function resampleLine(points: LatLng[], numSamples: number): LatLng[] {
  if (points.length < 2 || numSamples < 2) return points.slice();
  const dists = [0];
  for (let i = 1; i < points.length; i++) {
    dists.push(dists[i - 1] + haversineMeters(points[i - 1], points[i]));
  }
  const totalDist = dists[dists.length - 1];
  if (totalDist === 0) return points.slice();

  const result: LatLng[] = [];
  for (let s = 0; s < numSamples; s++) {
    const target = (s / (numSamples - 1)) * totalDist;
    let seg = 0;
    while (seg < dists.length - 2 && dists[seg + 1] < target) seg++;
    const segLen = dists[seg + 1] - dists[seg];
    const t = segLen > 0 ? (target - dists[seg]) / segLen : 0;
    result.push({
      lat: points[seg].lat + t * (points[seg + 1].lat - points[seg].lat),
      lng: points[seg].lng + t * (points[seg + 1].lng - points[seg].lng),
    });
  }
  return result;
}

export function computeCenterLine(line1: LatLng[], line2: LatLng[]): LatLng[] {
  const dSame =
    haversineMeters(line1[0], line2[0]) +
    haversineMeters(line1[line1.length - 1], line2[line2.length - 1]);
  const dRev =
    haversineMeters(line1[0], line2[line2.length - 1]) +
    haversineMeters(line1[line1.length - 1], line2[0]);
  const l2 = dRev < dSame ? [...line2].reverse() : line2;

  const n = Math.max(line1.length, l2.length, 20);
  const a = resampleLine(line1, n);
  const b = resampleLine(l2, n);
  return a.map((p, i) => ({
    lat: (p.lat + b[i].lat) / 2,
    lng: (p.lng + b[i].lng) / 2,
  }));
}

export function totalPolylineLength(points: LatLng[]): number {
  let d = 0;
  for (let i = 1; i < points.length; i++) {
    d += haversineMeters(points[i - 1], points[i]);
  }
  return d;
}

export function nearestPointOnPolyline(
  point: LatLng,
  polyline: LatLng[]
): { point: LatLng; index: number; distance: number; fraction: number } {
  let bestDist = Infinity;
  let bestPoint: LatLng = polyline[0];
  let bestIndex = 0;
  let bestFraction = 0;

  for (let i = 0; i < polyline.length - 1; i++) {
    const a = polyline[i];
    const b = polyline[i + 1];
    const dx = b.lng - a.lng;
    const dy = b.lat - a.lat;
    const lenSq = dx * dx + dy * dy;
    let t = 0;
    if (lenSq > 0) {
      t = Math.max(0, Math.min(1, ((point.lng - a.lng) * dx + (point.lat - a.lat) * dy) / lenSq));
    }
    const proj: LatLng = { lat: a.lat + t * dy, lng: a.lng + t * dx };
    const d = haversineMeters(point, proj);
    if (d < bestDist) {
      bestDist = d;
      bestPoint = proj;
      bestIndex = i;
      bestFraction = t;
    }
  }

  return { point: bestPoint, index: bestIndex, distance: bestDist, fraction: bestFraction };
}
