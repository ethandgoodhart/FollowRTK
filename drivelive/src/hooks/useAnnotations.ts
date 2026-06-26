'use client';

import { useMemo } from 'react';
import { RawAnnotations, CenterLine, GraphNode, LatLng } from '@/lib/types';
import { computeCenterLine, haversineMeters } from '@/lib/geo';
import { buildGraph } from '@/lib/graph';

function findPairs(items: { name: string; points: LatLng[] }[]) {
  const byName: Record<string, typeof items> = {};
  for (const item of items) {
    if (!byName[item.name]) byName[item.name] = [];
    byName[item.name].push(item);
  }
  const pairs: { name: string; a: typeof items[0]; b: typeof items[0] }[] = [];
  for (const name of Object.keys(byName)) {
    const group = byName[name];
    for (let i = 0; i + 1 < group.length; i += 2) {
      if (group[i].points.length >= 2 && group[i + 1].points.length >= 2) {
        pairs.push({ name, a: group[i], b: group[i + 1] });
      }
    }
  }
  return pairs;
}

function nearestPoint(target: LatLng, lines: LatLng[][]): LatLng | null {
  let best: LatLng | null = null;
  let bestDist = 100;
  for (const line of lines) {
    for (const pt of line) {
      const d = haversineMeters(target, pt);
      if (d < bestDist) {
        bestDist = d;
        best = { lat: pt.lat, lng: pt.lng };
      }
    }
  }
  return best;
}

export function useAnnotations(raw: RawAnnotations) {
  return useMemo(() => {
    const lanePairs = findPairs(raw.annotations.filter((a) => a.type === 'lane'));
    const connPairs = findPairs(raw.connectors);

    const laneCenterLines: CenterLine[] = [];
    const laneCenterPoints: LatLng[][] = [];

    for (const pair of lanePairs) {
      const points = computeCenterLine(pair.a.points, pair.b.points);
      laneCenterLines.push({ name: pair.name, type: 'lane', points });
      laneCenterPoints.push(points);
    }

    const connCenterLines: CenterLine[] = [];
    for (const pair of connPairs) {
      const points = computeCenterLine(pair.a.points, pair.b.points);
      if (laneCenterPoints.length > 0) {
        const snapFirst = nearestPoint(points[0], laneCenterPoints);
        const snapLast = nearestPoint(points[points.length - 1], laneCenterPoints);
        if (snapFirst) points[0] = snapFirst;
        if (snapLast) points[points.length - 1] = snapLast;
      }
      connCenterLines.push({ name: pair.name, type: 'connector', points });
    }

    const allCenterLines = [...laneCenterLines, ...connCenterLines];
    const graph = buildGraph(allCenterLines);

    return {
      laneBoundaries: raw.annotations,
      connectorBoundaries: raw.connectors,
      laneCenterLines,
      connCenterLines,
      allCenterLines,
      graph,
    };
  }, [raw]);
}
