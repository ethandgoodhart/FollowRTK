import { LatLng, CenterLine, GraphNode } from './types';
import { haversineMeters } from './geo';

function nodeId(p: LatLng): string {
  return `${p.lat.toFixed(8)},${p.lng.toFixed(8)}`;
}

function getOrCreateNode(graph: Map<string, GraphNode>, p: LatLng): GraphNode {
  const id = nodeId(p);
  let node = graph.get(id);
  if (!node) {
    node = { id, lat: p.lat, lng: p.lng, neighbors: [] };
    graph.set(id, node);
  }
  return node;
}

function addEdge(a: GraphNode, b: GraphNode, distance: number) {
  if (!a.neighbors.some((e) => e.nodeId === b.id)) {
    a.neighbors.push({ nodeId: b.id, distance });
  }
  if (!b.neighbors.some((e) => e.nodeId === a.id)) {
    b.neighbors.push({ nodeId: a.id, distance });
  }
}

export function buildGraph(centerLines: CenterLine[]): Map<string, GraphNode> {
  const graph = new Map<string, GraphNode>();

  for (const cl of centerLines) {
    for (let i = 0; i < cl.points.length - 1; i++) {
      const a = getOrCreateNode(graph, cl.points[i]);
      const b = getOrCreateNode(graph, cl.points[i + 1]);
      const dist = haversineMeters(cl.points[i], cl.points[i + 1]);
      addEdge(a, b, dist);
    }
  }

  return graph;
}

export function findNearestNode(point: LatLng, graph: Map<string, GraphNode>, maxDist = 50): string | null {
  let bestId: string | null = null;
  let bestDist = maxDist;

  for (const node of graph.values()) {
    const d = haversineMeters(point, { lat: node.lat, lng: node.lng });
    if (d < bestDist) {
      bestDist = d;
      bestId = node.id;
    }
  }

  return bestId;
}

export function dijkstra(
  graph: Map<string, GraphNode>,
  startId: string,
  endId: string
): LatLng[] {
  const dist = new Map<string, number>();
  const prev = new Map<string, string>();
  const visited = new Set<string>();

  for (const id of graph.keys()) {
    dist.set(id, Infinity);
  }
  dist.set(startId, 0);

  while (true) {
    let u: string | null = null;
    let uDist = Infinity;
    for (const [id, d] of dist) {
      if (!visited.has(id) && d < uDist) {
        u = id;
        uDist = d;
      }
    }
    if (u === null || u === endId) break;
    visited.add(u);

    const node = graph.get(u);
    if (!node) break;

    for (const edge of node.neighbors) {
      if (visited.has(edge.nodeId)) continue;
      const alt = uDist + edge.distance;
      if (alt < (dist.get(edge.nodeId) ?? Infinity)) {
        dist.set(edge.nodeId, alt);
        prev.set(edge.nodeId, u);
      }
    }
  }

  if (!prev.has(endId) && startId !== endId) return [];

  const path: LatLng[] = [];
  let current: string | undefined = endId;
  while (current) {
    const node = graph.get(current);
    if (node) path.unshift({ lat: node.lat, lng: node.lng });
    current = prev.get(current);
  }

  return path;
}
