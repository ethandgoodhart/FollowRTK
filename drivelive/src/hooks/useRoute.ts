'use client';

import { useReducer, useCallback, useMemo, useEffect } from 'react';
import { LatLng, GpsPosition, GraphNode, RouteState } from '@/lib/types';
import { totalPolylineLength } from '@/lib/geo';
import { findNearestNode, dijkstra } from '@/lib/graph';
import { snapToRoute, computeRouteProgress, computeEta } from '@/lib/route-tracking';

type Action =
  | { type: 'SET_SELECTING'; selecting: 'start' | 'end' | 'none' }
  | { type: 'SET_START'; point: LatLng }
  | { type: 'SET_END'; point: LatLng }
  | { type: 'SET_PATH'; path: LatLng[]; totalDistance: number }
  | { type: 'UPDATE_PROGRESS'; distanceRemaining: number; progress: number; eta: number | null; nearestRoutePoint: LatLng }
  | { type: 'CLEAR' };

const initialState: RouteState = {
  startPoint: null,
  endPoint: null,
  path: [],
  totalDistance: 0,
  distanceRemaining: 0,
  progress: 0,
  eta: null,
  nearestRoutePoint: null,
  selecting: 'none',
};

function reducer(state: RouteState, action: Action): RouteState {
  switch (action.type) {
    case 'SET_SELECTING':
      return { ...state, selecting: action.selecting };
    case 'SET_START':
      return { ...state, startPoint: action.point, selecting: 'none', path: [], totalDistance: 0 };
    case 'SET_END':
      return { ...state, endPoint: action.point, selecting: 'none' };
    case 'SET_PATH':
      return { ...state, path: action.path, totalDistance: action.totalDistance, distanceRemaining: action.totalDistance, progress: 0 };
    case 'UPDATE_PROGRESS':
      return { ...state, distanceRemaining: action.distanceRemaining, progress: action.progress, eta: action.eta, nearestRoutePoint: action.nearestRoutePoint };
    case 'CLEAR':
      return initialState;
    default:
      return state;
  }
}

export function useRoute(
  graph: Map<string, GraphNode>,
  gpsPosition: GpsPosition | null,
  speed: number
) {
  const [state, dispatch] = useReducer(reducer, initialState);

  const selectStart = useCallback(() => dispatch({ type: 'SET_SELECTING', selecting: 'start' }), []);
  const selectEnd = useCallback(() => dispatch({ type: 'SET_SELECTING', selecting: 'end' }), []);
  const clearRoute = useCallback(() => dispatch({ type: 'CLEAR' }), []);

  const handleMapClick = useCallback(
    (latlng: LatLng) => {
      if (state.selecting === 'none') return;
      const nodeId = findNearestNode(latlng, graph);
      if (!nodeId) return;
      const node = graph.get(nodeId);
      if (!node) return;
      const point = { lat: node.lat, lng: node.lng };

      if (state.selecting === 'start') {
        dispatch({ type: 'SET_START', point });
      } else {
        dispatch({ type: 'SET_END', point });
      }
    },
    [state.selecting, graph]
  );

  // Compute route when both points are set
  const routePath = useMemo(() => {
    if (!state.startPoint || !state.endPoint) return null;
    const startId = findNearestNode(state.startPoint, graph);
    const endId = findNearestNode(state.endPoint, graph);
    if (!startId || !endId) return null;
    return dijkstra(graph, startId, endId);
  }, [state.startPoint, state.endPoint, graph]);

  useEffect(() => {
    if (routePath && routePath.length >= 2) {
      dispatch({ type: 'SET_PATH', path: routePath, totalDistance: totalPolylineLength(routePath) });
    }
  }, [routePath]);

  // Track progress along route
  useEffect(() => {
    if (!gpsPosition || state.path.length < 2) return;
    const pos = { lat: gpsPosition.lat, lng: gpsPosition.lon };
    const snap = snapToRoute(pos, state.path);
    if (!snap) return;
    const { distRemaining, progress } = computeRouteProgress(snap.segmentIndex, snap.fraction, state.path);
    const eta = computeEta(distRemaining, speed);
    dispatch({
      type: 'UPDATE_PROGRESS',
      distanceRemaining: distRemaining,
      progress,
      eta,
      nearestRoutePoint: snap.nearestPoint,
    });
  }, [gpsPosition, state.path, speed]);

  return {
    ...state,
    selectStart,
    selectEnd,
    handleMapClick,
    clearRoute,
  };
}
