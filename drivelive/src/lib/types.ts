export interface LatLng {
  lat: number;
  lng: number;
}

export interface RawLane {
  id: string;
  name: string;
  type: 'lane';
  points: LatLng[];
}

export interface RawConnector {
  id: string;
  name: string;
  type: 'connector';
  points: LatLng[];
}

export interface RawEraserPoint {
  id: string;
  lat: number;
  lng: number;
  radius: number;
}

export interface RawAnnotations {
  annotations: RawLane[];
  connectors: RawConnector[];
  eraserPoints: RawEraserPoint[];
}

export interface CenterLine {
  name: string;
  type: 'lane' | 'connector';
  points: LatLng[];
}

export interface GraphNode {
  id: string;
  lat: number;
  lng: number;
  neighbors: GraphEdge[];
}

export interface GraphEdge {
  nodeId: string;
  distance: number;
}

export interface GpsPosition {
  lat: number;
  lon: number;
  fix: string;
  fix_code: number;
  sats: number;
  hdop: number;
  alt: number;
  ts: number;
  datetime: string;
  utc_time: string;
}

export interface RouteState {
  startPoint: LatLng | null;
  endPoint: LatLng | null;
  path: LatLng[];
  totalDistance: number;
  distanceRemaining: number;
  progress: number;
  eta: number | null;
  nearestRoutePoint: LatLng | null;
  selecting: 'start' | 'end' | 'none';
}
