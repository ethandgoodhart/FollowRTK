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

// A center line the user drew/edited by hand in the editor. It overrides the
// auto-generated pair (the corresponding auto pair is listed in
// suppressedAutoCenterLineIds) and is the source of truth for that lane.
export interface RawManualCenterLine {
  id: string;
  name: string;
  type: 'manual-centerline';
  points: LatLng[];
}

export interface RawAnnotations {
  annotations: RawLane[];
  connectors: RawConnector[];
  eraserPoints: RawEraserPoint[];
  // Hand corrections from the editor (the `live` branch honors these).
  manualCenterLines?: RawManualCenterLine[];
  suppressedAutoCenterLineIds?: string[];
}

export interface CenterLine {
  name: string;
  type: 'lane' | 'connector';
  points: LatLng[];
  // Full lane width (m) between the two paired boundaries, for lane center
  // lines. Used to offset the route into the right lane (the yellow center line
  // is a divider, not the drive line). Absent/0 for connectors and hand-drawn
  // manual center lines (no boundary pair to measure).
  width?: number;
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

export interface NtripStatus {
  provider: string;
  label: string;
  connected: boolean;
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

// One object seen by the obstacle detector (obstacle/detector.py /status).
export interface ObstacleDetection {
  cls: string;
  conf: number;
  box: [number, number, number, number];  // pixel x1,y1,x2,y2 in the camera frame
  in_zone: boolean;   // zone mode: footprint overlaps the brake zone; predictive: collision course
  overlap: number;    // fraction of footprint inside the zone (zone mode)
  // Predictive mode extras (obstacle/predictive.py):
  id?: number;        // persistent track id
  x_m?: number;       // cart-relative ground position, meters (right / forward)
  z_m?: number;
  vx?: number | null; // cart-relative velocity, m/s
  vz?: number | null;
  collide?: boolean;  // on a collision course with our trajectory
  t_hit?: number | null;  // predicted seconds until it blocks our path
}

// Verdict from the camera obstacle detector. Zone mode gives a binary brake;
// predictive mode adds brake_fraction: how hard to brake (0..1, 1 = full).
export interface ObstacleStatus {
  brake: boolean;
  brake_fraction?: number;  // predictive mode: required decel / max decel
  emergency?: boolean;      // predictive mode: slam it now
  ego_speed_mps?: number;   // predictive mode: cart speed used for the math
  detections: ObstacleDetection[];
  video_t?: number;   // seconds into the eval video (absent on live camera)
  connected: boolean;
  t?: number;
  ended?: boolean;
}

// Live telemetry from the cart's path follower (cartlib.server "follow" msg).
export interface FollowState {
  active: boolean;          // true while a drive is running, false on follow_end
  phase: string;            // init | tracking | done | abort
  reason?: string;
  fix?: string | null;
  alpha?: number | null;    // cross-track correction angle (deg)
  steer_cmd?: number;       // desired steering angle (deg) — drives the orange line
  steering_actual_deg?: number | null;
  steering_target_deg?: number | null;
  max_speed_mph?: number;
  live_speed_mph?: number;
  lookahead_m?: number;
  steer_gain?: number;
  xtrack_gain?: number;
  max_steer_deg?: number;
  turn_slowdown?: number;
  gas?: number;
  brake?: number;
  xtrack_m?: number;
  xtrack_signed_m?: number | null;  // + = cart is left of path direction
  heading_deg?: number | null;      // estimated absolute cart heading (compass deg)
  heading_err_deg?: number | null;  // + = cart points left of the line
  heading_gain?: number;
  dist_to_goal_m?: number;
  armed?: boolean;
}
