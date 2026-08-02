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
  last_error?: string | null;
}

// A destination pushed in from a remote client (companion app via the
// Cloudflare tunnel). The UI drops the pin, lets useRoute plan the purple
// route, and — when autostart is set — drives that computed route. `seq`
// increments per message so repeating the same coordinate still triggers.
export interface RemoteRoute {
  lat: number;
  lng: number;
  autostart: boolean;
  seq: number;
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
  steer_trim_deg?: number;
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

// --- perception -------------------------------------------------------------
// One tracked object, in the CART frame: metres forward and metres right of the
// cart origin. Deliberately not lat/lon — the minimap answers "what is around
// me right now", which needs no map and no GPS fix, so it keeps working in
// exactly the conditions where the operator most wants it.
export interface PerceptionTrack {
  id: number;
  cls: string;
  group: 'vru' | 'vehicle' | string;
  forward_m: number;
  lateral_m: number;   // + = right of the cart
  vf_ms: number;       // velocity relative to the cart, forward component
  vl_ms: number;       // ... and rightward component
  speed_ms: number;
  width_m: number;
  radius_m: number;    // uncertainty + half-width: the blob the policy avoids
  confirmed: boolean;
  moving: boolean;
  coasting: boolean;   // predicted, not currently detected (occluded/blind zone)
  clipped: boolean;    // feet below the frame: range is an upper bound
  conflict: boolean;   // this track is limiting (or would limit) our speed
  conf: number;
}

export interface PerceptionConflict {
  track_id: number;
  cls: string;
  station_m: number;
  time_s: number;
  gap_m: number;
  closing_ms: number;
  v_safe_mph: number;
}

export interface PerceptionDecision {
  v_allowed_mph: number;
  reason: string;
  limiting_track_id: number | null;
  emergency: boolean;
  degraded: boolean;
  layer: 'clear' | 'nominal' | 'reflex' | 'degraded' | string;
  conflicts: PerceptionConflict[];
}

export interface PerceptionState {
  ts: number;
  // True when the decision is computed but NOT applied to the cart. The minimap
  // says so loudly: an operator must never mistake a proposal for the reason
  // the cart just slowed down.
  shadow: boolean;
  detector_hz: number;
  frame_age_s: number;
  range_m: number;             // outer ring of the minimap
  fov_deg: number;             // camera horizontal field of view
  blind_zone_m: number;        // nearest ground the camera can resolve
  corridor_half_w_m: number;
  reflex_range_m: number;
  stopping_distance_m: number;
  speed_mph: number;
  decision: PerceptionDecision;
  tracks: PerceptionTrack[];
}
