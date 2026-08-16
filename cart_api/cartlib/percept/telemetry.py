"""
cartlib.percept.telemetry — what perception tells the operator.

One function, one dict, one place to change when the minimap wants a new field.
Keeping it out of ``policy`` means the safety code never grows a JSON opinion,
and keeping it out of ``server`` means the payload can be unit-tested without a
websocket.

The payload is deliberately in the CART frame rather than lat/lon. The minimap
is a "what is around me right now" instrument, and answering that in metres
forward and metres right needs no map, no projection, and no GPS fix -- so it
keeps working in exactly the situations where the operator most wants it.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

from .policy import (
    MPS_TO_MPH, PolicyConfig, SpeedDecision, stopping_distance_m,
)
from .track import EgoPose, Track


def track_payload(t: Track, ego: EgoPose, conflict_ids: frozenset) -> dict:
    """One track, in cart-relative terms the minimap can draw directly."""
    # Rotate the world-frame velocity into the cart frame so the arrow on the
    # minimap points where the object is going RELATIVE TO US -- which is the
    # question an operator glancing at it is actually asking.
    h = math.radians(ego.heading_deg)
    s, c = math.sin(h), math.cos(h)
    vf = t.vx * s + t.vy * c
    vl = t.vx * c - t.vy * s
    return {
        "id": t.id,
        "cls": t.cls,
        "group": t.group,
        "forward_m": round(t.last_forward_m, 2),
        "lateral_m": round(t.last_lateral_m, 2),
        "vf_ms": round(vf, 2),
        "vl_ms": round(vl, 2),
        "speed_ms": round(t.speed_ms, 2),
        "width_m": round(t.width_m, 2),
        "radius_m": round(t.radius_at(0.0), 2),
        "confirmed": t.confirmed,
        "moving": t.is_moving,
        "coasting": t.coast_s > 0.0,
        "clipped": t.clipped_bottom,
        "conflict": t.id in conflict_ids,
        "conf": round(t.conf, 2),
    }


def perception_payload(tracks: Sequence[Track], decision: SpeedDecision,
                       ego: EgoPose, cfg: Optional[PolicyConfig] = None,
                       detector_hz: float = 0.0, frame_age_s: float = 0.0,
                       ts: float = 0.0, shadow: bool = True,
                       fov_deg: float = 0.0,
                       height_m: Optional[float] = None,
                       pitch_deg: Optional[float] = None) -> dict:
    """The whole perception state, ready for ``json.dumps``.

    ``shadow`` says whether this decision is actually being applied to the cart
    or merely computed alongside it. The minimap shows that prominently: an
    operator watching a shadow-mode run needs to know the numbers on screen are
    a proposal, not the reason the cart just slowed down.
    """
    cfg = cfg or PolicyConfig()
    conflict_ids = frozenset(c.track_id for c in decision.conflicts)
    payload = {
        "ts": ts,
        "shadow": shadow,
        "detector_hz": round(detector_hz, 1),
        "frame_age_s": round(frame_age_s, 3),
        # Geometry the minimap draws as fixed furniture.
        "range_m": cfg.horizon_m,
        # Drawn as a wedge, so the operator can see at a glance that a blank
        # sector is outside the camera's view rather than known to be empty.
        "fov_deg": round(fov_deg, 1),
        "blind_zone_m": cfg.blind_zone_m,
        "corridor_half_w_m": cfg.cart_half_width_m + cfg.corridor_margin_m,
        "reflex_range_m": cfg.reflex_range_m,
        "stopping_distance_m": round(stopping_distance_m(ego.speed_ms, cfg), 2),
        "speed_mph": round(ego.speed_ms * MPS_TO_MPH, 2),
        "decision": decision.to_dict(),
        "tracks": [track_payload(t, ego, conflict_ids) for t in tracks],
    }
    if height_m is not None:
        payload["height_m"] = round(float(height_m), 3)
    if pitch_deg is not None:
        payload["pitch_deg"] = round(float(pitch_deg), 2)
    return payload
