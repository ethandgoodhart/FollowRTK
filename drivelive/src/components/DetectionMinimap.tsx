'use client';

import { useMemo, useState } from 'react';
import { PerceptionState, PerceptionTrack } from '@/lib/types';

/**
 * A cart-frame radar: what perception currently believes is around the cart,
 * and which of it is limiting the speed.
 *
 * Two design decisions worth knowing about:
 *
 *  - It draws UNCERTAINTY, not points. Each track is a disc sized by the
 *    radius the policy actually avoids, so a badly-ranged object at 25 m looks
 *    as vague as it is. A minimap of confident dots would imply a precision
 *    monocular ranging does not have, and the operator would learn to trust it
 *    more than it deserves.
 *
 *  - It draws what it CANNOT see: the field-of-view wedge and the near blind
 *    zone are shaded, so an empty sector reads as "not looked at" rather than
 *    "known clear". Most of the surprising things this system does are
 *    explained by one of those two regions.
 */

const CLS_COLOR: Record<string, string> = {
  person: '#38bdf8',
  bicycle: '#22d3ee',
  motorcycle: '#22d3ee',
  dog: '#a78bfa',
  car: '#fbbf24',
  truck: '#fb923c',
  bus: '#fb923c',
};

const LAYER_STYLE: Record<string, { label: string; cls: string }> = {
  clear: { label: 'CLEAR', cls: 'bg-emerald-500/20 text-emerald-300 ring-emerald-500/40' },
  nominal: { label: 'SLOWING', cls: 'bg-amber-500/20 text-amber-300 ring-amber-500/40' },
  reflex: { label: 'REFLEX STOP', cls: 'bg-red-500/25 text-red-300 ring-red-500/50' },
  degraded: { label: 'DEGRADED', cls: 'bg-fuchsia-500/20 text-fuchsia-300 ring-fuchsia-500/40' },
};

const SIZE = 260;                       // svg viewport, px
const C = SIZE / 2;                     // centre
const PAD = 14;                         // room for the outer ring label
const R = C - PAD;                      // outer ring radius, px

/** Rings chosen to bracket the distances that matter: reflex, stop, horizon. */
function ringsFor(rangeM: number): number[] {
  return [5, 10, 20, 30, 40].filter((r) => r <= rangeM);
}

export default function DetectionMinimap({ perception }: { perception: PerceptionState | null }) {
  const [open, setOpen] = useState(true);

  const p = perception;
  const rangeM = p?.range_m && p.range_m > 0 ? p.range_m : 30;
  const toPx = useMemo(() => (m: number) => (m / rangeM) * R, [rangeM]);

  // Cart frame -> svg. Forward is up (-y in svg), lateral-right is +x.
  const at = (fwd: number, lat: number) => ({ x: C + toPx(lat), y: C - toPx(fwd) });

  const stale = !p || (Date.now() / 1000 - p.ts) > 2.0;
  const layer = LAYER_STYLE[p?.decision.layer ?? 'clear'] ?? LAYER_STYLE.clear;

  if (!open) {
    return (
      <button
        onClick={() => setOpen(true)}
        className="absolute bottom-3 left-3 z-10 rounded-lg bg-neutral-900/85 px-3 py-2 text-xs
                   font-medium text-neutral-200 ring-1 ring-neutral-700 backdrop-blur"
      >
        detections {p ? `(${p.tracks.length})` : ''}
      </button>
    );
  }

  const fov = p?.fov_deg ?? 0;
  const halfFov = (fov / 2) * (Math.PI / 180);
  // Wedge covering everything the camera CANNOT see, drawn as two arcs.
  const fovEdgeL = { x: C - R * Math.sin(halfFov), y: C - R * Math.cos(halfFov) };
  const fovEdgeR = { x: C + R * Math.sin(halfFov), y: C - R * Math.cos(halfFov) };

  return (
    <div className="absolute bottom-3 left-3 z-10 w-[286px] rounded-xl bg-neutral-900/85
                    p-3 text-neutral-200 ring-1 ring-neutral-700 backdrop-blur">
      {/* header ------------------------------------------------------------ */}
      <div className="mb-2 flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <span className={`rounded px-1.5 py-0.5 text-[10px] font-bold tracking-wide ring-1 ${layer.cls}`}>
            {layer.label}
          </span>
          {p?.shadow && (
            <span className="rounded bg-neutral-700/60 px-1.5 py-0.5 text-[10px] font-bold
                             tracking-wide text-neutral-300 ring-1 ring-neutral-600">
              SHADOW
            </span>
          )}
        </div>
        <button onClick={() => setOpen(false)} className="text-xs text-neutral-500 hover:text-neutral-300">
          hide
        </button>
      </div>

      {/* radar ------------------------------------------------------------- */}
      <svg viewBox={`0 0 ${SIZE} ${SIZE}`} className="w-full">
        <defs>
          <marker id="mm-arrow" viewBox="0 0 8 8" refX="7" refY="4"
                  markerWidth="4" markerHeight="4" orient="auto-start-reverse">
            <path d="M 0 1 L 8 4 L 0 7 z" fill="context-stroke" />
          </marker>
        </defs>

        {/* everything outside the camera's field of view */}
        {fov > 0 && fov < 359 && (
          <path
            d={`M ${C} ${C} L ${fovEdgeL.x} ${fovEdgeL.y} A ${R} ${R} 0 1 0 ${fovEdgeR.x} ${fovEdgeR.y} Z`}
            fill="rgba(0,0,0,0.38)"
          />
        )}

        {/* range rings */}
        {ringsFor(rangeM).map((m) => (
          <g key={m}>
            <circle cx={C} cy={C} r={toPx(m)} fill="none" stroke="#3f3f46" strokeWidth={1} />
            <text x={C + 3} y={C - toPx(m) + 10} fontSize={8} fill="#71717a">{m}m</text>
          </g>
        ))}
        <circle cx={C} cy={C} r={R} fill="none" stroke="#52525b" strokeWidth={1.5} />

        {/* near blind zone: the camera cannot see the ground in here at all */}
        {p && p.blind_zone_m > 0 && (
          <circle cx={C} cy={C} r={toPx(p.blind_zone_m)}
                  fill="rgba(239,68,68,0.10)" stroke="rgba(239,68,68,0.45)"
                  strokeWidth={1} strokeDasharray="3 3" />
        )}

        {/* how much room we currently need to stop */}
        {p && p.stopping_distance_m > 0 && p.stopping_distance_m < rangeM && (
          <>
            <line x1={C - 7} y1={C - toPx(p.stopping_distance_m)}
                  x2={C + 7} y2={C - toPx(p.stopping_distance_m)}
                  stroke="#f87171" strokeWidth={1.5} />
            <text x={C + 10} y={C - toPx(p.stopping_distance_m) + 3}
                  fontSize={8} fill="#f87171">
              stop {p.stopping_distance_m.toFixed(1)}m
            </text>
          </>
        )}

        {/* the cart */}
        <polygon points={`${C},${C - 9} ${C - 6},${C + 6} ${C + 6},${C + 6}`}
                 fill="#e4e4e7" />

        {/* tracks */}
        {p?.tracks.map((t) => <TrackGlyph key={t.id} t={t} at={at} toPx={toPx} />)}

        {stale && (
          <text x={C} y={C + 4} fontSize={11} fill="#a1a1aa" textAnchor="middle">
            no perception feed
          </text>
        )}
      </svg>

      {/* footer ------------------------------------------------------------ */}
      <div className="mt-2 space-y-1">
        <div className="flex items-center justify-between text-xs">
          <span className="text-neutral-400">allowed</span>
          <span className="font-mono font-semibold text-neutral-100">
            {p ? `${p.decision.v_allowed_mph.toFixed(1)} mph` : '–'}
          </span>
        </div>
        <div className="flex items-center justify-between text-[11px] text-neutral-500">
          <span>{p ? `${p.tracks.length} tracked` : 'no data'}</span>
          <span className="font-mono">
            {p ? `${p.detector_hz.toFixed(0)} Hz · ${(p.frame_age_s * 1000).toFixed(0)} ms` : ''}
          </span>
        </div>
        {p?.decision.reason && p.decision.reason !== 'clear' && (
          <p className="pt-0.5 text-[11px] leading-snug text-neutral-300">{p.decision.reason}</p>
        )}
      </div>
    </div>
  );
}

/** One track: uncertainty disc, relative-velocity arrow, and its label. */
function TrackGlyph({ t, at, toPx }: {
  t: PerceptionTrack;
  at: (f: number, l: number) => { x: number; y: number };
  toPx: (m: number) => number;
}) {
  const { x, y } = at(t.forward_m, t.lateral_m);
  const base = CLS_COLOR[t.cls] ?? '#a1a1aa';
  const color = t.conflict ? '#f87171' : base;
  const r = Math.max(3.5, toPx(t.radius_m));

  // Arrow shows a second of travel RELATIVE to the cart, which is the quantity
  // that decides whether this ends in a conflict.
  const tip = at(t.forward_m + t.vf_ms, t.lateral_m + t.vl_ms);
  const showArrow = t.moving && Math.hypot(tip.x - x, tip.y - y) > 4;

  return (
    <g opacity={t.confirmed ? 1 : 0.55}>
      <circle
        cx={x} cy={y} r={r}
        fill={`${color}22`}
        stroke={color}
        strokeWidth={t.conflict ? 1.8 : 1}
        strokeDasharray={t.coasting ? '3 2' : undefined}
      />
      <circle cx={x} cy={y} r={2} fill={color} />
      {showArrow && (
        <line x1={x} y1={y} x2={tip.x} y2={tip.y}
              stroke={color} strokeWidth={1.4} markerEnd="url(#mm-arrow)" />
      )}
      {/* Range is an upper bound when the box is clipped: flag it, because a
          large number there is not evidence of safety. */}
      {t.clipped && (
        <circle cx={x} cy={y} r={r + 3} fill="none" stroke="#fbbf24"
                strokeWidth={1} strokeDasharray="2 3" />
      )}
      {t.conflict && (
        <text x={x + r + 3} y={y + 3} fontSize={8} fill="#fca5a5">
          {t.cls} {t.forward_m.toFixed(1)}m
        </text>
      )}
    </g>
  );
}
