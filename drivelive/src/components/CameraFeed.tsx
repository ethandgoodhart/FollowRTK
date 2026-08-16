'use client';

import { useEffect, useRef, useState } from 'react';
import { feetFromMeters, metersFromFeet } from '@/lib/geo';
import { PerceptionState } from '@/lib/types';

interface Props {
  perception: PerceptionState | null;
  cameraMount: { height_m: number; pitch_deg: number } | null;
  sendCommand: (obj: object) => boolean;
}

const DEFAULT_HEIGHT_FT = 5.85;
const DEFAULT_PITCH_DEG = 15.0;

export default function CameraFeed({ perception, cameraMount, sendCommand }: Props) {
  const [heightFt, setHeightFt] = useState(DEFAULT_HEIGHT_FT);
  const [pitch, setPitch] = useState(DEFAULT_PITCH_DEG);
  const dragging = useRef(false);

  useEffect(() => {
    if (dragging.current) return;
    const h = perception?.height_m ?? cameraMount?.height_m;
    const p = perception?.pitch_deg ?? cameraMount?.pitch_deg;
    if (h != null) setHeightFt(feetFromMeters(h));
    if (p != null) setPitch(p);
  }, [perception?.height_m, perception?.pitch_deg, cameraMount?.height_m, cameraMount?.pitch_deg]);

  const sendMount = (nextHeightFt: number, nextPitch: number) => {
    sendCommand({ type: 'camera_mount', height_m: metersFromFeet(nextHeightFt), pitch_deg: nextPitch });
  };

  const jpeg = perception?.preview_jpeg;

  return (
    <div className="w-[286px] rounded-xl bg-neutral-900/85 p-3 text-neutral-200 ring-1 ring-neutral-700 backdrop-blur">
      <div className="mb-2 flex items-center justify-between">
        <span className="text-[10px] font-semibold uppercase tracking-wide text-neutral-500">
          front camera
        </span>
        <span className="text-[10px] tabular-nums text-neutral-500">
          {jpeg ? 'live' : 'no feed'}
        </span>
      </div>

      <div className="relative mb-2 overflow-hidden rounded-lg bg-black" style={{ aspectRatio: '4 / 3' }}>
        {jpeg ? (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            src={`data:image/jpeg;base64,${jpeg}`}
            alt="Front camera"
            className="h-full w-full object-cover"
          />
        ) : (
          <div className="flex h-full items-center justify-center text-[11px] text-neutral-500">
            waiting for camera…
          </div>
        )}
      </div>

      <MountSlider
        label="height"
        description="lens above ground"
        suffix="ft"
        value={heightFt}
        min={2.5}
        max={10}
        step={0.05}
        digits={2}
        onDragStart={() => { dragging.current = true; }}
        onDragEnd={() => { dragging.current = false; }}
        onChange={(v) => {
          setHeightFt(v);
          sendMount(v, pitch);
        }}
      />
      <MountSlider
        label="pitch"
        description="positive = looking down"
        suffix="°"
        value={pitch}
        min={0}
        max={30}
        step={0.1}
        digits={1}
        onDragStart={() => { dragging.current = true; }}
        onDragEnd={() => { dragging.current = false; }}
        onChange={(v) => {
          setPitch(v);
          sendMount(heightFt, v);
        }}
      />
    </div>
  );
}

function MountSlider({
  label,
  description,
  suffix,
  value,
  min,
  max,
  step,
  digits,
  onChange,
  onDragStart,
  onDragEnd,
}: {
  label: string;
  description: string;
  suffix: string;
  value: number;
  min: number;
  max: number;
  step: number;
  digits: number;
  onChange: (value: number) => void;
  onDragStart: () => void;
  onDragEnd: () => void;
}) {
  return (
    <label className="block py-1">
      <div className="flex justify-between gap-3 text-[11px]">
        <span className="min-w-0 text-neutral-300">
          {label} <span className="text-neutral-500">({description})</span>
        </span>
        <span className="shrink-0 tabular-nums text-neutral-200">
          {value.toFixed(digits)}{suffix}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onPointerDown={onDragStart}
        onPointerUp={onDragEnd}
        onPointerCancel={onDragEnd}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="w-full accent-purple-500"
      />
    </label>
  );
}
