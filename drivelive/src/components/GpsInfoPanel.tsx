'use client';

import { GpsPosition } from '@/lib/types';

const FIX_LABELS: Record<number, { label: string; color: string }> = {
  4: { label: 'RTK FIX', color: 'bg-green-700 text-green-300' },
  5: { label: 'RTK FLOAT', color: 'bg-yellow-800 text-yellow-300' },
  2: { label: 'DGPS', color: 'bg-orange-800 text-orange-300' },
  1: { label: 'GPS', color: 'bg-red-900 text-red-400' },
  0: { label: 'NO FIX', color: 'bg-neutral-700 text-neutral-400' },
};

function compassDir(deg: number): string {
  const dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  return dirs[Math.round(deg / 45) % 8];
}

interface Props {
  position: GpsPosition | null;
  speed: number;
  heading: number | null;
  isConnected: boolean;
}

export default function GpsInfoPanel({ position, speed, heading, isConnected }: Props) {
  const fix = FIX_LABELS[position?.fix_code ?? 0] ?? FIX_LABELS[0];

  return (
    <div className="absolute top-3 left-3 z-10 w-64 rounded-xl bg-neutral-900/90 backdrop-blur-md p-4 text-sm text-neutral-200 shadow-lg border border-neutral-800">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-base font-bold text-white">DriveLive</h2>
        <div className={`w-2.5 h-2.5 rounded-full ${isConnected ? 'bg-green-400 shadow-[0_0_6px_#4f4]' : 'bg-red-500 shadow-[0_0_6px_#f44]'}`} />
      </div>

      <div className="flex items-center justify-between mb-2">
        <span className={`px-2 py-0.5 rounded text-xs font-bold uppercase tracking-wide ${fix.color}`}>
          {fix.label}
        </span>
        <span className="text-3xl font-bold text-white tabular-nums">
          {speed.toFixed(1)} <span className="text-sm text-neutral-400 font-normal">km/h</span>
        </span>
      </div>

      {position && (
        <>
          <div className="font-mono text-xs text-neutral-400 mb-2">
            {position.lat.toFixed(8)}, {position.lon.toFixed(8)}
          </div>
          <div className="grid grid-cols-3 gap-2 text-xs text-neutral-500">
            <div>
              Sats <span className="text-neutral-300 font-semibold">{position.sats}</span>
            </div>
            <div>
              HDOP <span className="text-neutral-300 font-semibold">{position.hdop.toFixed(1)}</span>
            </div>
            <div>
              Alt <span className="text-neutral-300 font-semibold">{position.alt.toFixed(0)}m</span>
            </div>
          </div>
          {heading !== null && (
            <div className="mt-2 text-xs text-neutral-500">
              Heading <span className="text-neutral-300 font-semibold">{heading.toFixed(0)}° {compassDir(heading)}</span>
            </div>
          )}
        </>
      )}

      {!position && (
        <div className="text-neutral-500 text-xs mt-1">Waiting for GPS...</div>
      )}
    </div>
  );
}
