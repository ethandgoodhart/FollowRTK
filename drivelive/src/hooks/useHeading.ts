'use client';

import { useMemo, useRef } from 'react';
import { GpsPosition } from '@/lib/types';
import { haversineMeters, bearing } from '@/lib/geo';

export function useHeading(getHistory: () => GpsPosition[], historyVersion: number) {
  const smoothSpeedRef = useRef(0);

  return useMemo(() => {
    const history = getHistory();
    if (history.length < 2) {
      return { heading: null, speed: 0, speedKmh: 0, speedMph: 0 };
    }

    const curr = history[history.length - 1];
    const prev = history[history.length - 2];
    const dt = curr.ts - prev.ts;
    const rawSpeed = dt > 0 ? haversineMeters(
      { lat: prev.lat, lng: prev.lon },
      { lat: curr.lat, lng: curr.lon }
    ) / dt : 0;

    smoothSpeedRef.current = 0.7 * smoothSpeedRef.current + 0.3 * rawSpeed;
    const speed = smoothSpeedRef.current;

    let head: number | null = null;
    if (speed > 0.5 && history.length >= 3) {
      const older = history[Math.max(0, history.length - 4)];
      head = bearing(
        { lat: older.lat, lng: older.lon },
        { lat: curr.lat, lng: curr.lon }
      );
    }

    return {
      heading: head,
      speed,
      speedKmh: speed * 3.6,
      speedMph: speed * 2.237,
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [historyVersion]);
}
