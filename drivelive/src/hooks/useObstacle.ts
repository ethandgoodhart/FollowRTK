//
// useObstacle.ts
//
// Created on July 10, 2026
//
// Created by Georg von Manstein
//

'use client';

import { useState, useEffect, useRef } from 'react';
import { ObstacleStatus } from '@/lib/types';

const POLL_MS = 300;

/**
 * Polls the camera obstacle detector (obstacle/detector.py) while enabled.
 * Returns its latest status, whether the service is reachable, and a
 * cache-busted URL for the annotated camera frame (bump per poll so an
 * <img> naturally refreshes into a low-rate video feed).
 */
export function useObstacle(enabled: boolean, baseUrl: string) {
  const [status, setStatus] = useState<ObstacleStatus | null>(null);
  const [online, setOnline] = useState(false);
  const tickRef = useRef(0);
  const [frameUrl, setFrameUrl] = useState<string | null>(null);

  useEffect(() => {
    if (!enabled) {
      setStatus(null);
      setOnline(false);
      setFrameUrl(null);
      return;
    }
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch(`${baseUrl}/status`, { cache: 'no-store' });
        if (!res.ok) throw new Error(`status ${res.status}`);
        const data = (await res.json()) as ObstacleStatus;
        if (cancelled) return;
        setStatus(data);
        setOnline(true);
        tickRef.current += 1;
        setFrameUrl(`${baseUrl}/frame.jpg?t=${tickRef.current}`);
      } catch {
        if (cancelled) return;
        setOnline(false);
      }
    };
    poll();
    const timer = setInterval(poll, POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [enabled, baseUrl]);

  return { status, online, frameUrl };
}
