'use client';

import { useEffect, useRef } from 'react';
import mapboxgl from 'mapbox-gl';
import { GpsPosition } from '@/lib/types';

const FIX_COLORS: Record<number, string> = {
  4: '#44ff44',
  5: '#ffcc00',
  2: '#ff8800',
  1: '#ff4444',
  0: '#888888',
};

interface Props {
  map: mapboxgl.Map | null;
  position: GpsPosition | null;
  heading: number | null;
}

export default function GpsMarker({ map, position, heading }: Props) {
  const markerRef = useRef<mapboxgl.Marker | null>(null);
  const elRef = useRef<HTMLDivElement | null>(null);
  const arrowRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (!map) return;

    const el = document.createElement('div');
    el.style.width = '22px';
    el.style.height = '22px';
    el.style.position = 'relative';

    const dot = document.createElement('div');
    dot.className = 'gps-pulse-dot';
    dot.style.width = '22px';
    dot.style.height = '22px';
    dot.style.borderRadius = '50%';
    dot.style.border = '3px solid white';
    dot.style.background = '#888';
    dot.style.boxShadow = '0 0 6px rgba(0,0,0,0.5)';
    el.appendChild(dot);

    const arrow = document.createElement('div');
    arrow.style.position = 'absolute';
    arrow.style.top = '-14px';
    arrow.style.left = '50%';
    arrow.style.transform = 'translateX(-50%)';
    arrow.style.width = '0';
    arrow.style.height = '0';
    arrow.style.borderLeft = '6px solid transparent';
    arrow.style.borderRight = '6px solid transparent';
    arrow.style.borderBottom = '12px solid white';
    arrow.style.opacity = '0';
    arrow.style.transition = 'opacity 0.3s';
    el.appendChild(arrow);

    elRef.current = dot;
    arrowRef.current = arrow;

    const marker = new mapboxgl.Marker({ element: el })
      .setLngLat([-122.1663, 37.4269])
      .addTo(map);
    markerRef.current = marker;

    return () => {
      marker.remove();
      markerRef.current = null;
    };
  }, [map]);

  useEffect(() => {
    if (!markerRef.current || !position) return;
    markerRef.current.setLngLat([position.lon, position.lat]);
    if (elRef.current) {
      elRef.current.style.background = FIX_COLORS[position.fix_code] || FIX_COLORS[0];
    }
  }, [position]);

  useEffect(() => {
    if (!arrowRef.current || !markerRef.current) return;
    const el = markerRef.current.getElement();
    if (heading !== null) {
      arrowRef.current.style.opacity = '1';
      el.style.transformOrigin = 'center center';
      el.style.transform = `rotate(${heading}deg)`;
    } else {
      arrowRef.current.style.opacity = '0';
      el.style.transform = '';
    }
  }, [heading]);

  return null;
}
