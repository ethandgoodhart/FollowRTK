'use client';

import { useEffect, useMemo, useState } from 'react';
import mapboxgl from 'mapbox-gl';
import { RawAnnotations } from '@/lib/types';
import { courseOverGround } from '@/lib/geo';
import { useAnnotations } from '@/hooks/useAnnotations';
import { useGps } from '@/hooks/useGps';
import { useSpeed } from '@/hooks/useSpeed';
import { useRoute } from '@/hooks/useRoute';
import MapView from './MapView';
import AnnotationLayers from './AnnotationLayers';
import GpsMarker from './GpsMarker';
import RouteLayer from './RouteLayer';
import RecoveryLayer from './RecoveryLayer';
import TrajectoryLayer from './TrajectoryLayer';
import RouteSelector from './RouteSelector';
import GpsInfoPanel from './GpsInfoPanel';
import RoutePanel from './RoutePanel';
import DriveControl from './DriveControl';

interface Props {
  rawAnnotations: RawAnnotations;
}

export default function DriveApp({ rawAnnotations }: Props) {
  const [map, setMap] = useState<mapboxgl.Map | null>(null);
  const token = process.env.NEXT_PUBLIC_MAPBOX_TOKEN!;
  const wsUrl = process.env.NEXT_PUBLIC_GPS_WS_URL || 'ws://localhost:8765';

  // Yellow overlay = lane center lines ONLY (the lane dividers), matching the
  // `live` editor. The snapped connector center-lines live only inside `graph`
  // for routing; drawing them yellow would double-draw every connector (raw
  // blue + distorted yellow) — that was the visual regression vs live.
  const { laneBoundaries, connectorBoundaries, laneCenterLines, graph } = useAnnotations(rawAnnotations);
  const { position, isConnected, getHistory, historyVersion, follow, sendCommand } = useGps(wsUrl);
  const { speedMph, speed } = useSpeed(getHistory, historyVersion);
  const route = useRoute(graph, position, speed, laneCenterLines);

  // Panic stop: 'q' or Esc slams the brake to full immediately, anytime.
  // Ignored while typing in a field so it can't fire by accident.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== 'Escape' && e.key !== 'q' && e.key !== 'Q') return;
      const el = e.target as HTMLElement | null;
      const tag = el?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el?.isContentEditable) return;
      e.preventDefault();
      sendCommand({ type: 'stop', emergency: true });
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [sendCommand]);

  // Heading for the turquoise prediction = real course over ground from the GPS
  // track (NOT follow.heading_deg, which is the needle's path+wheel-lean hack).
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const heading = useMemo(() => courseOverGround(getHistory()), [historyVersion]);

  return (
    <div className="w-screen h-screen relative">
      <MapView token={token} onMapReady={setMap}>
        {map && (
          <>
            <AnnotationLayers map={map} lanes={laneBoundaries} connectors={connectorBoundaries} centerLines={laneCenterLines} />
            <GpsMarker map={map} position={position} heading={follow?.active ? follow.heading_deg : null} />
            <RouteLayer map={map} path={route.path} startPoint={route.startPoint} endPoint={route.endPoint} />
            <RecoveryLayer map={map} position={position} path={route.path} heading={heading} />
            <TrajectoryLayer
              map={map}
              position={position}
              speed={speed}
              heading={heading}
              steerDeg={follow?.active ? follow.steering_actual_deg ?? 0 : 0}
            />
            <RouteSelector map={map} selecting={route.selecting} onMapClick={route.handleMapClick} />
          </>
        )}
      </MapView>

      <GpsInfoPanel position={position} speed={speedMph} isConnected={isConnected} />
      <RoutePanel route={route} onSelectEnd={route.selectEnd} onClear={route.clearRoute} />
      <DriveControl route={route} follow={follow} speedMph={speedMph} isConnected={isConnected} sendCommand={sendCommand} />
    </div>
  );
}
