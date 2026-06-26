'use client';

import { useState } from 'react';
import mapboxgl from 'mapbox-gl';
import { RawAnnotations } from '@/lib/types';
import { useAnnotations } from '@/hooks/useAnnotations';
import { useGps } from '@/hooks/useGps';
import { useHeading } from '@/hooks/useHeading';
import { useRoute } from '@/hooks/useRoute';
import MapView from './MapView';
import AnnotationLayers from './AnnotationLayers';
import GpsMarker from './GpsMarker';
import RouteLayer from './RouteLayer';
import RouteSelector from './RouteSelector';
import GpsInfoPanel from './GpsInfoPanel';
import RoutePanel from './RoutePanel';

interface Props {
  rawAnnotations: RawAnnotations;
}

export default function DriveApp({ rawAnnotations }: Props) {
  const [map, setMap] = useState<mapboxgl.Map | null>(null);
  const token = process.env.NEXT_PUBLIC_MAPBOX_TOKEN!;
  const wsUrl = process.env.NEXT_PUBLIC_GPS_WS_URL || 'ws://localhost:8765';

  const { laneBoundaries, connectorBoundaries, allCenterLines, graph } = useAnnotations(rawAnnotations);
  const { position, isConnected, getHistory, historyVersion } = useGps(wsUrl);
  const { heading, speedKmh, speed } = useHeading(getHistory, historyVersion);
  const route = useRoute(graph, position, speed);

  return (
    <div className="w-screen h-screen relative">
      <MapView token={token} onMapReady={setMap}>
        {map && (
          <>
            <AnnotationLayers map={map} lanes={laneBoundaries} connectors={connectorBoundaries} centerLines={allCenterLines} />
            <GpsMarker map={map} position={position} heading={heading} />
            <RouteLayer map={map} path={route.path} startPoint={route.startPoint} endPoint={route.endPoint} />
            <RouteSelector map={map} selecting={route.selecting} onMapClick={route.handleMapClick} />
          </>
        )}
      </MapView>

      <GpsInfoPanel position={position} speed={speedKmh} heading={heading} isConnected={isConnected} />
      <RoutePanel route={route} onSelectStart={route.selectStart} onSelectEnd={route.selectEnd} onClear={route.clearRoute} />
    </div>
  );
}
