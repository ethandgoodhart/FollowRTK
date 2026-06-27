'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import { GpsPosition, FollowState } from '@/lib/types';

const MAX_HISTORY = 500;

export function useGps(wsUrl: string) {
  const [position, setPosition] = useState<GpsPosition | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [follow, setFollow] = useState<FollowState | null>(null);
  const historyRef = useRef<GpsPosition[]>([]);
  const [historyVersion, setHistoryVersion] = useState(0);
  const wsRef = useRef<WebSocket | null>(null);

  const getHistory = useCallback(() => historyRef.current, []);

  const sendCommand = useCallback((obj: object) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }, []);

  useEffect(() => {
    let reconnectTimer: ReturnType<typeof setTimeout>;

    function connect() {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => setIsConnected(true);

      ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === 'position') {
          const p = msg.data as GpsPosition;
          setPosition(p);
          historyRef.current.push(p);
          if (historyRef.current.length > MAX_HISTORY) {
            historyRef.current = historyRef.current.slice(-MAX_HISTORY);
          }
          setHistoryVersion((v) => v + 1);
        } else if (msg.type === 'history') {
          historyRef.current = msg.data as GpsPosition[];
          if (historyRef.current.length > 0) {
            setPosition(historyRef.current[historyRef.current.length - 1]);
          }
          setHistoryVersion((v) => v + 1);
        } else if (msg.type === 'follow') {
          setFollow({ ...(msg.data as FollowState), active: true });
        } else if (msg.type === 'follow_end') {
          setFollow({ ...(msg.data as FollowState), active: false });
        }
      };

      ws.onclose = () => {
        setIsConnected(false);
        wsRef.current = null;
        reconnectTimer = setTimeout(connect, 2000);
      };

      ws.onerror = () => ws.close();
    }

    connect();

    return () => {
      clearTimeout(reconnectTimer);
      wsRef.current?.close();
    };
  }, [wsUrl]);

  return { position, isConnected, getHistory, historyVersion, follow, sendCommand };
}
