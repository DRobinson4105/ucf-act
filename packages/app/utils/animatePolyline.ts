import { useEffect, useRef, useState } from "react";
import { Animated, Easing } from "react-native";

export interface LatLng {
  latitude: number;
  longitude: number;
}

function slicePath(coords: LatLng[], fraction: number): LatLng[] {
  if (coords.length < 2 || fraction <= 0) return [];
  if (fraction >= 1) return coords;

  const segDists: number[] = [];
  let totalDist = 0;
  for (let i = 1; i < coords.length; i++) {
    const dLat = coords[i].latitude - coords[i - 1].latitude;
    const dLon = coords[i].longitude - coords[i - 1].longitude;
    const d = Math.sqrt(dLat * dLat + dLon * dLon);
    segDists.push(d);
    totalDist += d;
  }

  if (totalDist === 0) return [coords[0]];

  const targetDist = totalDist * fraction;
  let accumulated = 0;

  for (let i = 0; i < segDists.length; i++) {
    if (accumulated + segDists[i] >= targetDist) {
      const segFraction =
        segDists[i] === 0 ? 0 : (targetDist - accumulated) / segDists[i];
      const interpLat =
        coords[i].latitude +
        (coords[i + 1].latitude - coords[i].latitude) * segFraction;
      const interpLon =
        coords[i].longitude +
        (coords[i + 1].longitude - coords[i].longitude) * segFraction;
      return [
        ...coords.slice(0, i + 1),
        { latitude: interpLat, longitude: interpLon },
      ];
    }
    accumulated += segDists[i];
  }

  return coords;
}

export function useAnimatedPolyline(
  fullPath: LatLng[],
  durationMs: number = 800
): LatLng[] {
  const [visiblePath, setVisiblePath] = useState<LatLng[]>([]);
  const progressRef = useRef(new Animated.Value(0));
  const listenerIdRef = useRef<string | null>(null);
  const pathRef = useRef<LatLng[]>([]);
  const animRef = useRef<Animated.CompositeAnimation | null>(null);

  useEffect(() => {
    const prev = pathRef.current;
    const changed =
      fullPath.length !== prev.length ||
      fullPath.some(
        (p, i) =>
          p.latitude !== prev[i]?.latitude || p.longitude !== prev[i]?.longitude
      );

    if (!changed) return;
    pathRef.current = fullPath;

    if (animRef.current) {
      animRef.current.stop();
      animRef.current = null;
    }
    if (listenerIdRef.current) {
      progressRef.current.removeListener(listenerIdRef.current);
      listenerIdRef.current = null;
    }

    if (fullPath.length < 2) {
      setVisiblePath(fullPath);
      return;
    }

    progressRef.current.setValue(0);

    const currentPath = fullPath;

    listenerIdRef.current = progressRef.current.addListener(({ value }) => {
      setVisiblePath(slicePath(currentPath, value));
    });

    const anim = Animated.timing(progressRef.current, {
      toValue: 1,
      duration: durationMs,
      easing: Easing.out(Easing.cubic),
      useNativeDriver: false,
    });

    animRef.current = anim;
    anim.start(({ finished }) => {
      if (finished) {
        setVisiblePath(currentPath);
      }
    });

    return () => {
      if (animRef.current) {
        animRef.current.stop();
        animRef.current = null;
      }
      if (listenerIdRef.current) {
        progressRef.current.removeListener(listenerIdRef.current);
        listenerIdRef.current = null;
      }
    };
  }, [fullPath, durationMs]);

  return visiblePath;
}
