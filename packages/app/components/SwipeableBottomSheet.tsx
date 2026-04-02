import React, { useEffect, useRef } from "react";
import {
  Animated,
  Dimensions,
  PanResponder,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

const { height: SCREEN_HEIGHT } = Dimensions.get("window");
export const SHEET_MIN_HEIGHT = 230;
const MAX_HEIGHT = SCREEN_HEIGHT * 0.85;
const DRAG_THRESHOLD = 3;

interface SwipeableBottomSheetProps {
  children: React.ReactNode;
  onSwipeUp?: () => void;
  onSwipeDown?: () => void;
  initiallyExpanded?: boolean;
  expandRef?: React.MutableRefObject<{ expand: () => void } | null>;
  maxHeight?: number;
}

export default function SwipeableBottomSheet({
  children,
  onSwipeUp,
  onSwipeDown,
  initiallyExpanded = true,
  expandRef,
}: SwipeableBottomSheetProps) {
  const insets = useSafeAreaInsets();
  const INITIAL_OFFSET = MAX_HEIGHT - SHEET_MIN_HEIGHT;
  const translateY = useRef(
    new Animated.Value(initiallyExpanded ? 0 : INITIAL_OFFSET)
  ).current;
  const lastGesture = useRef(initiallyExpanded ? 0 : INITIAL_OFFSET);
  const isExpanded = useRef(initiallyExpanded);

  const expand = React.useCallback(() => {
    Animated.spring(translateY, {
      toValue: 0,
      useNativeDriver: true,
      tension: 65,
      friction: 10,
    }).start();
    lastGesture.current = 0;
    isExpanded.current = true;
    onSwipeUp?.();
  }, [translateY, onSwipeUp]);

  useEffect(() => {
    if (expandRef) {
      expandRef.current = { expand };
    }
  }, [expandRef, expand]);

  const panResponder = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => false,
      onStartShouldSetPanResponderCapture: () => false,
      onMoveShouldSetPanResponder: (_, gestureState) => {
        const isDraggingVertically =
          Math.abs(gestureState.dy) > Math.abs(gestureState.dx) * 2;
        return (
          isDraggingVertically && Math.abs(gestureState.dy) > DRAG_THRESHOLD
        );
      },
      onMoveShouldSetPanResponderCapture: () => false,
      onPanResponderGrant: () => {
        lastGesture.current = isExpanded.current ? 0 : INITIAL_OFFSET;
      },
      onPanResponderMove: (_, gestureState) => {
        const newValue = lastGesture.current + gestureState.dy;
        if (newValue >= 0 && newValue <= INITIAL_OFFSET) {
          translateY.setValue(newValue);
        }
      },
      onPanResponderRelease: (_, gestureState) => {
        const currentValue = lastGesture.current + gestureState.dy;
        const threshold = INITIAL_OFFSET / 2;
        const velocity = gestureState.vy;

        if (
          velocity < -0.5 ||
          (Math.abs(velocity) < 0.5 && currentValue < threshold)
        ) {
          Animated.spring(translateY, {
            toValue: 0,
            useNativeDriver: true,
            tension: 65,
            friction: 10,
          }).start();
          lastGesture.current = 0;
          isExpanded.current = true;
          onSwipeUp?.();
        } else {
          Animated.spring(translateY, {
            toValue: INITIAL_OFFSET,
            useNativeDriver: true,
            tension: 65,
            friction: 10,
          }).start();
          lastGesture.current = INITIAL_OFFSET;
          isExpanded.current = false;
          onSwipeDown?.();
        }
      },
    })
  ).current;

  return (
    <Animated.View
      style={[
        {
          position: "absolute",
          bottom: 0,
          left: 0,
          right: 0,
          backgroundColor: "#1A1A1A",
          borderTopLeftRadius: 24,
          borderTopRightRadius: 24,
          zIndex: 50,
          shadowColor: "#000",
          shadowOffset: { width: 0, height: -4 },
          shadowOpacity: 0.3,
          shadowRadius: 12,
          elevation: 16,
          height: MAX_HEIGHT,
          transform: [{ translateY }],
          paddingBottom: insets.bottom,
        },
      ]}
      {...panResponder.panHandlers}
    >
      <View style={{ alignItems: "center", justifyContent: "center", paddingTop: 16, paddingBottom: 8 }}>
        <View style={{ width: 56, height: 4, backgroundColor: "#2A2A2A", borderRadius: 2 }} />
      </View>
      {children}
    </Animated.View>
  );
}
