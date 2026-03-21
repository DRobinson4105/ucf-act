import CampusMap from "@/components/CampusMap";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { api } from "@/convex/_generated/api";
import { useRide } from "@/contexts/RideContext";
import { useDebouncedNavigation } from "@/hooks/useDebouncedNavigation";
import { Car, MapPin, Search, Star, X } from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
  Animated,
  Modal,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { useQuery } from "convex/react";

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const {
    currentRide, cancelRide, boardRide, rideHistory,
    pendingReview, submitReview, dismissReview,
  } = useRide();
  const router = useDebouncedNavigation();
  const [reviewRating, setReviewRating] = useState(0);
  const [reviewComment, setReviewComment] = useState("");
  const pulseAnim = useRef(new Animated.Value(0.4)).current;
  const allCarts = useQuery(api.carts.getAll, {});

  useEffect(() => {
    const isActive = !!currentRide && !["completed", "cancelled"].includes(currentRide.status);
    if (isActive) {
      const anim = Animated.loop(
        Animated.sequence([
          Animated.timing(pulseAnim, { toValue: 1, duration: 900, useNativeDriver: true }),
          Animated.timing(pulseAnim, { toValue: 0.4, duration: 900, useNativeDriver: true }),
        ])
      );
      anim.start();
      return () => anim.stop();
    } else {
      pulseAnim.setValue(0.4);
    }
  }, [currentRide?.status]);

  const recentLocations = rideHistory
    .map((ride) => CAMPUS_LOCATIONS.find((l) => l.id === ride.dropoffLocationId))
    .filter((loc): loc is (typeof CAMPUS_LOCATIONS)[0] => loc !== undefined)
    .filter((loc, i, arr) => arr.findIndex((l) => l.id === loc.id) === i)
    .slice(0, 3);

  const isTracking = !!currentRide &&
    ["requested", "assigned", "arriving", "in_progress"].includes(currentRide.status);
  const isArriving = currentRide?.status === "arriving";
  const canCancel = !!currentRide &&
    ["requested", "assigned", "arriving", "in_progress"].includes(currentRide.status);

  const getStatusDotColor = () => {
    switch (currentRide?.status) {
      case "requested": return Colors.textSecondary;
      case "assigned":  return Colors.accent;
      case "arriving":  return Colors.success;
      case "in_progress": return Colors.primary;
      default: return Colors.accent;
    }
  };

  const getTrackingText = () => {
    switch (currentRide?.status) {
      case "requested":   return { title: "Finding your cart…", sub: "Looking for an available ACT cart nearby" };
      case "assigned":    return { title: "Cart on the way", sub: "Your ACT cart is heading to your pickup" };
      case "arriving":    return { title: "Cart has arrived!", sub: "Head to your pickup and tap when you're there" };
      case "in_progress": return { title: "On your way", sub: "Sit back and enjoy your autonomous ride" };
      default:            return { title: "", sub: "" };
    }
  };

  const pickupName  = CAMPUS_LOCATIONS.find(l => l.id === currentRide?.pickupLocationId)?.shortName  ?? "Pickup";
  const dropoffName = CAMPUS_LOCATIONS.find(l => l.id === currentRide?.dropoffLocationId)?.shortName ?? "Destination";

  return (
    <View className="flex-1 bg-background">
      {/* Live map — always visible as background */}
      <View style={StyleSheet.absoluteFill} pointerEvents="none">
        <CampusMap
          selectedPickup={null}
          selectedDropoff={null}
          onSelectPickup={() => {}}
          onSelectDropoff={() => {}}
          selectingType="pickup"
          allCarts={allCarts ?? []}
          cartId={currentRide?.cartId}
          hideUserLocation={currentRide?.status === "in_progress"}
          showRoute={false}
          interactive={false}
          fullScreen
        />
      </View>

      {/* Search bar overlay */}
      <View className="absolute left-0 right-0 px-5" style={{ top: insets.top + 16 }}>
        <TouchableOpacity
          className="flex-row items-center bg-card rounded-2xl px-4 py-4 gap-3 shadow-md border border-border"
          onPress={() => router.push("/plan-ride")}
          activeOpacity={isTracking ? 1 : 0.7}
          disabled={router.isNavigating || isTracking}
        >
          <Search size={22} color={isTracking ? Colors.textSecondary : Colors.accent} />
          <Text
            className="flex-1 text-lg font-semibold"
            style={{ color: isTracking ? Colors.textSecondary : Colors.text }}
          >
            {isTracking ? "Ride in progress" : "Where to?"}
          </Text>
        </TouchableOpacity>
      </View>

      {/* Active ride tracking card */}
      {isTracking && currentRide && (
        <View
          className="absolute left-0 right-0 bottom-0 bg-surface rounded-t-3xl px-5 pt-3 shadow-2xl border-t border-border"
          style={{ paddingBottom: insets.bottom + 20 }}
        >
          <View style={styles.handle} />

          {/* Status indicator */}
          <View className="flex-row items-center gap-2.5 mb-1">
            <Animated.View
              style={[styles.statusDot, { backgroundColor: getStatusDotColor(), opacity: pulseAnim }]}
            />
            <Text className="text-lg font-bold text-text">{getTrackingText().title}</Text>
          </View>
          <Text className="text-sm text-textSecondary mb-4" style={{ paddingLeft: 20.5 }}>
            {getTrackingText().sub}
          </Text>

          {/* Route chip */}
          <View className="flex-row items-center bg-card rounded-xl px-4 py-3 mb-4 gap-2 border border-border">
            <View style={styles.originDot} />
            <Text className="text-sm text-textSecondary" numberOfLines={1} style={{ flex: 1 }}>
              {pickupName}
            </Text>
            <Text className="text-textSecondary" style={{ marginHorizontal: 2 }}>→</Text>
            <View style={styles.destDot} />
            <Text className="text-sm font-semibold text-text" numberOfLines={1} style={{ flex: 1 }}>
              {dropoffName}
            </Text>
          </View>

          {/* Cart info row */}
          <View className="flex-row items-center gap-3 mb-4">
            <View className="w-12 h-12 bg-card rounded-xl items-center justify-center border border-border">
              <Car size={22} color={Colors.accent} />
            </View>
            <View className="flex-1">
              <Text className="text-base font-semibold text-text">ACT-001</Text>
              <Text className="text-sm text-textSecondary">Autonomous Golf Cart · Free</Text>
            </View>
          </View>

          {/* Primary action */}
          {isArriving && (
            <TouchableOpacity
              className="rounded-2xl py-4 items-center mb-3"
              style={{ backgroundColor: Colors.accent }}
              onPress={boardRide}
              activeOpacity={0.8}
            >
              <Text className="font-bold text-base" style={{ color: Colors.black }}>I'm Here</Text>
            </TouchableOpacity>
          )}

          {/* Cancel */}
          {canCancel && (
            <TouchableOpacity
              className="flex-row items-center justify-center gap-2 rounded-xl py-3.5 border"
              style={{
                borderColor: "rgba(239,68,68,0.35)",
                backgroundColor: "rgba(239,68,68,0.06)",
                marginTop: isArriving ? 0 : 0,
              }}
              onPress={cancelRide}
              activeOpacity={0.8}
            >
              <X size={16} color={Colors.error} />
              <Text className="font-semibold text-sm" style={{ color: Colors.error }}>
                Cancel Ride
              </Text>
            </TouchableOpacity>
          )}
        </View>
      )}

      {/* Post-ride review modal */}
      <Modal
        visible={!!pendingReview}
        transparent
        animationType="slide"
        onRequestClose={dismissReview}
      >
        <View style={{ flex: 1, justifyContent: "flex-end", backgroundColor: "rgba(0,0,0,0.55)" }}>
          <View
            className="bg-surface rounded-t-3xl px-6 pt-4"
            style={{ paddingBottom: insets.bottom + 16 }}
          >
            <View style={styles.handle} />

            <Text className="text-2xl font-bold text-text mb-1">How was your ride?</Text>
            {pendingReview && (
              <Text className="text-sm text-textSecondary mb-6">
                {CAMPUS_LOCATIONS.find(l => l.id === pendingReview.pickupLocationId)?.name ?? "Pickup"}
                {" → "}
                {CAMPUS_LOCATIONS.find(l => l.id === pendingReview.dropoffLocationId)?.name ?? "Dropoff"}
              </Text>
            )}

            {/* Stars */}
            <View className="flex-row justify-center mb-2" style={{ gap: 8 }}>
              {[1, 2, 3, 4, 5].map((n) => (
                <TouchableOpacity
                  key={n}
                  onPress={() => setReviewRating(n)}
                  activeOpacity={0.7}
                  hitSlop={{ top: 8, bottom: 8, left: 4, right: 4 }}
                >
                  <Star
                    size={40}
                    color={n <= reviewRating ? Colors.accent : Colors.border}
                    fill={n <= reviewRating ? Colors.accent : "transparent"}
                  />
                </TouchableOpacity>
              ))}
            </View>

            {reviewRating === 0 ? (
              <Text className="text-xs text-textSecondary text-center mb-4">
                Tap a star to rate your ride
              </Text>
            ) : (
              <View className="mb-4" />
            )}

            <TextInput
              placeholder="Add a comment (optional)"
              placeholderTextColor={Colors.textSecondary}
              value={reviewComment}
              onChangeText={setReviewComment}
              multiline
              numberOfLines={3}
              style={{
                backgroundColor: Colors.card,
                borderRadius: 12,
                padding: 14,
                color: Colors.text,
                borderWidth: 1,
                borderColor: Colors.border,
                marginBottom: 16,
                minHeight: 76,
                textAlignVertical: "top",
                fontSize: 15,
              }}
            />

            <TouchableOpacity
              style={[
                styles.submitBtn,
                reviewRating === 0 && styles.submitBtnDisabled,
              ]}
              onPress={() => {
                if (reviewRating === 0) return;
                submitReview(reviewRating, reviewComment.trim() || undefined);
                setReviewRating(0);
                setReviewComment("");
              }}
              activeOpacity={reviewRating === 0 ? 1 : 0.8}
            >
              <Text style={[
                styles.submitBtnText,
                reviewRating === 0 && styles.submitBtnTextDisabled,
              ]}>
                Submit Review
              </Text>
            </TouchableOpacity>

            <TouchableOpacity
              className="items-center py-3"
              onPress={() => {
                dismissReview();
                setReviewRating(0);
                setReviewComment("");
              }}
              activeOpacity={0.7}
            >
              <Text className="text-sm text-textSecondary">Skip for now</Text>
            </TouchableOpacity>
          </View>
        </View>
      </Modal>

      {/* Recent locations — only when not tracking */}
      {!isTracking && recentLocations.length > 0 && (
        <View
          className="absolute left-0 right-0 bottom-0 bg-surface rounded-t-3xl px-5 pt-3 shadow-xl border-t border-border"
          style={{ paddingBottom: insets.bottom + 8 }}
        >
          <View style={styles.handle} />
          <Text className="text-base font-bold text-text mb-3 px-1">Recent Locations</Text>
          <ScrollView showsVerticalScrollIndicator={false}>
            {recentLocations.map((location) => (
              <TouchableOpacity
                key={location.id}
                className="flex-row items-center bg-card rounded-2xl p-4 mb-3 gap-4 border border-border"
                onPress={() =>
                  router.push({ pathname: "/plan-ride", params: { destinationId: location.id } })
                }
                disabled={router.isNavigating}
                activeOpacity={0.7}
              >
                <View className="w-10 h-10 rounded-xl bg-surface items-center justify-center border border-white/5">
                  <MapPin size={18} color={Colors.accent} />
                </View>
                <View className="flex-1">
                  <Text className="text-base font-semibold text-text">{location.name}</Text>
                  <Text className="text-sm text-textSecondary">{location.shortName}</Text>
                </View>
              </TouchableOpacity>
            ))}
          </ScrollView>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  handle: {
    width: 40,
    height: 4,
    borderRadius: 2,
    backgroundColor: Colors.border,
    alignSelf: "center",
    marginBottom: 16,
  },
  statusDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  originDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: Colors.textSecondary,
  },
  destDot: {
    width: 8,
    height: 8,
    borderRadius: 2,
    backgroundColor: Colors.accent,
  },
  submitBtn: {
    borderRadius: 16,
    paddingVertical: 16,
    alignItems: "center",
    marginBottom: 4,
    backgroundColor: Colors.accent,
  },
  submitBtnDisabled: {
    backgroundColor: Colors.border,
    opacity: 0.6,
  },
  submitBtnText: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.black,
  },
  submitBtnTextDisabled: {
    color: Colors.textSecondary,
  },
});
