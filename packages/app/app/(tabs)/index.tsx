import { CAMPUS_LOCATIONS, CampusLocation } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { useRide } from "@/contexts/RideContext";
import { useRouter } from "expo-router";
import { MaterialCommunityIcons } from "@expo/vector-icons";
import {
  BookOpen,
  Building2,
  MapPin,
  Navigation,
  ParkingSquare,
  Search,
  Star,
  TreePine,
  Users,
  UtensilsCrossed,
  X,
} from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
  Animated,
  Keyboard,
  KeyboardAvoidingView,
  Modal,
  Platform,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

function greeting() {
  const h = new Date().getHours();
  if (h < 12) return "Good morning";
  if (h < 17) return "Good afternoon";
  return "Good evening";
}

function locationIcon(type: CampusLocation["type"], color: string) {
  const size = 18;
  switch (type) {
    case "academic":     return <BookOpen size={size} color={color} />;
    case "residential":  return <Users size={size} color={color} />;
    case "dining":       return <UtensilsCrossed size={size} color={color} />;
    case "recreation":   return <TreePine size={size} color={color} />;
    case "parking":      return <ParkingSquare size={size} color={color} />;
    default:             return <Building2 size={size} color={color} />;
  }
}

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const { user } = useAuth();
  const {
    currentRide, cancelRide, boardRide, rideHistory,
    pendingReview, submitReview, dismissReview,
  } = useRide();
  const router = useRouter();
  const [reviewRating, setReviewRating] = useState(0);
  const [reviewComment, setReviewComment] = useState("");
  const pulseAnim = useRef(new Animated.Value(0.4)).current;

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
    .filter((loc): loc is CampusLocation => loc !== undefined)
    .filter((loc, i, arr) => arr.findIndex((l) => l.id === loc.id) === i)
    .slice(0, 3);

  const isTracking = !!currentRide &&
    ["requested", "assigned", "arriving", "in_progress"].includes(currentRide.status);
  const isArriving = currentRide?.status === "arriving";
  const canCancel = !!currentRide &&
    ["requested", "assigned", "arriving", "in_progress"].includes(currentRide.status);

  const getStatusDotColor = () => {
    switch (currentRide?.status) {
      case "requested":   return Colors.textSecondary;
      case "assigned":    return Colors.accent;
      case "arriving":    return Colors.success;
      case "in_progress": return Colors.primary;
      default:            return Colors.accent;
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
  const firstName   = user?.name?.split(" ")[0];

  return (
    <View style={[styles.root, { paddingTop: insets.top }]}>
      <ScrollView
        contentContainerStyle={styles.scroll}
        showsVerticalScrollIndicator={false}
        keyboardShouldPersistTaps="handled"
      >
        {/* ── Header ──────────────────────────────────── */}
        <View style={styles.header}>
          <View>
            <Text style={styles.greetingLabel}>{greeting()}{firstName ? `, ${firstName}` : ""}</Text>
            <Text style={styles.greetingSub}>UCF Autonomous Campus Transit</Text>
          </View>
          <View style={styles.logoMark}>
            <MaterialCommunityIcons name="golf-cart" size={22} color={Colors.accent} />
          </View>
        </View>

        {/* ── Where to CTA ────────────────────────────── */}
        <TouchableOpacity
          style={[styles.searchBar, isTracking && styles.searchBarDisabled]}
          onPress={() => router.push("/plan-ride")}
          activeOpacity={isTracking ? 1 : 0.75}
          disabled={isTracking}
        >
          <Search size={20} color={isTracking ? Colors.textSecondary : Colors.accent} />
          <Text style={[styles.searchText, isTracking && styles.searchTextDisabled]}>
            {isTracking ? "Ride in progress" : "Where to?"}
          </Text>
        </TouchableOpacity>

        {/* ── Active ride card ─────────────────────────── */}
        {isTracking && currentRide && (
          <View style={styles.rideCard}>
            {/* Status */}
            <View style={styles.rideStatusRow}>
              <Animated.View
                style={[styles.statusDot, { backgroundColor: getStatusDotColor(), opacity: pulseAnim }]}
              />
              <Text style={styles.rideStatusTitle}>{getTrackingText().title}</Text>
            </View>
            <Text style={styles.rideStatusSub}>{getTrackingText().sub}</Text>

            {/* Route chip */}
            <View style={styles.routeChip}>
              <View style={styles.originDot} />
              <Text style={styles.routeFrom} numberOfLines={1}>{pickupName}</Text>
              <Text style={styles.routeArrow}>→</Text>
              <View style={styles.destDot} />
              <Text style={styles.routeTo} numberOfLines={1}>{dropoffName}</Text>
            </View>

            {/* Cart row */}
            <View style={styles.cartRow}>
              <View style={styles.cartIcon}>
                <MaterialCommunityIcons name="golf-cart" size={20} color={Colors.accent} />
              </View>
              <View style={{ flex: 1 }}>
                <Text style={styles.cartName}>ACT-001</Text>
                <Text style={styles.cartSub}>Autonomous Golf Cart · Free</Text>
              </View>
            </View>

            {isArriving && (
              <TouchableOpacity style={styles.boardBtn} onPress={boardRide} activeOpacity={0.8}>
                <Text style={styles.boardBtnText}>I'm Here</Text>
              </TouchableOpacity>
            )}

            <TouchableOpacity
              style={styles.trackBtn}
              onPress={() => router.push("/plan-ride")}
              activeOpacity={0.8}
            >
              <Navigation size={15} color={Colors.accent} />
              <Text style={styles.trackBtnText}>Track Ride</Text>
            </TouchableOpacity>

            {canCancel && (
              <TouchableOpacity style={styles.cancelBtn} onPress={cancelRide} activeOpacity={0.8}>
                <X size={15} color={Colors.error} />
                <Text style={styles.cancelBtnText}>Cancel Ride</Text>
              </TouchableOpacity>
            )}
          </View>
        )}

        {/* ── Campus stops ─────────────────────────────── */}
        {!isTracking && (
          <>
            <Text style={styles.sectionLabel}>Campus Stops</Text>
            <ScrollView
              horizontal
              showsHorizontalScrollIndicator={false}
              contentContainerStyle={styles.chipsRow}
            >
              {CAMPUS_LOCATIONS.map((loc) => (
                <TouchableOpacity
                  key={loc.id}
                  style={styles.chip}
                  onPress={() =>
                    router.push({ pathname: "/plan-ride", params: { destinationId: loc.id } })
                  }
                  activeOpacity={0.7}
                >
                  <View style={styles.chipIcon}>
                    {locationIcon(loc.type, Colors.accent)}
                  </View>
                  <Text style={styles.chipText} numberOfLines={1}>{loc.shortName}</Text>
                </TouchableOpacity>
              ))}
            </ScrollView>

            {/* ── Recent ───────────────────────────────────── */}
            {recentLocations.length > 0 && (
              <>
                <Text style={styles.sectionLabel}>Recent</Text>
                {recentLocations.map((loc) => (
                  <TouchableOpacity
                    key={loc.id}
                    style={styles.recentRow}
                    onPress={() =>
                      router.push({ pathname: "/plan-ride", params: { destinationId: loc.id } })
                    }
                    disabled={false}
                    activeOpacity={0.7}
                  >
                    <View style={styles.recentIconWrap}>
                      <MapPin size={17} color={Colors.accent} />
                    </View>
                    <View style={{ flex: 1 }}>
                      <Text style={styles.recentName}>{loc.name}</Text>
                      <Text style={styles.recentSub}>{loc.shortName}</Text>
                    </View>
                  </TouchableOpacity>
                ))}
              </>
            )}
          </>
        )}
      </ScrollView>

      {/* ── Post-ride review modal ───────────────────── */}
      <Modal
        visible={!!pendingReview}
        transparent
        animationType="slide"
        onRequestClose={dismissReview}
      >
        <KeyboardAvoidingView
          behavior={Platform.OS === "ios" ? "padding" : "height"}
          style={{ flex: 1 }}
        >
          <View style={styles.modalBackdrop}>
            {/* Backdrop dismiss — only covers area ABOVE the sheet */}
            <Pressable style={{ flex: 1 }} onPress={Keyboard.dismiss} />

            <View style={[styles.modalSheet, { paddingBottom: insets.bottom + 16 }]}>
              <View style={styles.handle} />

              <Text style={styles.reviewTitle}>How was your ride?</Text>
              {pendingReview && (
                <Text style={styles.reviewRoute}>
                  {CAMPUS_LOCATIONS.find(l => l.id === pendingReview.pickupLocationId)?.name ?? "Pickup"}
                  {" → "}
                  {CAMPUS_LOCATIONS.find(l => l.id === pendingReview.dropoffLocationId)?.name ?? "Dropoff"}
                </Text>
              )}

              <View style={styles.starsRow}>
                {[1, 2, 3, 4, 5].map((n) => (
                  <Pressable
                    key={n}
                    onPress={() => setReviewRating(n)}
                    hitSlop={16}
                  >
                    <Star
                      size={40}
                      color={n <= reviewRating ? Colors.accent : Colors.border}
                      fill={n <= reviewRating ? Colors.accent : "transparent"}
                    />
                  </Pressable>
                ))}
              </View>

              {reviewRating === 0 ? (
                <Text style={styles.starHint}>Tap a star to rate your ride</Text>
              ) : (
                <View style={{ height: 16 }} />
              )}

              <TextInput
                placeholder="Add a comment (optional)"
                placeholderTextColor={Colors.textSecondary}
                value={reviewComment}
                onChangeText={setReviewComment}
                multiline
                numberOfLines={3}
                style={styles.commentInput}
              />

              <TouchableOpacity
                style={[styles.submitBtn, reviewRating === 0 && styles.submitBtnDisabled]}
                onPress={() => {
                  if (reviewRating === 0) return;
                  submitReview(reviewRating, reviewComment.trim() || undefined);
                  setReviewRating(0);
                  setReviewComment("");
                }}
                activeOpacity={reviewRating === 0 ? 1 : 0.8}
              >
                <Text style={[styles.submitBtnText, reviewRating === 0 && styles.submitBtnTextDisabled]}>
                  Submit Review
                </Text>
              </TouchableOpacity>

              <TouchableOpacity
                style={styles.skipBtn}
                onPress={() => {
                  dismissReview();
                  setReviewRating(0);
                  setReviewComment("");
                }}
                activeOpacity={0.7}
              >
                <Text style={styles.skipText}>Skip for now</Text>
              </TouchableOpacity>
            </View>
          </View>
        </KeyboardAvoidingView>
      </Modal>
    </View>
  );
}

const styles = StyleSheet.create({
  root: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  scroll: {
    paddingHorizontal: 20,
    paddingBottom: 32,
  },

  // Header
  header: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    marginTop: 20,
    marginBottom: 20,
  },
  greetingLabel: {
    fontSize: 22,
    fontWeight: "700",
    color: Colors.text,
    marginBottom: 2,
  },
  greetingSub: {
    fontSize: 13,
    color: Colors.textSecondary,
  },
  logoMark: {
    width: 44,
    height: 44,
    borderRadius: 22,
    backgroundColor: Colors.card,
    borderWidth: 1,
    borderColor: Colors.border,
    alignItems: "center",
    justifyContent: "center",
  },

  // Search bar
  searchBar: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    backgroundColor: Colors.card,
    borderRadius: 16,
    paddingHorizontal: 16,
    paddingVertical: 16,
    borderWidth: 1,
    borderColor: Colors.border,
    marginBottom: 24,
  },
  searchBarDisabled: {
    opacity: 0.6,
  },
  searchText: {
    fontSize: 17,
    fontWeight: "600",
    color: Colors.text,
    flex: 1,
  },
  searchTextDisabled: {
    color: Colors.textSecondary,
  },

  // Active ride card
  rideCard: {
    backgroundColor: Colors.surface,
    borderRadius: 20,
    padding: 18,
    borderWidth: 1,
    borderColor: Colors.border,
    marginBottom: 24,
    gap: 0,
  },
  rideStatusRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
    marginBottom: 4,
  },
  statusDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  rideStatusTitle: {
    fontSize: 17,
    fontWeight: "700",
    color: Colors.text,
  },
  rideStatusSub: {
    fontSize: 13,
    color: Colors.textSecondary,
    marginBottom: 14,
    paddingLeft: 16,
  },
  routeChip: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: Colors.card,
    borderRadius: 12,
    paddingHorizontal: 14,
    paddingVertical: 10,
    marginBottom: 14,
    gap: 6,
    borderWidth: 1,
    borderColor: Colors.border,
  },
  originDot: {
    width: 7,
    height: 7,
    borderRadius: 4,
    backgroundColor: Colors.textSecondary,
  },
  destDot: {
    width: 7,
    height: 7,
    borderRadius: 2,
    backgroundColor: Colors.accent,
  },
  routeFrom: {
    flex: 1,
    fontSize: 13,
    color: Colors.textSecondary,
  },
  routeArrow: {
    fontSize: 13,
    color: Colors.textSecondary,
  },
  routeTo: {
    flex: 1,
    fontSize: 13,
    fontWeight: "600",
    color: Colors.text,
  },
  cartRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    marginBottom: 16,
  },
  cartIcon: {
    width: 44,
    height: 44,
    borderRadius: 12,
    backgroundColor: Colors.card,
    borderWidth: 1,
    borderColor: Colors.border,
    alignItems: "center",
    justifyContent: "center",
  },
  cartName: {
    fontSize: 15,
    fontWeight: "600",
    color: Colors.text,
    marginBottom: 2,
  },
  cartSub: {
    fontSize: 13,
    color: Colors.textSecondary,
  },
  boardBtn: {
    backgroundColor: Colors.accent,
    borderRadius: 14,
    paddingVertical: 16,
    alignItems: "center",
    marginBottom: 10,
  },
  boardBtnText: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.black,
  },
  trackBtn: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 6,
    borderRadius: 12,
    paddingVertical: 14,
    borderWidth: 1,
    borderColor: Colors.accent,
    backgroundColor: "transparent",
    marginBottom: 8,
  },
  trackBtnText: {
    fontSize: 14,
    fontWeight: "600",
    color: Colors.accent,
  },
  cancelBtn: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 6,
    borderRadius: 12,
    paddingVertical: 14,
    borderWidth: 1,
    borderColor: "rgba(239,68,68,0.35)",
    backgroundColor: "transparent",
  },
  cancelBtnText: {
    fontSize: 14,
    fontWeight: "600",
    color: Colors.error,
  },

  // Section label
  sectionLabel: {
    fontSize: 13,
    fontWeight: "700",
    color: Colors.textSecondary,
    textTransform: "uppercase",
    letterSpacing: 0.6,
    marginBottom: 12,
  },

  // Campus stop chips (horizontal scroll)
  chipsRow: {
    gap: 10,
    paddingBottom: 4,
    marginBottom: 28,
  },
  chip: {
    alignItems: "center",
    gap: 8,
    backgroundColor: Colors.card,
    borderRadius: 14,
    paddingHorizontal: 14,
    paddingVertical: 12,
    borderWidth: 1,
    borderColor: Colors.border,
    minWidth: 80,
  },
  chipIcon: {
    width: 36,
    height: 36,
    borderRadius: 18,
    backgroundColor: Colors.surface,
    alignItems: "center",
    justifyContent: "center",
  },
  chipText: {
    fontSize: 12,
    fontWeight: "600",
    color: Colors.text,
    textAlign: "center",
  },

  // Recent rows
  recentRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 14,
    paddingVertical: 14,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  recentIconWrap: {
    width: 38,
    height: 38,
    borderRadius: 19,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
    borderWidth: 1,
    borderColor: Colors.border,
  },
  recentName: {
    fontSize: 15,
    fontWeight: "500",
    color: Colors.text,
    marginBottom: 2,
  },
  recentSub: {
    fontSize: 12,
    color: Colors.textSecondary,
  },

  // Review modal
  modalBackdrop: {
    flex: 1,
    justifyContent: "flex-end",
    backgroundColor: "rgba(17,17,17,0.55)",
  },
  modalSheet: {
    backgroundColor: Colors.surface,
    borderTopLeftRadius: 24,
    borderTopRightRadius: 24,
    paddingHorizontal: 24,
    paddingTop: 16,
  },
  handle: {
    width: 40,
    height: 4,
    borderRadius: 2,
    backgroundColor: Colors.border,
    alignSelf: "center",
    marginBottom: 20,
  },
  reviewTitle: {
    fontSize: 22,
    fontWeight: "700",
    color: Colors.text,
    marginBottom: 4,
  },
  reviewRoute: {
    fontSize: 13,
    color: Colors.textSecondary,
    marginBottom: 20,
  },
  starsRow: {
    flexDirection: "row",
    justifyContent: "center",
    gap: 8,
    marginBottom: 8,
  },
  starHint: {
    fontSize: 12,
    color: Colors.textSecondary,
    textAlign: "center",
    marginBottom: 16,
  },
  commentInput: {
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
  },
  submitBtn: {
    borderRadius: 14,
    paddingVertical: 16,
    alignItems: "center",
    marginBottom: 4,
    backgroundColor: Colors.accent,
  },
  submitBtnDisabled: {
    backgroundColor: Colors.border,
    opacity: 0.5,
  },
  submitBtnText: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.black,
  },
  submitBtnTextDisabled: {
    color: Colors.textSecondary,
  },
  skipBtn: {
    alignItems: "center",
    paddingVertical: 12,
  },
  skipText: {
    fontSize: 14,
    color: Colors.textSecondary,
  },
});
