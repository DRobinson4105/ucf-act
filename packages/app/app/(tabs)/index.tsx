import CampusMap from "@/components/CampusMap";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { api } from "@/convex/_generated/api";
import { useRide } from "@/contexts/RideContext";
import { useDebouncedNavigation } from "@/hooks/useDebouncedNavigation";
import { Calendar, MapPin, Search, X } from "lucide-react-native";
import React from "react";
import {
    ScrollView,
    Text,
    TouchableOpacity,
    View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { useQuery } from "convex/react";

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const { currentRide, cancelRide, boardRide, rideHistory } = useRide();
  const router = useDebouncedNavigation();
  const allCarts = useQuery(api.carts.getAll, {});

  const recentLocations = rideHistory
    .map((ride) => CAMPUS_LOCATIONS.find((l) => l.id === ride.dropoffLocationId))
    .filter((loc): loc is (typeof CAMPUS_LOCATIONS)[0] => loc !== undefined)
    .filter((loc, i, arr) => arr.findIndex((l) => l.id === loc.id) === i)
    .slice(0, 3);

  const isTracking = !!currentRide && ["assigned", "arriving", "in_progress"].includes(currentRide.status);
  const isArriving = currentRide?.status === "arriving";
  const canCancel = currentRide?.status === "assigned" || currentRide?.status === "arriving" || currentRide?.status === "in_progress";

  const getTrackingText = () => {
    switch (currentRide?.status) {
      case "assigned": return { title: "Cart on the way", sub: "Your ACT cart is heading to your pickup" };
      case "arriving": return { title: "Cart has arrived!", sub: "Walk to your pickup and tap Ready" };
      case "in_progress": return { title: "Trip in progress", sub: "Sit back and enjoy your autonomous ride" };
      default: return { title: "", sub: "" };
    }
  };

  return (
    <View className="flex-1 bg-background">
      {/* Live map — always visible as background */}
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

      {/* Search bar overlay */}
      <View
        className="absolute left-0 right-0 px-5"
        style={{ top: insets.top + 16 }}
      >
        <TouchableOpacity
          className="flex-row items-center bg-card rounded-2xl px-4 py-4 gap-3 shadow-md border border-border"
          onPress={() => router.push("/plan-ride")}
          activeOpacity={0.7}
          disabled={router.isNavigating || isTracking}
        >
          <Search size={22} color={Colors.accent} />
          <Text className="flex-1 text-lg font-semibold text-text">Where to?</Text>
          <View className="flex-row items-center bg-surface px-3 py-2 rounded-xl gap-1.5 border border-white/5">
            <Calendar size={16} color={Colors.text} />
            <Text className="text-sm font-semibold text-text">Later</Text>
          </View>
        </TouchableOpacity>
      </View>

      {/* Active ride tracking card */}
      {isTracking && currentRide && (
        <View
          className="absolute left-0 right-0 bottom-0 bg-surface rounded-t-3xl px-5 pt-3 pb-6 shadow-2xl border-t border-border"
          style={{ paddingBottom: insets.bottom + 16 }}
        >
          <View className="w-10 h-1 bg-border rounded-full self-center mb-4" />

          <Text className="text-lg font-bold text-text mb-1">{getTrackingText().title}</Text>
          <Text className="text-sm text-textSecondary mb-4">{getTrackingText().sub}</Text>

          <View className="flex-row items-center gap-3 mb-4">
            <View className="w-10 h-10 bg-card rounded-xl items-center justify-center border border-border">
              <Text style={{ fontSize: 20 }}>🛺</Text>
            </View>
            <View className="flex-1">
              <Text className="text-base font-semibold text-text">ACT-001</Text>
              <Text className="text-sm text-textSecondary">ACT Golf Cart • White</Text>
            </View>
          </View>

          {isArriving && (
            <TouchableOpacity
              className="bg-primary rounded-xl py-4 items-center mb-3"
              onPress={boardRide}
              activeOpacity={0.8}
            >
              <Text className="text-white font-bold text-base">I'm Ready — Board Cart</Text>
            </TouchableOpacity>
          )}

          {canCancel && (
            <TouchableOpacity
              className="flex-row items-center justify-center gap-2 bg-surface rounded-xl py-4 border border-red-500/40"
              onPress={cancelRide}
              activeOpacity={0.8}
            >
              <X size={18} color={Colors.error} />
              <Text className="font-semibold text-base" style={{ color: Colors.error }}>Cancel Ride</Text>
            </TouchableOpacity>
          )}
        </View>
      )}

      {/* Recent locations — only when not tracking */}
      {!isTracking && recentLocations.length > 0 && (
        <View
          className="absolute left-0 right-0 bottom-0 bg-surface rounded-t-3xl px-5 pt-3 shadow-xl border-t border-border"
          style={{ paddingBottom: insets.bottom + 8 }}
        >
          <View className="w-10 h-1 bg-border rounded-full self-center mb-4" />
          <Text className="text-base font-bold text-text mb-3 px-1">Recent Locations</Text>
          <ScrollView showsVerticalScrollIndicator={false}>
            {recentLocations.map((location) => (
              <TouchableOpacity
                key={location.id}
                className="flex-row items-center bg-card rounded-2xl p-4 mb-3 gap-4 border border-border"
                onPress={() => router.push({ pathname: "/plan-ride", params: { destinationId: location.id } })}
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
