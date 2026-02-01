import RideTrackingModal from "@/components/RideTrackingModal";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useRide } from "@/contexts/RideContext";
import { useDebouncedNavigation } from "@/hooks/useDebouncedNavigation";
import { Calendar, Clock, Search } from "lucide-react-native";
import React, { useEffect, useState } from "react";
import {
  ScrollView,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const { currentRide, updateRideStatus, rideHistory } = useRide();
  const [showTracking, setShowTracking] = useState(false);
  const router = useDebouncedNavigation();

  useEffect(() => {
    if (currentRide) {
      setShowTracking(true);
    }
  }, [currentRide]);

  const handleCloseTracking = () => {
    if (currentRide?.status === "completed") {
      setShowTracking(false);
    }
  };

  const recentLocations = rideHistory
    .slice(0, 3)
    .map((ride) =>
      CAMPUS_LOCATIONS.find((l) => l.id === ride.dropoffLocationId)
    )
    .filter((loc): loc is (typeof CAMPUS_LOCATIONS)[0] => loc !== undefined);

  if (currentRide && showTracking) {
    return (
      <RideTrackingModal
        visible={true}
        ride={currentRide}
        onClose={handleCloseTracking}
        onUpdateStatus={updateRideStatus}
      />
    );
  }

  return (
    <View className="flex-1 bg-background">
      <View className="px-5 pb-5 bg-background" style={{ paddingTop: insets.top + 20 }}>
        <TouchableOpacity
          className="flex-row items-center bg-surface rounded-2xl px-4 py-4 gap-3 shadow-md border border-border"
          onPress={() => router.push("/plan-ride")}
          activeOpacity={0.7}
          disabled={router.isNavigating}
        >
          <Search size={20} color={Colors.textSecondary} />
          <Text className="flex-1 text-lg font-semibold text-text">Where to?</Text>
          <View className="flex-row items-center bg-card px-3 py-2 rounded-xl gap-1.5">
            <Calendar size={16} color={Colors.text} />
            <Text className="text-sm font-semibold text-text">Later</Text>
          </View>
        </TouchableOpacity>
      </View>

      <ScrollView
        className="flex-1"
        showsVerticalScrollIndicator={false}
      >
        {recentLocations.length > 0 && (
          <View className="px-5 pt-2">
            {recentLocations.map((location) => (
              <TouchableOpacity
                key={location.id}
                className="flex-row items-center bg-surface rounded-2xl p-4 mb-3 gap-4 border border-border"
                onPress={() => {
                  router.push({
                    pathname: "/plan-ride",
                    params: { destinationId: location.id },
                  });
                }}
                disabled={router.isNavigating}
              >
                <View className="w-12 h-12 rounded-xl bg-card items-center justify-center">
                  <Clock size={20} color={Colors.textSecondary} />
                </View>
                <View className="flex-1">
                  <Text className="text-lg font-semibold text-text mb-1">{location.name}</Text>
                  <Text className="text-sm text-textSecondary">
                    {location.shortName}
                  </Text>
                </View>
              </TouchableOpacity>
            ))}
          </View>
        )}
      </ScrollView>
    </View>
  );
}
