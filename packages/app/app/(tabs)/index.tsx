import CampusMap from "@/components/CampusMap";
import RideTrackingModal from "@/components/RideTrackingModal";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useRide } from "@/contexts/RideContext";
import { ArrowRight, MapPin } from "lucide-react-native";
import React, { useState } from "react";
import {
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const { currentRide, requestRide, updateRideStatus } = useRide();
  const [selectedPickup, setSelectedPickup] = useState<string | null>(null);
  const [selectedDropoff, setSelectedDropoff] = useState<string | null>(null);
  const [selectingType, setSelectingType] = useState<"pickup" | "dropoff">(
    "pickup"
  );
  const [showTracking, setShowTracking] = useState(false);

  const handleRequestRide = () => {
    if (selectedPickup && selectedDropoff) {
      requestRide(selectedPickup, selectedDropoff);
      setShowTracking(true);
    }
  };

  const handleCloseTracking = () => {
    if (currentRide?.status === "completed") {
      setShowTracking(false);
      setSelectedPickup(null);
      setSelectedDropoff(null);
    }
  };

  const getLocationName = (id: string | null) => {
    if (!id) return "Select location";
    return CAMPUS_LOCATIONS.find((l) => l.id === id)?.name || "Unknown";
  };

  const canRequestRide =
    selectedPickup && selectedDropoff && selectedPickup !== selectedDropoff;

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
    <View style={styles.container}>
      <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
        <View>
          <Text style={styles.greeting}>Hello! 👋</Text>
          <Text style={styles.title}>Where to?</Text>
        </View>
      </View>

      <ScrollView
        style={styles.scrollView}
        showsVerticalScrollIndicator={false}
      >
        <View style={styles.selectionContainer}>
          <TouchableOpacity
            style={[
              styles.locationCard,
              selectingType === "pickup" && styles.locationCardActive,
            ]}
            onPress={() => setSelectingType("pickup")}
          >
            <View
              style={[styles.locationIcon, { backgroundColor: Colors.accent }]}
            >
              <MapPin size={20} color={Colors.white} />
            </View>
            <View style={styles.locationInfo}>
              <Text style={styles.locationLabel}>Pickup Location</Text>
              <Text style={styles.locationValue} numberOfLines={1}>
                {getLocationName(selectedPickup)}
              </Text>
            </View>
            {selectingType === "pickup" && (
              <View style={styles.activeIndicator} />
            )}
          </TouchableOpacity>

          <TouchableOpacity
            style={[
              styles.locationCard,
              selectingType === "dropoff" && styles.locationCardActive,
            ]}
            onPress={() => setSelectingType("dropoff")}
          >
            <View
              style={[styles.locationIcon, { backgroundColor: Colors.primary }]}
            >
              <MapPin size={20} color={Colors.white} />
            </View>
            <View style={styles.locationInfo}>
              <Text style={styles.locationLabel}>Drop-off Location</Text>
              <Text style={styles.locationValue} numberOfLines={1}>
                {getLocationName(selectedDropoff)}
              </Text>
            </View>
            {selectingType === "dropoff" && (
              <View style={styles.activeIndicator} />
            )}
          </TouchableOpacity>
        </View>

        <View style={styles.mapSection}>
          <Text style={styles.sectionTitle}>
            Tap on the map to select {selectingType} location
          </Text>
          <CampusMap
            selectedPickup={selectedPickup}
            selectedDropoff={selectedDropoff}
            onSelectPickup={setSelectedPickup}
            onSelectDropoff={setSelectedDropoff}
            selectingType={selectingType}
          />
        </View>

        <View style={styles.quickSelectSection}>
          <Text style={styles.sectionTitle}>Quick Select</Text>
          <View style={styles.quickSelectGrid}>
            {CAMPUS_LOCATIONS.slice(0, 6).map((location) => (
              <TouchableOpacity
                key={location.id}
                style={styles.quickSelectItem}
                onPress={() => {
                  if (selectingType === "pickup") {
                    setSelectedPickup(location.id);
                    setSelectingType("dropoff");
                  } else {
                    setSelectedDropoff(location.id);
                  }
                }}
              >
                <View style={styles.quickSelectIcon}>
                  <MapPin size={16} color={Colors.accent} />
                </View>
                <Text style={styles.quickSelectName} numberOfLines={2}>
                  {location.name}
                </Text>
              </TouchableOpacity>
            ))}
          </View>
        </View>
      </ScrollView>

      {canRequestRide && (
        <View style={styles.footer}>
          <TouchableOpacity
            style={styles.requestButton}
            onPress={handleRequestRide}
          >
            <Text style={styles.requestButtonText}>Request ACT Ride</Text>
            <ArrowRight size={20} color={Colors.white} />
          </TouchableOpacity>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  header: {
    paddingHorizontal: 20,
    paddingBottom: 20,
    backgroundColor: Colors.white,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  greeting: {
    fontSize: 16,
    color: Colors.textSecondary,
    marginBottom: 4,
  },
  title: {
    fontSize: 28,
    fontWeight: "700" as const,
    color: Colors.text,
  },
  scrollView: {
    flex: 1,
  },
  selectionContainer: {
    padding: 16,
    gap: 12,
  },
  locationCard: {
    backgroundColor: Colors.white,
    borderRadius: 16,
    padding: 16,
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    borderWidth: 2,
    borderColor: Colors.border,
  },
  locationCardActive: {
    borderColor: Colors.accent,
    backgroundColor: Colors.surface,
  },
  locationIcon: {
    width: 44,
    height: 44,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
  },
  locationInfo: {
    flex: 1,
  },
  locationLabel: {
    fontSize: 12,
    color: Colors.textSecondary,
    marginBottom: 4,
    fontWeight: "600" as const,
    textTransform: "uppercase",
  },
  locationValue: {
    fontSize: 16,
    color: Colors.text,
    fontWeight: "600" as const,
  },
  activeIndicator: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: Colors.accent,
  },
  mapSection: {
    padding: 16,
    paddingTop: 8,
  },
  sectionTitle: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: Colors.textSecondary,
    marginBottom: 16,
    paddingHorizontal: 4,
  },
  quickSelectSection: {
    padding: 16,
    paddingTop: 8,
    paddingBottom: 24,
  },
  quickSelectGrid: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 12,
  },
  quickSelectItem: {
    width: "31%",
    backgroundColor: Colors.white,
    borderRadius: 16,
    padding: 16,
    alignItems: "center",
    gap: 8,
    borderWidth: 1,
    borderColor: Colors.border,
  },
  quickSelectIcon: {
    width: 40,
    height: 40,
    borderRadius: 12,
    backgroundColor: Colors.surface,
    alignItems: "center",
    justifyContent: "center",
  },
  quickSelectName: {
    fontSize: 11,
    fontWeight: "600" as const,
    color: Colors.text,
    textAlign: "center",
    minHeight: 32,
  },
  footer: {
    padding: 16,
    backgroundColor: Colors.white,
    borderTopWidth: 1,
    borderTopColor: Colors.border,
  },
  requestButton: {
    backgroundColor: Colors.accent,
    borderRadius: 16,
    paddingVertical: 18,
    paddingHorizontal: 24,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 8,
    shadowColor: Colors.accent,
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 8,
    elevation: 6,
  },
  requestButtonText: {
    fontSize: 17,
    fontWeight: "700" as const,
    color: Colors.white,
  },
});
