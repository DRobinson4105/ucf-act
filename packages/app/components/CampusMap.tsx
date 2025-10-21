import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import React, { useMemo, useState } from "react";
import { Dimensions, StyleSheet, View } from "react-native";
import MapView, { Marker, Polyline, Region } from "react-native-maps";

const { width } = Dimensions.get("window");

interface CampusMapProps {
  selectedPickup: string | null;
  selectedDropoff: string | null;
  onSelectPickup: (locationId: string) => void;
  onSelectDropoff: (locationId: string) => void;
  selectingType: "pickup" | "dropoff";
  vehiclePosition?: { latitude: number; longitude: number };
}

const UCF_COORDINATES: Region = {
  latitude: 28.6019444,
  longitude: -81.2004444,
  latitudeDelta: 0.005,
  longitudeDelta: 0.007,
};

export default function CampusMap({
  selectedPickup,
  selectedDropoff,
  onSelectPickup,
  onSelectDropoff,
  selectingType,
  vehiclePosition,
}: CampusMapProps) {
  const [routeCoords, setRouteCoords] = useState<
    { latitude: number; longitude: number }[]
  >([]);
  const handleMarkerPress = (locationId: string) => {
    if (selectingType === "pickup") {
      onSelectPickup(locationId);
    } else {
      onSelectDropoff(locationId);
    }
  };

  useMemo(() => {
    const pickup = CAMPUS_LOCATIONS.find((l) => l.id === selectedPickup);
    const dropoff = CAMPUS_LOCATIONS.find((l) => l.id === selectedDropoff);

    if (pickup && dropoff) {
      setRouteCoords([
        { latitude: pickup.latitude, longitude: pickup.longitude },
        { latitude: dropoff.latitude, longitude: dropoff.longitude },
      ]);
    } else {
      setRouteCoords([]);
    }
  }, [selectedPickup, selectedDropoff]);

  return (
    <View style={styles.container}>
      <MapView style={styles.map} initialRegion={UCF_COORDINATES}>
        {vehiclePosition && (
          <Marker
            coordinate={vehiclePosition}
            title="Vehicle"
            pinColor={Colors.secondary}
          />
        )}

        {/* Pickup & Dropoff markers */}
        {CAMPUS_LOCATIONS.map((loc) => {
          const isPickup = selectedPickup === loc.id;
          const isDropoff = selectedDropoff === loc.id;

          return (
            <Marker
              key={loc.id}
              coordinate={{
                latitude: loc.latitude,
                longitude: loc.longitude,
              }}
              onPress={() => handleMarkerPress(loc.id)}
              pinColor={
                isPickup ? Colors.accent : isDropoff ? Colors.primary : "gray"
              }
              title={loc.name}
              description={
                isPickup
                  ? "Pickup location"
                  : isDropoff
                    ? "Drop-off location"
                    : "Tap to select"
              }
            />
          );
        })}

        {/* Route line */}
        {routeCoords.length > 0 && (
          <Polyline
            coordinates={routeCoords}
            strokeColor={Colors.primary}
            strokeWidth={4}
          />
        )}
      </MapView>
    </View>
  );
}
const styles = StyleSheet.create({
  container: {
    width: width - 32,
    alignSelf: "center",
  },
  map: {
    width: "100%",
    height: 350,
    borderRadius: 20,
    overflow: "hidden",
  },
  legend: {
    flexDirection: "row",
    justifyContent: "center",
    gap: 24,
    marginTop: 16,
    paddingVertical: 12,
    backgroundColor: Colors.white,
    borderRadius: 12,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  legendItem: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  legendDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
  },
  legendText: {
    fontSize: 13,
    color: Colors.textSecondary,
    fontWeight: "500" as const,
  },
});
