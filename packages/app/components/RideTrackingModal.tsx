import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { Ride } from "@/types/ride";
import { Clock, Navigation, X } from "lucide-react-native";
import React, { useCallback, useEffect, useRef } from "react";
import { Modal, StyleSheet, Text, TouchableOpacity, View } from "react-native";
import CampusMap from "./CampusMap";

interface RideTrackingModalProps {
  visible: boolean;
  ride: Ride | null;
  onClose: () => void;
  onUpdateStatus: (
    status: Ride["status"],
    vehiclePosition?: { latitude: number; longitude: number }
  ) => void;
}

export default function RideTrackingModal({
  visible,
  ride,
  onClose,
  onUpdateStatus,
}: RideTrackingModalProps) {
  const hasInitializedRef = useRef<{
    status: string;
    hasVehiclePosition: boolean;
  } | null>(null);
  const onUpdateStatusRef = useRef(onUpdateStatus);

  useEffect(() => {
    onUpdateStatusRef.current = onUpdateStatus;
  }, [onUpdateStatus]);

  const simulateVehicleMovement = useCallback(
    (
      from: { latitude: number; longitude: number },
      to: { latitude: number; longitude: number },
      nextStatus: Ride["status"],
      currentStatus: Ride["status"]
    ) => {
      const steps = 30;
      const deltaLat = (to.latitude - from.latitude) / steps;
      const deltaLng = (to.longitude - from.longitude) / steps;
      let currentStep = 0;

      const interval = setInterval(() => {
        currentStep++;
        const newLat = from.latitude + deltaLat * currentStep;
        const newLng = from.longitude + deltaLng * currentStep;

        onUpdateStatusRef.current(currentStatus, {
          latitude: newLat,
          longitude: newLng,
        });

        if (currentStep >= steps) {
          clearInterval(interval);
          setTimeout(() => {
            onUpdateStatusRef.current(nextStatus, {
              latitude: to.latitude,
              longitude: to.longitude,
            });
          }, 500);
        }
      }, 100);
    },
    []
  );

  useEffect(() => {
    if (!ride) {
      hasInitializedRef.current = null;
      return;
    }

    const pickupLocation = CAMPUS_LOCATIONS.find(
      (l) => l.id === ride.pickupLocationId
    );
    const dropoffLocation = CAMPUS_LOCATIONS.find(
      (l) => l.id === ride.dropoffLocationId
    );

    if (!pickupLocation || !dropoffLocation) return;

    const currentKey = `${ride.status}-${!!ride.vehiclePosition}`;
    const prevKey = hasInitializedRef.current
      ? `${hasInitializedRef.current.status}-${hasInitializedRef.current.hasVehiclePosition}`
      : null;

    if (currentKey === prevKey) return;

    hasInitializedRef.current = {
      status: ride.status,
      hasVehiclePosition: !!ride.vehiclePosition,
    };

    if (ride.status === "assigned" && !ride.vehiclePosition) {
      const startLat = pickupLocation.latitude - 0.003;
      const startLng = pickupLocation.longitude - 0.003;
      onUpdateStatusRef.current("assigned", {
        latitude: startLat,
        longitude: startLng,
      });

      const timer = setTimeout(() => {
        simulateVehicleMovement(
          { latitude: startLat, longitude: startLng },
          {
            latitude: pickupLocation.latitude,
            longitude: pickupLocation.longitude,
          },
          "arriving",
          "assigned"
        );
      }, 2000);

      return () => clearTimeout(timer);
    } else if (ride.status === "arriving" && ride.vehiclePosition) {
      const arrivedLat =
        Math.abs(ride.vehiclePosition.latitude - pickupLocation.latitude) <
        0.0001;
      const arrivedLng =
        Math.abs(ride.vehiclePosition.longitude - pickupLocation.longitude) <
        0.0001;

      if (arrivedLat && arrivedLng) {
        const timer1 = setTimeout(() => {
          onUpdateStatusRef.current("in-progress", {
            latitude: pickupLocation.latitude,
            longitude: pickupLocation.longitude,
          });
          const timer2 = setTimeout(() => {
            simulateVehicleMovement(
              {
                latitude: pickupLocation.latitude,
                longitude: pickupLocation.longitude,
              },
              {
                latitude: dropoffLocation.latitude,
                longitude: dropoffLocation.longitude,
              },
              "completed",
              "in-progress"
            );
          }, 1500);
          return () => clearTimeout(timer2);
        }, 3000);

        return () => clearTimeout(timer1);
      }
    }
  }, [ride, simulateVehicleMovement]);

  if (!ride) return null;

  const pickupLocation = CAMPUS_LOCATIONS.find(
    (l) => l.id === ride.pickupLocationId
  );
  const dropoffLocation = CAMPUS_LOCATIONS.find(
    (l) => l.id === ride.dropoffLocationId
  );

  const getStatusInfo = () => {
    switch (ride.status) {
      case "pending":
        return {
          title: "Requesting Ride...",
          subtitle: "Finding an available ACT vehicle",
          color: Colors.warning,
        };
      case "assigned":
        return {
          title: `${ride.vehicleId} is on the way`,
          subtitle: `Arriving in ${ride.estimatedWaitTime} min`,
          color: Colors.accent,
        };
      case "arriving":
        return {
          title: "Vehicle Arriving",
          subtitle: "Your ACT is almost there!",
          color: Colors.accent,
        };
      case "in-progress":
        return {
          title: "Trip in Progress",
          subtitle: "Enjoy your ride!",
          color: Colors.success,
        };
      default:
        return {
          title: "Ride Status",
          subtitle: "",
          color: Colors.textSecondary,
        };
    }
  };

  const statusInfo = getStatusInfo();

  return (
    <Modal
      visible={visible}
      animationType="slide"
      transparent={false}
      onRequestClose={onClose}
    >
      <View style={styles.container}>
        <View style={styles.header}>
          <View style={styles.headerLeft}>
            <View
              style={[styles.statusDot, { backgroundColor: statusInfo.color }]}
            />
            <View>
              <Text style={styles.headerTitle}>{statusInfo.title}</Text>
              <Text style={styles.headerSubtitle}>{statusInfo.subtitle}</Text>
            </View>
          </View>
          <TouchableOpacity onPress={onClose} style={styles.closeButton}>
            <X size={24} color={Colors.text} />
          </TouchableOpacity>
        </View>

        <View style={styles.mapContainer}>
          <CampusMap
            selectedPickup={ride.pickupLocationId}
            selectedDropoff={ride.dropoffLocationId}
            onSelectPickup={() => {}}
            onSelectDropoff={() => {}}
            selectingType="pickup"
            vehiclePosition={ride.vehiclePosition}
          />
        </View>

        <View style={styles.details}>
          <View style={styles.routeInfo}>
            <View style={styles.routeItem}>
              <View
                style={[styles.routeDot, { backgroundColor: Colors.accent }]}
              />
              <View style={styles.routeTextContainer}>
                <Text style={styles.routeLabel}>Pickup</Text>
                <Text style={styles.routeLocation}>{pickupLocation?.name}</Text>
              </View>
            </View>

            <View style={styles.routeLine} />

            <View style={styles.routeItem}>
              <View
                style={[styles.routeDot, { backgroundColor: Colors.primary }]}
              />
              <View style={styles.routeTextContainer}>
                <Text style={styles.routeLabel}>Drop-off</Text>
                <Text style={styles.routeLocation}>
                  {dropoffLocation?.name}
                </Text>
              </View>
            </View>
          </View>

          {ride.vehicleId && (
            <View style={styles.vehicleInfo}>
              <View style={styles.infoRow}>
                <Navigation size={18} color={Colors.accent} />
                <Text style={styles.infoText}>Vehicle: {ride.vehicleId}</Text>
              </View>
              {ride.estimatedWaitTime && ride.status !== "in-progress" && (
                <View style={styles.infoRow}>
                  <Clock size={18} color={Colors.accent} />
                  <Text style={styles.infoText}>
                    ETA: {ride.estimatedWaitTime} min
                  </Text>
                </View>
              )}
            </View>
          )}
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    padding: 20,
    paddingTop: 60,
    backgroundColor: Colors.white,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  headerLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    flex: 1,
  },
  statusDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
  },
  headerTitle: {
    fontSize: 18,
    fontWeight: "700" as const,
    color: Colors.text,
  },
  headerSubtitle: {
    fontSize: 14,
    color: Colors.textSecondary,
    marginTop: 2,
  },
  closeButton: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: Colors.surface,
    alignItems: "center",
    justifyContent: "center",
  },
  mapContainer: {
    padding: 16,
    marginTop: 20,
  },
  details: {
    padding: 20,
  },
  routeInfo: {
    backgroundColor: Colors.white,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: Colors.border,
    marginBottom: 16,
  },
  routeItem: {
    flexDirection: "row",
    alignItems: "flex-start",
    gap: 12,
  },
  routeDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
    marginTop: 4,
  },
  routeLine: {
    width: 2,
    height: 24,
    backgroundColor: Colors.border,
    marginLeft: 5,
    marginVertical: 8,
  },
  routeTextContainer: {
    flex: 1,
  },
  routeLabel: {
    fontSize: 12,
    color: Colors.textSecondary,
    marginBottom: 2,
    textTransform: "uppercase",
    fontWeight: "600" as const,
  },
  routeLocation: {
    fontSize: 16,
    color: Colors.text,
    fontWeight: "600" as const,
  },
  vehicleInfo: {
    backgroundColor: Colors.surface,
    borderRadius: 16,
    padding: 16,
    gap: 12,
  },
  infoRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 10,
  },
  infoText: {
    fontSize: 15,
    color: Colors.text,
    fontWeight: "500" as const,
  },
});
