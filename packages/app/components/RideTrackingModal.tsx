import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { Ride } from "@/types/ride";
import { MapPin } from "lucide-react-native";
import React, { useCallback, useEffect, useRef } from "react";
import {
  Modal,
  Text,
  View
} from "react-native";
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
          title: `${ride.vehicle.id} is on the way`,
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
      <View className="flex-1 bg-background">
        <View className="absolute top-[60px] left-5 right-5 bg-green-50 rounded-xl p-3 z-10">
          <Text className="text-sm text-green-800 text-center font-semibold">{statusInfo.subtitle}</Text>
        </View>

        <CampusMap
          selectedPickup={ride.pickupLocationId}
          selectedDropoff={ride.dropoffLocationId}
          onSelectPickup={() => {}}
          onSelectDropoff={() => {}}
          selectingType="pickup"
          vehiclePosition={ride.vehiclePosition}
          showRoute={true}
          interactive={false}
          fullScreen={true}
        />

        <View className="absolute bottom-0 left-0 right-0 bg-white rounded-t-3xl px-5 pt-3 pb-5 max-h-[60%] shadow-lg">
          <View className="w-10 h-1 bg-border rounded-full self-center mb-4" />

          <View className="bg-[#1A1A1A] rounded-2xl p-4 mb-4">
            <Text className="text-sm text-white mb-1">The driver will arrive in</Text>
            <Text className="text-xl font-bold text-white">
              {String(ride.estimatedWaitTime || 0).padStart(2, "0")}:
              {Math.floor(Math.random() * 60)
                .toString()
                .padStart(2, "0")}{" "}
              Mins
            </Text>
          </View>

          <View className="bg-surface rounded-2xl p-4 mb-4">
            <View className="flex-row justify-between items-center mb-4">
              <View>
                <Text className="text-lg font-bold text-text mb-1">
                  {ride.vehicle?.licensePlate || "ACT-001"}
                </Text>
                <Text className="text-sm text-textSecondary">
                  {ride.vehicle?.model || "ACT Golf Cart"} •{" "}
                  {ride.vehicle?.color || "White"}
                </Text>
              </View>
              <View className="w-[60px] h-[60px] bg-white rounded-xl items-center justify-center">
                <Text className="text-4xl">🚗</Text>
              </View>
            </View>
          </View>

          <View className="gap-4">
            <View className="flex-row items-start gap-3">
              <View className="w-5 h-5 rounded-full bg-text mt-0.5" />
              <View className="flex-1">
                <Text className="text-xs text-textSecondary mb-0.5 font-semibold">Start Location</Text>
                <Text className="text-[15px] text-text font-semibold">
                  {pickupLocation?.name || "Your Current Location"}
                </Text>
              </View>
            </View>
            <View className="flex-row items-start gap-3">
              <MapPin size={20} color={Colors.text} />
              <View className="flex-1">
                <Text className="text-xs text-textSecondary mb-0.5 font-semibold">Your Destination</Text>
                <Text className="text-[15px] text-text font-semibold">{dropoffLocation?.name}</Text>
              </View>
            </View>
          </View>
        </View>
      </View>
    </Modal>
  );
}
