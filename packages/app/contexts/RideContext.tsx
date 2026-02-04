import { MOCK_PAST_RIDES } from "@/mocks/rides";
import { Notification, Ride, RideStatus } from "@/types/ride";
import createContextHook from "@nkzw/create-context-hook";
import AsyncStorage from "@react-native-async-storage/async-storage";
import { useCallback, useEffect, useMemo, useState } from "react";

const STORAGE_KEY = "act_rides";
const NOTIFICATIONS_KEY = "act_notifications";

export const [RideProvider, useRide] = createContextHook(() => {
  const [currentRide, setCurrentRide] = useState<Ride | null>(null);
  const [rideHistory, setRideHistory] = useState<Ride[]>([]);
  const [notifications, setNotifications] = useState<Notification[]>([]);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    loadData();
  }, []);

  const loadData = async () => {
    try {
      const storedRides = await AsyncStorage.getItem(STORAGE_KEY);
      const storedNotifications = await AsyncStorage.getItem(NOTIFICATIONS_KEY);

      if (storedRides) {
        const parsed = JSON.parse(storedRides);
        setRideHistory(
          parsed.map((r: Ride) => ({
            ...r,
            requestedAt: new Date(r.requestedAt),
            pickupAt: r.pickupAt ? new Date(r.pickupAt) : undefined,
            completedAt: r.completedAt ? new Date(r.completedAt) : undefined,
          }))
        );
      } else {
        setRideHistory(MOCK_PAST_RIDES);
      }

      if (storedNotifications) {
        const parsed = JSON.parse(storedNotifications);
        setNotifications(
          parsed.map((n: Notification) => ({
            ...n,
            timestamp: new Date(n.timestamp),
          }))
        );
      }
    } catch (error) {
      console.error("Error loading data:", error);
      setRideHistory(MOCK_PAST_RIDES);
    } finally {
      setIsLoading(false);
    }
  };

  const saveRides = async (rides: Ride[]) => {
    try {
      await AsyncStorage.setItem(STORAGE_KEY, JSON.stringify(rides));
    } catch (error) {
      console.error("Error saving rides:", error);
    }
  };

  const saveNotifications = async (notifs: Notification[]) => {
    try {
      await AsyncStorage.setItem(NOTIFICATIONS_KEY, JSON.stringify(notifs));
    } catch (error) {
      console.error("Error saving notifications:", error);
    }
  };

  const addNotification = useCallback(
    (params: {
      title: string;
      message: string;
      type: Notification["type"];
    }) => {
      const notification: Notification = {
        id: Date.now().toString(),
        title: params.title,
        message: params.message,
        timestamp: new Date(),
        read: false,
        type: params.type,
      };
      setNotifications((prev) => {
        const updated = [notification, ...prev];
        saveNotifications(updated);
        return updated;
      });
    },
    []
  );

  const requestRide = useCallback(
    (pickupLocationId: string, dropoffLocationId: string) => {
      const ride: Ride = {
        id: Date.now().toString(),
        pickupLocationId,
        dropoffLocationId,
        status: "pending",
        requestedAt: new Date(),
        estimatedWaitTime: Math.floor(Math.random() * 5) + 3,
        vehicle: {
          id: "",
          model: "",
          color: "",
          licensePlate: ""
        },
      };

      setCurrentRide(ride);
      addNotification({
        title: "Ride Requested",
        message: "Looking for an available ACT vehicle...",
        type: "ride-update",
      });

      setTimeout(() => {
        setCurrentRide((prev) => {
          if (!prev) return null;

          const vehicleModels = [
            "ACT Golf Cart",
            "E-Z-GO RXV",
            "Club Car Onward",
            "Yamaha Drive2",
          ];
          const vehicleColors = [
            "White",
            "Silver",
            "Navy Blue",
            "Forest Green",
          ];

          const vehicleId = `ACT-${String(Math.floor(Math.random() * 999) + 1).padStart(3, "0")}`;
          const licensePlate = `ACT-${Math.floor(1000 + Math.random() * 9000)}`;

          const updated = {
            ...prev,
            status: "assigned" as RideStatus,
            vehicleId,
            vehicle: {
              id: vehicleId,
              model:
                vehicleModels[Math.floor(Math.random() * vehicleModels.length)],
              color:
                vehicleColors[Math.floor(Math.random() * vehicleColors.length)],
              licensePlate,
            },
          };
          addNotification({
            title: "Cart Assigned",
            message: `Self-driving cart ${updated.vehicleId} is on the way!`,
            type: "ride-update",
          });
          return updated;
        });
      }, 2000);
    },
    [addNotification]
  );

  const updateRideStatus = useCallback(
    (
      status: RideStatus,
      vehiclePosition?: { latitude: number; longitude: number }
    ) => {
      setCurrentRide((prev) => {
        if (!prev) return null;

        const updatedRide: Ride = {
          ...prev,
          status,
          vehiclePosition,
        };

        if (status === "arriving") {
          updatedRide.pickupAt = new Date();
          addNotification({
            title: "Cart Arriving",
            message: "Your self-driving cart is almost there!",
            type: "ride-update",
          });
        } else if (status === "in-progress") {
          addNotification({
            title: "Trip Started",
            message: "Enjoy your autonomous ride!",
            type: "ride-update",
          });
        } else if (status === "completed") {
          updatedRide.completedAt = new Date();
          setRideHistory((prevHistory) => {
            const newHistory = [updatedRide, ...prevHistory];
            saveRides(newHistory);
            return newHistory;
          });
          addNotification({
            title: "Trip Completed",
            message: "Thanks for using ACT!",
            type: "ride-update",
          });
          return null;
        }

        return updatedRide;
      });
    },
    [addNotification]
  );

  const cancelRide = useCallback(() => {
    setCurrentRide((prev) => {
      if (prev) {
        addNotification({
          title: "Ride Cancelled",
          message: "Your ride has been cancelled.",
          type: "ride-update",
        });
        return null;
      }
      return prev;
    });
  }, [addNotification]);

  const markNotificationAsRead = useCallback((id: string) => {
    setNotifications((prev) => {
      const updated = prev.map((n) => (n.id === id ? { ...n, read: true } : n));
      saveNotifications(updated);
      return updated;
    });
  }, []);

  const markAllNotificationsAsRead = useCallback(() => {
    setNotifications((prev) => {
      const updated = prev.map((n) => ({ ...n, read: true }));
      saveNotifications(updated);
      return updated;
    });
  }, []);

  return useMemo(
    () => ({
      currentRide,
      rideHistory,
      notifications,
      isLoading,
      requestRide,
      updateRideStatus,
      cancelRide,
      markNotificationAsRead,
      markAllNotificationsAsRead,
    }),
    [
      currentRide,
      rideHistory,
      notifications,
      isLoading,
      requestRide,
      updateRideStatus,
      cancelRide,
      markNotificationAsRead,
      markAllNotificationsAsRead,
    ]
  );
});
