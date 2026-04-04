import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import { api } from "@/convex/_generated/api";
import { Id } from "@/convex/_generated/dataModel";
import { Notification, Ride, RideStatus } from "@/types/ride";
import {
  startRideActivity,
  updateRideActivity,
  endRideActivity,
  hasActiveActivity,
} from "@/utils/liveActivity";
import createContextHook from "@nkzw/create-context-hook";
import { useMutation, useQuery } from "convex/react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";

function haversineDistanceMi(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 3958.8;
  const toRad = Math.PI / 180;
  const dLat = (lat2 - lat1) * toRad;
  const dLon = (lon2 - lon1) * toRad;
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

export interface PendingReview {
  rideId: Id<"rides">;
  pickupLocationId: string;
  dropoffLocationId: string;
}

function findLocationId(lat: number, lon: number): string {
  // Match a Convex ride's lat/lon back to a CAMPUS_LOCATIONS id
  let best = CAMPUS_LOCATIONS[0];
  let bestDist = Infinity;
  for (const loc of CAMPUS_LOCATIONS) {
    const d =
      Math.abs(loc.latitude - lat) + Math.abs(loc.longitude - lon);
    if (d < bestDist) {
      bestDist = d;
      best = loc;
    }
  }
  return best.id;
}

export const [RideProvider, useRide] = createContextHook(() => {
  const activeRideData = useQuery(api.rides.getActiveRide);
  const rideHistoryData = useQuery(api.rides.getRideHistory);
  const assignedCartId = activeRideData?.cartId;
  const assignedCart = useQuery(
    api.carts.getById,
    assignedCartId ? { cartId: assignedCartId } : "skip"
  );
  const requestRideMutation = useMutation(api.rides.requestRide);
  const cancelRideMutation = useMutation(api.rides.cancelRide);
  const boardRideMutation = useMutation(api.rides.boardRide);
  const addReviewMutation = useMutation(api.rides.addReview);

  const [notifications, setNotifications] = useState<Notification[]>([]);
  const [pendingReview, setPendingReview] = useState<PendingReview | null>(null);
  const prevStatusRef = useRef<RideStatus | null>(null);
  const prevLAStatusRef = useRef<RideStatus | null>(null);
  const prevRideRef = useRef<Ride | null>(null);
  const initialDistanceRef = useRef<number | null>(null);
  const prevDistTargetRef = useRef<string | null>(null);

  // Map Convex activeRide to the local Ride type consumed by UI components
  const currentRide: Ride | null = useMemo(() => {
    if (!activeRideData) return null;
    return {
      id: activeRideData._id,
      convexId: activeRideData._id,
      cartId: activeRideData.cartId,
      pickupLocationId: findLocationId(
        activeRideData.origin.latitude,
        activeRideData.origin.longitude
      ),
      dropoffLocationId: findLocationId(
        activeRideData.destination.latitude,
        activeRideData.destination.longitude
      ),
      status: activeRideData.status as RideStatus,
      requestedAt: new Date(activeRideData.startTime),
      completedAt: activeRideData.endTime
        ? new Date(activeRideData.endTime)
        : undefined,
      vehicle: {
        id: activeRideData.cartId ?? "",
        model: "ACT Golf Cart",
        color: "White",
      },
    };
  }, [activeRideData]);

  // Map completed Convex rides to the local Ride type for history
  const rideHistory: Ride[] = useMemo(() => {
    if (!rideHistoryData) return [];
    return rideHistoryData.map((r) => ({
      id: r._id,
      convexId: r._id,
      cartId: r.cartId,
      pickupLocationId: findLocationId(
        r.origin.latitude,
        r.origin.longitude
      ),
      dropoffLocationId: findLocationId(
        r.destination.latitude,
        r.destination.longitude
      ),
      status: r.status as RideStatus,
      requestedAt: new Date(r.startTime),
      completedAt: r.endTime ? new Date(r.endTime) : undefined,
      vehicle: {
        id: r.cartId ?? "",
        model: "ACT Golf Cart",
        color: "White",
      },
      rating: r.rating,
      reviewComment: r.reviewComment,
    }));
  }, [rideHistoryData]);

  // Detect ride completion (in_progress → gone) to trigger review screen
  useEffect(() => {
    const prev = prevRideRef.current;
    prevRideRef.current = currentRide;
    if (prev?.status === "in_progress" && !currentRide && prev.convexId) {
      setPendingReview({
        rideId: prev.convexId as Id<"rides">,
        pickupLocationId: prev.pickupLocationId,
        dropoffLocationId: prev.dropoffLocationId,
      });
    }
  }, [currentRide]);

  // Derive notifications from status transitions
  useEffect(() => {
    const newStatus = currentRide?.status ?? null;
    const prev = prevStatusRef.current;
    if (newStatus === prev) return;
    prevStatusRef.current = newStatus;

    if (!newStatus) return;

    const notif = (() => {
      switch (newStatus) {
        case "requested":
          return { title: "Ride Requested", message: "Looking for an available ACT cart...", type: "ride-update" as const };
        case "assigned":
          return { title: "Cart Assigned", message: `A self-driving cart is on the way!`, type: "ride-update" as const };
        case "arriving":
          return { title: "Cart Arriving", message: "Your cart is almost there!", type: "ride-update" as const };
        case "in_progress":
          return { title: "Trip Started", message: "Enjoy your autonomous ride!", type: "ride-update" as const };
        case "completed":
          return { title: "Trip Completed", message: "Thanks for using ACT!", type: "ride-update" as const };
        case "cancelled":
          return { title: "Ride Cancelled", message: "Your ride has been cancelled.", type: "ride-update" as const };
        default:
          return null;
      }
    })();

    if (notif) {
      const notification: Notification = {
        id: Date.now().toString(),
        ...notif,
        timestamp: new Date(),
        read: false,
      };
      setNotifications((prev) => [notification, ...prev]);
    }
  }, [currentRide?.status]);

  // Trigger Live Activity start/stop on status transitions (uses its own ref to
  // avoid the race where the notification effect above already updated prevStatusRef)
  useEffect(() => {
    const status = currentRide?.status ?? null;
    const prev = prevLAStatusRef.current;
    if (status === prev) return;
    prevLAStatusRef.current = status;

    // Reset distance tracking when target changes
    const targetType = status === "in_progress" ? "dropoff" : "pickup";
    if (targetType !== prevDistTargetRef.current) {
      initialDistanceRef.current = null;
      prevDistTargetRef.current = targetType;
    }

    if (!status) {
      if (prev) endRideActivity();
      return;
    }

    if (status === "assigned") {
      const pickupLoc = CAMPUS_LOCATIONS.find(
        (l) => l.id === currentRide!.pickupLocationId
      );
      const dropoffLoc = CAMPUS_LOCATIONS.find(
        (l) => l.id === currentRide!.dropoffLocationId
      );
      startRideActivity({
        cartName: "ACT-001",
        pickupName: pickupLoc?.shortName ?? "Pickup",
        dropoffName: dropoffLoc?.shortName ?? "Destination",
        status: "assigned",
      });
    } else if (status === "completed" || status === "cancelled") {
      endRideActivity();
    }
    // "arriving" and "in_progress" are handled by the distance-tracking effect below
  }, [currentRide?.status, currentRide?.pickupLocationId, currentRide?.dropoffLocationId]);

  // Update Live Activity with distance/progress whenever cart position changes
  useEffect(() => {
    if (!currentRide || !activeRideData || !assignedCart?.location) return;
    if (!hasActiveActivity()) return;

    const status = currentRide.status;
    if (!["assigned", "arriving", "in_progress"].includes(status)) return;

    const target =
      status === "in_progress"
        ? activeRideData.destination
        : activeRideData.origin;

    const dist = haversineDistanceMi(
      assignedCart.location.latitude,
      assignedCart.location.longitude,
      target.latitude,
      target.longitude
    );

    if (initialDistanceRef.current === null || dist > initialDistanceRef.current) {
      initialDistanceRef.current = dist;
    }

    const total = initialDistanceRef.current;
    const progress = total > 0.01 ? Math.max(0, Math.min(1, 1 - dist / total)) : 0;

    const destLoc =
      status === "in_progress"
        ? CAMPUS_LOCATIONS.find((l) => l.id === currentRide.dropoffLocationId)
        : CAMPUS_LOCATIONS.find((l) => l.id === currentRide.pickupLocationId);

    updateRideActivity(status, {
      distanceMiles: dist,
      progress,
      destinationName: destLoc?.shortName ?? "Destination",
    });
  }, [
    assignedCart?.location?.latitude,
    assignedCart?.location?.longitude,
    currentRide?.status,
    activeRideData?.origin,
    activeRideData?.destination,
  ]);

  const requestRide = useCallback(
    async (
      pickup: { latitude: number; longitude: number; name: string },
      dropoff: { latitude: number; longitude: number; name: string }
    ) => {
      await requestRideMutation({
        origin: { latitude: pickup.latitude, longitude: pickup.longitude, name: pickup.name },
        destination: { latitude: dropoff.latitude, longitude: dropoff.longitude, name: dropoff.name },
      });
    },
    [requestRideMutation]
  );

  const cancelRide = useCallback(async () => {
    if (!currentRide?.convexId) return;
    await cancelRideMutation({ rideId: currentRide.convexId as Id<"rides"> });
  }, [cancelRideMutation, currentRide?.convexId]);

  const boardRide = useCallback(async () => {
    if (!currentRide?.convexId) return;
    await boardRideMutation({ rideId: currentRide.convexId as Id<"rides"> });
  }, [boardRideMutation, currentRide?.convexId]);

  const updateRideStatus = useCallback(
    (_status: RideStatus, _vehiclePosition?: { latitude: number; longitude: number }) => {
      // Status updates come from the cart bridge via Convex — no-op on client
    },
    []
  );

  const submitReview = useCallback(
    async (rating: number, comment?: string) => {
      if (!pendingReview) return;
      await addReviewMutation({
        rideId: pendingReview.rideId,
        rating,
        reviewComment: comment,
      });
      setPendingReview(null);
    },
    [addReviewMutation, pendingReview]
  );

  const dismissReview = useCallback(() => {
    setPendingReview(null);
  }, []);

  const markNotificationAsRead = useCallback((id: string) => {
    setNotifications((prev) =>
      prev.map((n) => (n.id === id ? { ...n, read: true } : n))
    );
  }, []);

  const markAllNotificationsAsRead = useCallback(() => {
    setNotifications((prev) => prev.map((n) => ({ ...n, read: true })));
  }, []);

  const isLoading = activeRideData === undefined || rideHistoryData === undefined;

  return useMemo(
    () => ({
      currentRide,
      rideHistory,
      notifications,
      isLoading,
      pendingReview,
      requestRide,
      updateRideStatus,
      cancelRide,
      boardRide,
      submitReview,
      dismissReview,
      markNotificationAsRead,
      markAllNotificationsAsRead,
    }),
    [
      currentRide,
      rideHistory,
      notifications,
      isLoading,
      pendingReview,
      requestRide,
      updateRideStatus,
      cancelRide,
      boardRide,
      submitReview,
      dismissReview,
      markNotificationAsRead,
      markAllNotificationsAsRead,
    ]
  );
});
