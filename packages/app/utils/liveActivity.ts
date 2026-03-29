import { Platform } from "react-native";

let LiveActivities: any = null;

async function getLiveActivities() {
  if (Platform.OS !== "ios") return null;
  if (!LiveActivities) {
    try {
      // @ts-expect-error — experimental package, no published type declarations
      LiveActivities = await import("expo-live-activities");
    } catch {
      return null;
    }
  }
  return LiveActivities;
}

interface RideActivityState {
  status: string;
  statusText: string;
  cartName: string;
  pickupName: string;
  dropoffName: string;
}

const STATUS_TEXT: Record<string, string> = {
  assigned: "Cart on the way",
  arriving: "Cart arrived!",
  in_progress: "On your way",
};

let currentActivityId: string | null = null;

export async function startRideActivity(params: {
  cartName: string;
  pickupName: string;
  dropoffName: string;
  status: string;
}): Promise<void> {
  const LA = await getLiveActivities();
  if (!LA) return;

  try {
    const state: RideActivityState = {
      status: params.status,
      statusText: STATUS_TEXT[params.status] ?? "Ride active",
      cartName: params.cartName,
      pickupName: params.pickupName,
      dropoffName: params.dropoffName,
    };

    const result = await LA.startActivity("RideActivityAttributes", state);
    currentActivityId = result?.id ?? null;
  } catch (e) {
    console.log("Failed to start Live Activity:", e);
  }
}

export async function updateRideActivity(status: string): Promise<void> {
  const LA = await getLiveActivities();
  if (!LA || !currentActivityId) return;

  try {
    await LA.updateActivity(currentActivityId, {
      status,
      statusText: STATUS_TEXT[status] ?? "Ride active",
    });
  } catch (e) {
    console.log("Failed to update Live Activity:", e);
  }
}

export async function endRideActivity(): Promise<void> {
  const LA = await getLiveActivities();
  if (!LA || !currentActivityId) return;

  try {
    await LA.endActivity(currentActivityId);
    currentActivityId = null;
  } catch (e) {
    console.log("Failed to end Live Activity:", e);
  }
}
