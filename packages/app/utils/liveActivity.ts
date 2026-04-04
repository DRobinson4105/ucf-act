import { Platform } from "react-native";

let LiveActivity: typeof import("expo-live-activity") | null = null;

async function getLiveActivity() {
  if (Platform.OS !== "ios") return null;
  if (!LiveActivity) {
    try {
      LiveActivity = await import("expo-live-activity");
    } catch {
      return null;
    }
  }
  return LiveActivity;
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
  const LA = await getLiveActivity();
  if (!LA) return;

  try {
    const statusText = STATUS_TEXT[params.status] ?? "Ride active";

    const state = {
      title: statusText,
      subtitle: `${params.pickupName} → ${params.dropoffName}`,
    };

    const config = {
      backgroundColor: "#111111",
      titleColor: "#2DD4BF",
      subtitleColor: "#A1A1AA",
      deepLinkUrl: "/plan-ride",
    };

    const id = LA.startActivity(state, config);
    currentActivityId = id ?? null;
  } catch (e) {
    console.log("Failed to start Live Activity:", e);
  }
}

export async function updateRideActivity(status: string): Promise<void> {
  const LA = await getLiveActivity();
  if (!LA || !currentActivityId) return;

  try {
    LA.updateActivity(currentActivityId, {
      title: STATUS_TEXT[status] ?? "Ride active",
    });
  } catch (e) {
    console.log("Failed to update Live Activity:", e);
  }
}

export async function endRideActivity(): Promise<void> {
  const LA = await getLiveActivity();
  if (!LA || !currentActivityId) return;

  try {
    LA.stopActivity(currentActivityId, {
      title: "Ride complete",
      subtitle: "Thanks for riding with ACT!",
    });
    currentActivityId = null;
  } catch (e) {
    console.log("Failed to end Live Activity:", e);
  }
}
