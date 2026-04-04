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

export function hasActiveActivity(): boolean {
  return currentActivityId !== null;
}

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
    const targetName =
      params.status === "in_progress"
        ? params.dropoffName
        : params.pickupName;

    const state = {
      title: statusText,
      subtitle: targetName,
    };

    const config = {
      backgroundColor: "#111111",
      titleColor: "#F5F5F5",
      subtitleColor: "#A1A1AA",
      progressViewTint: "#2DD4BF",
      deepLinkUrl: "/plan-ride",
    };

    const id = LA.startActivity(state, config);
    currentActivityId = id ?? null;
  } catch (e) {
    console.log("Failed to start Live Activity:", e);
  }
}

export async function updateRideActivity(
  status: string,
  distanceInfo?: {
    distanceMiles: number;
    progress: number;
    destinationName: string;
  }
): Promise<void> {
  const LA = await getLiveActivity();
  if (!LA || !currentActivityId) return;

  try {
    const statusText = STATUS_TEXT[status] ?? "Ride active";

    const state: {
      title: string;
      subtitle?: string;
      progressBar?: { progress: number };
    } = { title: statusText };

    if (distanceInfo) {
      const d = distanceInfo.distanceMiles;
      const distStr = d < 0.1 ? `${Math.round(d * 5280)} ft` : `${d.toFixed(1)} mi`;
      // "0.3 mi · Engineering I" — Swift widget parses distance vs destination
      state.subtitle = `${distStr} \u00B7 ${distanceInfo.destinationName}`;
      state.progressBar = { progress: distanceInfo.progress };
    }

    LA.updateActivity(currentActivityId, state);
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
