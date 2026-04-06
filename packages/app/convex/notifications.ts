"use node";

import { v } from "convex/values";
import { internalAction } from "./_generated/server";
import { internal } from "./_generated/api";

export const sendPush = internalAction({
  args: {
    userId: v.id("users"),
    title: v.string(),
    body: v.string(),
    rideStatus: v.optional(v.string()),
  },
  handler: async (ctx, args) => {
    const user = await ctx.runQuery(internal.users.getUserById, {
      userId: args.userId,
    });

    if (!user) {
      console.error(`[push] User ${args.userId} not found`);
      return;
    }

    if (!user.expoPushToken) {
      console.warn(
        `[push] No push token for user ${args.userId} (${user.name}), skipping`
      );
      return;
    }

    const token = user.expoPushToken;
    if (!token.startsWith("ExponentPushToken[")) {
      console.error(`[push] Invalid token format for user ${args.userId}: ${token}`);
      return;
    }

    const message = {
      to: token,
      sound: "default" as const,
      title: args.title,
      body: args.body,
      data: { type: "ride-update", rideStatus: args.rideStatus },
      priority: "high" as const,
    };

    try {
      const response = await fetch("https://exp.host/--/api/v2/push/send", {
        method: "POST",
        headers: {
          Accept: "application/json",
          "Accept-encoding": "gzip, deflate",
          "Content-Type": "application/json",
        },
        body: JSON.stringify(message),
      });

      if (!response.ok) {
        console.error(
          `[push] Expo API returned ${response.status}: ${await response.text()}`
        );
        return;
      }

      const result = await response.json();

      // Expo returns { data: { status, id?, message?, details? } }
      const ticket = result.data;
      if (ticket?.status === "error") {
        console.error(
          `[push] Expo ticket error: ${ticket.message} (${ticket.details?.error ?? "unknown"})`
        );
        // If the token is invalid, clear it so we don't keep retrying
        if (ticket.details?.error === "DeviceNotRegistered") {
          console.warn(`[push] Clearing invalid token for user ${args.userId}`);
          await ctx.runMutation(internal.users.clearPushToken, {
            userId: args.userId,
          });
        }
      } else {
        console.log(
          `[push] Sent to ${user.name}: "${args.title}" (ticket: ${ticket?.id ?? "ok"})`
        );
      }
    } catch (error) {
      console.error("[push] Network error sending push notification:", error);
    }
  },
});
