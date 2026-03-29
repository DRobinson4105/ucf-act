"use node";

import { v } from "convex/values";
import { internalAction } from "./_generated/server";
import { internal } from "./_generated/api";

export const sendPush = internalAction({
  args: {
    userId: v.id("users"),
    title: v.string(),
    body: v.string(),
  },
  handler: async (ctx, args) => {
    const user = await ctx.runQuery(internal.users.getUserById, {
      userId: args.userId,
    });

    if (!user?.expoPushToken) {
      console.log(`No push token for user ${args.userId}, skipping`);
      return;
    }

    const message = {
      to: user.expoPushToken,
      sound: "default",
      title: args.title,
      body: args.body,
      data: { type: "ride-update" },
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

      const result = await response.json();
      console.log("Push notification sent:", result);
    } catch (error) {
      console.error("Failed to send push notification:", error);
    }
  },
});
