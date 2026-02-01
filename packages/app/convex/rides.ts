import { v } from "convex/values";
import { mutation, query } from "./_generated/server";
import { auth } from "./auth";

export const requestRide = mutation({
  args: {
    origin: v.object({
      latitude: v.number(),
      longitude: v.number(),
      name: v.optional(v.string()),
    }),
    destination: v.object({
      latitude: v.number(),
      longitude: v.number(),
      name: v.optional(v.string()),
    }),
  },
  handler: async (ctx, args) => {
    const userId = await auth.getUserId(ctx);
    if (!userId) {
      throw new Error("Unauthenticated call to requestRide");
    }

    // Placeholder for ride creation logic
    // const rideId = await ctx.db.insert("rides", {
    //   userId: userId,
    //   origin: args.origin,
    //   destination: args.destination,
    //   status: "requested",
    //   startTime: Date.now(),
    // });

    return "mock_ride_id";
  },
});

export const getActiveRide = query({
  args: {},
  handler: async (ctx) => {
    // return null;
    return null;
  },
});

export const myHistory = query({
  args: {},
  handler: async (ctx) => {
    const userId = await auth.getUserId(ctx);
    if (!userId) return [];

    const rides = await ctx.db
      .query("rides")
      .withIndex("by_user", (q) => q.eq("userId", userId))
      .order("desc") // Most recent first
      .collect();

    return rides;
  },
});
