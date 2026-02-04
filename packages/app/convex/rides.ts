import { v } from "convex/values";
import { mutation, query } from "./_generated/server";

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
    const identity = await ctx.auth.getUserIdentity();
    if (!identity) {
      throw new Error("Unauthenticated called to requestRide");
    }
    // const user = await ctx.db
    //   .query("users")
    //   .withIndex("by_token", (q) => q.eq("tokenIdentifier", identity.tokenIdentifier))
    //   .unique();

    // if (!user) throw new Error("User not found");

    // const rideId = await ctx.db.insert("rides", {
    //   userId: user._id,
    //   origin: args.origin,
    //   destination: args.destination,
    //   status: "requested",
    //   startTime: Date.now(),
    // });
    // return rideId;
    return "mock_ride_id";
  },
});

export const getActiveRide = query({
  args: {},
  handler: async (ctx) => {
    // const identity = await ctx.auth.getUserIdentity();
    // if (!identity) return null;
    // ... logic to find active ride
    return null;
  },
});
