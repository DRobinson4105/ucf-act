import { v } from "convex/values";
import { internalMutation, mutation, query } from "./_generated/server";

export const getAll = query({
  args: {},
  handler: async (ctx) => {
    return await ctx.db.query("carts").collect();
  },
});

export const getById = query({
  args: { cartId: v.id("carts") },
  handler: async (ctx, args) => {
    return await ctx.db.get(args.cartId);
  },
});

// Called only by the cart bridge via HTTP action
export const updateLocation = internalMutation({
  args: {
    cartId: v.id("carts"),
    latitude: v.number(),
    longitude: v.number(),
    heading: v.optional(v.number()),
    batteryLevel: v.optional(v.number()),
    status: v.optional(
      v.union(
        v.literal("idle"),
        v.literal("busy"),
        v.literal("charging"),
        v.literal("offline")
      )
    ),
    speed: v.optional(v.number()),
  },
  handler: async (ctx, args) => {
    const { cartId, latitude, longitude, heading, batteryLevel, status, speed } = args;

    const patch: Record<string, unknown> = {
      location: { latitude, longitude, heading },
      lastUpdated: Date.now(),
    };
    if (batteryLevel !== undefined) patch.batteryLevel = batteryLevel;
    if (status !== undefined) patch.status = status;
    if (speed !== undefined) patch.speed = speed;

    await ctx.db.patch(cartId, patch);
  },
});

// Called only by the cart bridge via HTTP action — replaces route on each phase transition
export const updateRoute = internalMutation({
  args: {
    cartId: v.id("carts"),
    waypoints: v.array(v.object({ latitude: v.number(), longitude: v.number() })),
  },
  handler: async (ctx, args) => {
    await ctx.db.patch(args.cartId, { currentRoute: args.waypoints });
  },
});

// One-time seeding — run once from Convex dashboard to create the ACT-001 cart
export const seed = mutation({
  args: {},
  handler: async (ctx) => {
    const existing = await ctx.db
      .query("carts")
      .filter((q) => q.eq(q.field("name"), "ACT-001"))
      .first();
    if (existing) return existing._id;

    return await ctx.db.insert("carts", {
      name: "ACT-001",
      location: { latitude: 28.6024, longitude: -81.2001, heading: 0 },
      status: "idle",
      batteryLevel: 100,
      lastUpdated: Date.now(),
    });
  },
});
