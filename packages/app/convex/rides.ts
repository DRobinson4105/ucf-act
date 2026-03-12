import { v } from "convex/values";
import {
  internalMutation,
  internalQuery,
  mutation,
  query,
} from "./_generated/server";

const locationArg = v.object({
  latitude: v.number(),
  longitude: v.number(),
  name: v.optional(v.string()),
});

async function getAuthUser(ctx: { auth: { getUserIdentity: () => Promise<{ tokenIdentifier: string } | null> }; db: any }) {
  const identity = await ctx.auth.getUserIdentity();
  if (!identity) return null;
  return await ctx.db
    .query("users")
    .withIndex("by_token", (q: any) => q.eq("tokenIdentifier", identity.tokenIdentifier))
    .unique();
}

export const requestRide = mutation({
  args: {
    origin: locationArg,
    destination: locationArg,
  },
  handler: async (ctx, args) => {
    const user = await getAuthUser(ctx);
    if (!user) throw new Error("Unauthenticated");

    // Cancel any existing non-terminal rides for this user
    const existing = await ctx.db
      .query("rides")
      .withIndex("by_user", (q: any) => q.eq("userId", user._id))
      .filter((q: any) =>
        q.and(
          q.neq(q.field("status"), "completed"),
          q.neq(q.field("status"), "cancelled")
        )
      )
      .first();
    if (existing) {
      await ctx.db.patch(existing._id, { status: "cancelled" });
    }

    return await ctx.db.insert("rides", {
      userId: user._id,
      origin: args.origin,
      destination: args.destination,
      status: "requested",
      startTime: Date.now(),
    });
  },
});

export const getActiveRide = query({
  args: {},
  handler: async (ctx) => {
    const user = await getAuthUser(ctx);
    if (!user) return null;

    return await ctx.db
      .query("rides")
      .withIndex("by_user", (q: any) => q.eq("userId", user._id))
      .filter((q: any) =>
        q.and(
          q.neq(q.field("status"), "completed"),
          q.neq(q.field("status"), "cancelled")
        )
      )
      .order("desc")
      .first();
  },
});

export const getRideHistory = query({
  args: {},
  handler: async (ctx) => {
    const user = await getAuthUser(ctx);
    if (!user) return [];

    return await ctx.db
      .query("rides")
      .withIndex("by_user", (q: any) => q.eq("userId", user._id))
      .filter((q: any) => q.eq(q.field("status"), "completed"))
      .order("desc")
      .take(50);
  },
});

export const cancelRide = mutation({
  args: { rideId: v.id("rides") },
  handler: async (ctx, args) => {
    const user = await getAuthUser(ctx);
    if (!user) throw new Error("Unauthenticated");

    const ride = await ctx.db.get(args.rideId);
    if (!ride || ride.userId !== user._id) throw new Error("Ride not found");
    if (ride.status === "completed" || ride.status === "cancelled") {
      throw new Error("Ride already ended");
    }

    await ctx.db.patch(args.rideId, { status: "cancelled" });
  },
});

// Called by the user when they've boarded the cart at pickup
export const boardRide = mutation({
  args: { rideId: v.id("rides") },
  handler: async (ctx, args) => {
    const user = await getAuthUser(ctx);
    if (!user) throw new Error("Unauthenticated");
    const ride = await ctx.db.get(args.rideId);
    if (!ride || ride.userId !== user._id) throw new Error("Ride not found");
    if (ride.status !== "arriving") throw new Error("Cart is not at pickup yet");
    await ctx.db.patch(args.rideId, { status: "in_progress" });
  },
});

// Called only by HTTP actions (cart bridge)
export const updateStatus = internalMutation({
  args: {
    rideId: v.id("rides"),
    status: v.union(
      v.literal("requested"),
      v.literal("assigned"),
      v.literal("arriving"),
      v.literal("in_progress"),
      v.literal("completed"),
      v.literal("cancelled")
    ),
    cartId: v.optional(v.id("carts")),
  },
  handler: async (ctx, args) => {
    const patch: Record<string, unknown> = { status: args.status };
    if (args.cartId !== undefined) patch.cartId = args.cartId;
    if (args.status === "completed") patch.endTime = Date.now();
    await ctx.db.patch(args.rideId, patch);
  },
});

// Called only by HTTP actions (cart bridge polling ride status)
export const getRideById = internalQuery({
  args: { rideId: v.id("rides") },
  handler: async (ctx, args) => {
    return await ctx.db.get(args.rideId);
  },
});

export const addReview = mutation({
  args: {
    rideId: v.id("rides"),
    rating: v.number(),
    reviewComment: v.optional(v.string()),
  },
  handler: async (ctx, args) => {
    const user = await getAuthUser(ctx);
    if (!user) throw new Error("Unauthenticated");
    const ride = await ctx.db.get(args.rideId);
    if (!ride || ride.userId !== user._id) throw new Error("Ride not found");
    await ctx.db.patch(args.rideId, {
      rating: args.rating,
      reviewComment: args.reviewComment,
    });
  },
});

// Called only by HTTP actions (cart bridge polling for assignments)
export const getNextPendingRide = internalQuery({
  args: {},
  handler: async (ctx) => {
    return await ctx.db
      .query("rides")
      .withIndex("by_status", (q: any) => q.eq("status", "requested"))
      .order("asc")
      .first();
  },
});
