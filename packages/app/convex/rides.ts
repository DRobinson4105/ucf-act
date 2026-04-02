import { v } from "convex/values";
import {
  internalMutation,
  internalQuery,
  mutation,
  query,
} from "./_generated/server";
import { internal } from "./_generated/api";

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

    await ctx.scheduler.runAfter(0, internal.notifications.sendPush, {
      userId: ride.userId,
      title: "Ride Cancelled",
      body: "Your ride has been cancelled.",
    });
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

    // Send push notification for critical status changes
    const ride = await ctx.db.get(args.rideId);
    if (!ride) return;

    const pushMessages: Record<string, { title: string; body: string }> = {
      assigned: { title: "Cart Found", body: "A self-driving cart is on the way to your pickup!" },
      arriving: { title: "Cart Arrived", body: "Your cart has arrived! Head to your pickup." },
      completed: { title: "Trip Complete", body: "Thanks for riding with ACT!" },
      cancelled: { title: "Ride Cancelled", body: "Your ride has been cancelled." },
    };

    const msg = pushMessages[args.status];
    if (msg) {
      await ctx.scheduler.runAfter(0, internal.notifications.sendPush, {
        userId: ride.userId,
        title: msg.title,
        body: msg.body,
      });
    }
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

// Called by HMI — finds the active ride currently assigned to a specific cart
export const getActiveRideForCart = query({
  args: { cartId: v.id("carts") },
  handler: async (ctx, args) => {
    for (const status of ["assigned", "arriving", "in_progress"] as const) {
      const ride = await ctx.db
        .query("rides")
        .withIndex("by_status", (q: any) => q.eq("status", status))
        .filter((q: any) => q.eq(q.field("cartId"), args.cartId))
        .first();
      if (ride) return ride;
    }
    return null;
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
