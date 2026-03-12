import { httpRouter } from "convex/server";
import { httpAction } from "./_generated/server";
import { internal } from "./_generated/api";
import { Id } from "./_generated/dataModel";

const http = httpRouter();

function isAuthorized(request: Request): boolean {
  const secret = process.env.CART_BRIDGE_SECRET;
  if (!secret) return false;
  return request.headers.get("Authorization") === `Bearer ${secret}`;
}

// POST /api/cart/telemetry
// Body: { cartId, latitude, longitude, heading?, batteryLevel?, status? }
http.route({
  path: "/api/cart/telemetry",
  method: "POST",
  handler: httpAction(async (ctx, request) => {
    if (!isAuthorized(request)) {
      return new Response("Unauthorized", { status: 401 });
    }

    const body = await request.json();
    const { cartId, latitude, longitude, heading, batteryLevel, status } = body;

    if (!cartId || latitude === undefined || longitude === undefined) {
      return new Response("Missing required fields", { status: 400 });
    }

    await ctx.runMutation(internal.carts.updateLocation, {
      cartId: cartId as Id<"carts">,
      latitude,
      longitude,
      heading,
      batteryLevel,
      status,
    });

    return new Response(JSON.stringify({ ok: true }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }),
});

// GET /api/cart/assignments?cartId=xxx
// Cart bridge polls this to pick up pending rides.
http.route({
  path: "/api/cart/assignments",
  method: "GET",
  handler: httpAction(async (ctx, request) => {
    if (!isAuthorized(request)) {
      return new Response("Unauthorized", { status: 401 });
    }

    const url = new URL(request.url);
    const cartId = url.searchParams.get("cartId");

    if (!cartId) {
      return new Response("Missing cartId", { status: 400 });
    }

    const ride = await ctx.runQuery(internal.rides.getNextPendingRide, {});

    if (!ride) {
      return new Response(JSON.stringify({ ride: null }), {
        status: 200,
        headers: { "Content-Type": "application/json" },
      });
    }

    // Assign the ride to this cart
    await ctx.runMutation(internal.rides.updateStatus, {
      rideId: ride._id,
      status: "assigned",
      cartId: cartId as Id<"carts">,
    });

    return new Response(JSON.stringify({ ride }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }),
});

// POST /api/cart/arrived-pickup
// Body: { rideId } — cart bridge signals it has reached the pickup point
http.route({
  path: "/api/cart/arrived-pickup",
  method: "POST",
  handler: httpAction(async (ctx, request) => {
    if (!isAuthorized(request)) {
      return new Response("Unauthorized", { status: 401 });
    }

    const { rideId } = await request.json();
    if (!rideId) {
      return new Response("Missing rideId", { status: 400 });
    }

    await ctx.runMutation(internal.rides.updateStatus, {
      rideId: rideId as Id<"rides">,
      status: "arriving",
    });

    return new Response(JSON.stringify({ ok: true }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }),
});

// GET /api/cart/ride-status?rideId=xxx
// Cart bridge polls this to know when the user has boarded (in_progress)
http.route({
  path: "/api/cart/ride-status",
  method: "GET",
  handler: httpAction(async (ctx, request) => {
    if (!isAuthorized(request)) {
      return new Response("Unauthorized", { status: 401 });
    }

    const url = new URL(request.url);
    const rideId = url.searchParams.get("rideId");
    if (!rideId) {
      return new Response("Missing rideId", { status: 400 });
    }

    const ride = await ctx.runQuery(internal.rides.getRideById, { rideId: rideId as Id<"rides"> });
    if (!ride) {
      return new Response(JSON.stringify({ status: null }), {
        status: 200,
        headers: { "Content-Type": "application/json" },
      });
    }

    return new Response(JSON.stringify({ status: ride.status }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }),
});

// POST /api/cart/ride-complete
// Body: { rideId }
http.route({
  path: "/api/cart/ride-complete",
  method: "POST",
  handler: httpAction(async (ctx, request) => {
    if (!isAuthorized(request)) {
      return new Response("Unauthorized", { status: 401 });
    }

    const { rideId } = await request.json();

    if (!rideId) {
      return new Response("Missing rideId", { status: 400 });
    }

    await ctx.runMutation(internal.rides.updateStatus, {
      rideId: rideId as Id<"rides">,
      status: "completed",
    });

    return new Response(JSON.stringify({ ok: true }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }),
});

export default http;
