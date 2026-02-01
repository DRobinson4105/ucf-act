import { defineSchema, defineTable } from "convex/server";
import { v } from "convex/values";

export default defineSchema({
  users: defineTable({
    name: v.string(),
    email: v.optional(v.string()),
    phone: v.optional(v.string()),
    // We'll store the auth provider ID here (e.g. from the token)
    tokenIdentifier: v.string(),
    settings: v.optional(
      v.object({
        notificationsEnabled: v.boolean(),
      })
    ),
  }).index("by_token", ["tokenIdentifier"]),

  rides: defineTable({
    userId: v.id("users"),
    cartId: v.optional(v.id("carts")),
    startTime: v.number(), // Unix timestamp
    endTime: v.optional(v.number()),
    status: v.union(
      v.literal("requested"),
      v.literal("assigned"),
      v.literal("in_progress"),
      v.literal("completed"),
      v.literal("cancelled")
    ),
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
  })
    .index("by_user", ["userId"])
    .index("by_status", ["status"]),

  carts: defineTable({
    name: v.string(),
    location: v.object({
      latitude: v.number(),
      longitude: v.number(),
      heading: v.optional(v.number()),
    }),
    status: v.union(
      v.literal("idle"),
      v.literal("busy"),
      v.literal("charging"),
      v.literal("offline")
    ),
    batteryLevel: v.number(),
    lastUpdated: v.number(),
  }),
});
