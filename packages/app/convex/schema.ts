import { authTables } from "@convex-dev/auth/server";
import { defineSchema, defineTable } from "convex/server";
import { v } from "convex/values";

export default defineSchema({
  ...authTables,
  users: defineTable({
    name: v.optional(v.string()),
    image: v.optional(v.string()),
    email: v.optional(v.string()),
    emailVerificationTime: v.optional(v.number()),
    phone: v.optional(v.string()),
    campusId: v.optional(v.string()),
    hasCompletedOnboarding: v.optional(v.boolean()),
    settings: v.optional(
      v.object({
        notificationsEnabled: v.boolean(),
      })
    ),
  }).index("email", ["email"]),

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

  notifications: defineTable({
    userId: v.id("users"),
    title: v.string(),
    message: v.string(),
    type: v.union(v.literal("ride_update"), v.literal("promo"), v.literal("system")),
    read: v.boolean(),
    timestamp: v.number(),
    data: v.optional(v.any()), // For extra payload like rideId
  })
    .index("by_user", ["userId"])
    .index("by_user_read", ["userId", "read"]),
});
