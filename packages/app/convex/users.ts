import { mutation, query } from "./_generated/server";
import { auth } from "./auth";

export const currentUser = query({
  args: {},
  handler: async (ctx) => {
    const userId = await auth.getUserId(ctx);
    if (!userId) {
      return null;
    }
    return await ctx.db.get(userId);
  },
});

export const completeOnboarding = mutation({
  args: {},
  handler: async (ctx) => {
    const userId = await auth.getUserId(ctx);
    if (!userId) {
      throw new Error("Unauthenticated call to completeOnboarding");
    }
    await ctx.db.patch(userId, { hasCompletedOnboarding: true });
  },
});
