import { Password } from "@convex-dev/auth/providers/Password";
import { convexAuth } from "@convex-dev/auth/server";
import { v } from "convex/values";
import { mutation } from "./_generated/server";

export const { auth } = convexAuth({
  providers: [Password],
});

export const verifyCode = mutation({
  args: { phone: v.string(), code: v.string() },
  handler: async (ctx, args) => {
    // TODO: Implement actual code verification
    // For now, accept any code "123456"
    if (args.code === "123456") {
      // Create user if not exists?
      // For now just return a mock token
      return { token: "mock_token_123", userId: "mock_user_id" };
    }
    throw new Error("Invalid code");
  },
});
