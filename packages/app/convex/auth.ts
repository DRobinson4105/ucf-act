import { v } from "convex/values";
import { mutation } from "./_generated/server";

export const sendVerificationCode = mutation({
  args: { phone: v.string() },
  handler: async (ctx, args) => {
    // TODO: Implement actual SMS sending (Twilio, etc.)
    // For now, we'll just log it or return a mock success
    console.log(`Sending code to ${args.phone}`);
    return { success: true, message: "Code sent" };
  },
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
