import Apple from "@auth/core/providers/apple";

export default {
  providers: [
    Apple({
      clientId: process.env.APPLE_CLIENT_ID,
      clientSecret: process.env.APPLE_CLIENT_SECRET,
    }),
  ],
};
