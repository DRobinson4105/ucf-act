export default {
  providers: [
    {
      // Apple OIDC — Convex verifies the identityToken JWT using Apple's public keys
      domain: "https://appleid.apple.com",
      applicationID: "com.ucf.act",
    },
  ],
};
