import { api } from "@/convex/_generated/api";
import { useAuthActions } from "@convex-dev/auth/react";
import createContextHook from "@nkzw/create-context-hook";
import { useConvexAuth, useQuery } from "convex/react";
import * as AppleAuthentication from "expo-apple-authentication";
import { useMemo } from "react";

interface User {
  _id: string;
  name?: string;
  email?: string;
  phone?: string;
  hasCompletedOnboarding?: boolean;
}

export const [AuthProvider, useAuth] = createContextHook(() => {
  const { isAuthenticated, isLoading: isAuthLoading } = useConvexAuth();
  const { signIn, signOut } = useAuthActions();

  // Fetch the current user profile from Convex
  // This will be null if not authenticated or user setup not complete
  const user = useQuery(api.users.currentUser);

  const loginWithApple = async () => {
    try {
      const credential = await AppleAuthentication.signInAsync({
        requestedScopes: [
          AppleAuthentication.AppleAuthenticationScope.FULL_NAME,
          AppleAuthentication.AppleAuthenticationScope.EMAIL,
        ],
      });

      if (credential.identityToken) {
        await signIn("apple", {
             idToken: credential.identityToken,
             // Optional: pass name/email if first time, but Apple usually provides it in token or separate fields
             // name: credential.fullName?.givenName,
        });
      }
    } catch (e: any) {
      if (e.code === "ERR_CANCELED") {
        // handle that the user canceled the sign-in flow
      } else {
        // handle other errors
        console.error("Apple Sign In Error:", e);
      }
    }
  };

  const logout = async () => {
    await signOut();
  };

  const hasCompletedOnboarding = !!(user?.name); // Example: if name is set, onboarding done?
  // Or we can rely on a specific field in the user object
  // For now, let's assume if we have a user object, we are partially onboarded,
  // but we might want a specific flag.
  // The original context had `hasCompletedOnboarding`.

  return useMemo(
    () => ({
      user,
      isLoading: isAuthLoading || (isAuthenticated && user === undefined),
      isAuthenticated,
      hasCompletedOnboarding: !!user, // Simplified for now
      loginWithApple,
      logout,
    }),
    [user, isAuthLoading, isAuthenticated, loginWithApple, logout]
  );
});
