import { api } from "@/convex/_generated/api";
import createContextHook from "@nkzw/create-context-hook";
import AsyncStorage from "@react-native-async-storage/async-storage";
import * as SecureStore from "expo-secure-store";
import { useConvexAuth, useMutation, useQuery } from "convex/react";
import { useCallback, useEffect, useMemo, useState } from "react";

const ONBOARDING_STORAGE_KEY = "act_onboarding_complete";
export const TOKEN_STORAGE_KEY = "act_apple_token";

// ---------------------------------------------------------------------------
// Module-level Apple token singleton
// Shared between ConvexProviderWithAuth (in _layout.tsx) and the app
// ---------------------------------------------------------------------------
let _appleToken: string | null = null;
let _appleTokenIssuedAt: number | null = null; // ms since epoch
const _tokenListeners = new Set<() => void>();

export function setAppleToken(token: string | null) {
  _appleToken = token;
  _appleTokenIssuedAt = token ? Date.now() : null;
  _tokenListeners.forEach((fn) => fn());
}

// Hook passed to ConvexProviderWithAuth in _layout.tsx
export function useAppleTokenAuth() {
  const [, rerender] = useState(0);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    SecureStore.getItemAsync(TOKEN_STORAGE_KEY).then((stored) => {
      if (stored) {
        _appleToken = stored;
        // We don't know how old the stored token is; treat it as 8 min old so
        // forceRefresh calls still return it (giving a ~1 min grace window before
        // we'd clear a genuinely expired token).
        _appleTokenIssuedAt = Date.now() - 8 * 60 * 1000;
      }
      setIsLoading(false);
      rerender((n) => n + 1);
    });

    const listener = () => rerender((n) => n + 1);
    _tokenListeners.add(listener);
    return () => {
      _tokenListeners.delete(listener);
    };
  }, []);

  const fetchAccessToken = useCallback(
    async ({ forceRefreshToken }: { forceRefreshToken: boolean }) => {
      if (forceRefreshToken) {
        // Apple identity tokens can't be refreshed silently. Only clear the
        // token if it's genuinely expired (>9 min old). If it was issued
        // recently, Convex called forceRefresh due to a timing race on the
        // first mutation — not actual expiry — so return the token to re-auth.
        const ageMs = _appleTokenIssuedAt ? Date.now() - _appleTokenIssuedAt : Infinity;
        if (ageMs > 9 * 60 * 1000) {
          await SecureStore.deleteItemAsync(TOKEN_STORAGE_KEY);
          setAppleToken(null);
          return null;
        }
        return _appleToken;
      }
      return _appleToken;
    },
    []
  );

  return {
    isLoading,
    isAuthenticated: _appleToken !== null,
    fetchAccessToken,
  };
}

// ---------------------------------------------------------------------------
// AuthContext — app-facing auth state
// ---------------------------------------------------------------------------
interface User {
  id: string;
  name?: string;
  email?: string;
  phone?: string;
}

export const [AuthProvider, useAuth] = createContextHook(() => {
  const { isAuthenticated, isLoading: authLoading } = useConvexAuth();
  const convexUser = useQuery(api.users.currentUser);
  const updateUserMutation = useMutation(api.users.updateUser);

  const [hasCompletedOnboarding, setHasCompletedOnboarding] = useState(false);
  const [onboardingLoaded, setOnboardingLoaded] = useState(false);

  // Load onboarding completion flag
  useEffect(() => {
    AsyncStorage.getItem(ONBOARDING_STORAGE_KEY)
      .then((val) => setHasCompletedOnboarding(val === "true"))
      .catch(() => {})
      .finally(() => setOnboardingLoaded(true));
  }, []);

  // Reset onboarding if auth is lost (e.g. token expired)
  useEffect(() => {
    if (!authLoading && !isAuthenticated && hasCompletedOnboarding) {
      setHasCompletedOnboarding(false);
      AsyncStorage.removeItem(ONBOARDING_STORAGE_KEY).catch(() => {});
    }
  }, [authLoading, isAuthenticated, hasCompletedOnboarding]);

  const isLoading =
    authLoading ||
    !onboardingLoaded ||
    (isAuthenticated && convexUser === undefined);

  const user: User | null = convexUser
    ? {
        id: convexUser._id,
        name: convexUser.name,
        email: convexUser.email,
        phone: convexUser.phone,
      }
    : null;

  const updateUser = useCallback(
    async (updates: Partial<User>) => {
      await updateUserMutation({ name: updates.name, phone: updates.phone });
    },
    [updateUserMutation]
  );

  const completeOnboarding = useCallback(async () => {
    await AsyncStorage.setItem(ONBOARDING_STORAGE_KEY, "true");
    setHasCompletedOnboarding(true);
  }, []);

  const logout = useCallback(async () => {
    await SecureStore.deleteItemAsync(TOKEN_STORAGE_KEY);
    setAppleToken(null);
    await AsyncStorage.removeItem(ONBOARDING_STORAGE_KEY);
    setHasCompletedOnboarding(false);
  }, []);

  return useMemo(
    () => ({
      user,
      isLoading,
      isAuthenticated,
      hasCompletedOnboarding,
      updateUser,
      completeOnboarding,
      logout,
    }),
    [
      user,
      isLoading,
      isAuthenticated,
      hasCompletedOnboarding,
      updateUser,
      completeOnboarding,
      logout,
    ]
  );
});
