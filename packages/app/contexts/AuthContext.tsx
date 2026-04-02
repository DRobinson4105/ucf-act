import { api } from "@/convex/_generated/api";
import createContextHook from "@nkzw/create-context-hook";
import AsyncStorage from "@react-native-async-storage/async-storage";
import * as SecureStore from "expo-secure-store";
import { useConvexAuth, useMutation, useQuery } from "convex/react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";

const ONBOARDING_STORAGE_KEY = "act_onboarding_complete";
export const TOKEN_STORAGE_KEY = "act_apple_token";

// ---------------------------------------------------------------------------
// JWT expiry helper — reads the `exp` claim directly from the token payload.
// Apple identity tokens are valid for ~10 minutes; checking exp is more
// reliable than a time-based heuristic.
// ---------------------------------------------------------------------------
function isJwtExpired(token: string): boolean {
  try {
    const [, payloadB64] = token.split(".");
    // JWT uses base64url — convert to standard base64 before decoding
    const padded = payloadB64.replace(/-/g, "+").replace(/_/g, "/");
    const payload = JSON.parse(atob(padded));
    // Add 30 s buffer for clock skew; exp is in seconds
    return typeof payload.exp !== "number" || payload.exp * 1000 < Date.now() - 30_000;
  } catch {
    return true; // unparseable → assume expired
  }
}

// ---------------------------------------------------------------------------
// Module-level Apple token singleton — shared between ConvexProviderWithAuth
// and the rest of the app
// ---------------------------------------------------------------------------
let _appleToken: string | null = null;
const _tokenListeners = new Set<() => void>();

export function setAppleToken(token: string | null) {
  _appleToken = token;
  _tokenListeners.forEach((fn) => fn());
}

// Hook passed to ConvexProviderWithAuth in _layout.tsx
export function useAppleTokenAuth() {
  const [, rerender] = useState(0);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    SecureStore.getItemAsync(TOKEN_STORAGE_KEY).then((stored) => {
      if (stored) {
        if (!isJwtExpired(stored)) {
          _appleToken = stored;
        } else {
          // Silently discard an expired persisted token
          SecureStore.deleteItemAsync(TOKEN_STORAGE_KEY).catch(() => {});
        }
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
        // Convex calls forceRefresh when the server rejects the token.
        // If the JWT is genuinely expired, clear it and return null so the
        // user is redirected to sign-in. If it's still valid, return it —
        // the rejection was a timing race on first connect, not real expiry.
        if (!_appleToken || isJwtExpired(_appleToken)) {
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
  const storeUserMutation = useMutation(api.users.storeUser);
  const updateUserMutation = useMutation(api.users.updateUser);

  const [hasCompletedOnboarding, setHasCompletedOnboarding] = useState(false);
  const [onboardingLoaded, setOnboardingLoaded] = useState(false);
  const storeUserCalledRef = useRef(false);

  // Load onboarding completion flag
  useEffect(() => {
    AsyncStorage.getItem(ONBOARDING_STORAGE_KEY)
      .then((val) => setHasCompletedOnboarding(val === "true"))
      .catch(() => {})
      .finally(() => setOnboardingLoaded(true));
  }, []);

  // Reset storeUser gate when auth is lost
  useEffect(() => {
    if (!authLoading && !isAuthenticated) {
      storeUserCalledRef.current = false;
    }
  }, [authLoading, isAuthenticated]);

  // Create/update the Convex user record as soon as the SDK confirms auth.
  // useConvexAuth().isAuthenticated is only true AFTER the server has validated
  // the token — so no retry loop is needed here.
  useEffect(() => {
    if (!isAuthenticated || storeUserCalledRef.current) return;
    storeUserCalledRef.current = true;
    (async () => {
      const pendingName = await AsyncStorage.getItem("act_pending_name");
      try {
        await storeUserMutation({ name: pendingName || undefined });
        if (pendingName) await AsyncStorage.removeItem("act_pending_name");
      } catch (e) {
        storeUserCalledRef.current = false; // allow one retry on next render
        console.error("storeUser error:", e);
      }
    })();
  }, [isAuthenticated, storeUserMutation]);

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
