import { AuthProvider, useAuth } from "@/contexts/AuthContext";
import { RideProvider } from "@/contexts/RideContext";
import { ConvexAuthProvider } from "@convex-dev/auth/react";
import {
  Inter_400Regular,
  Inter_700Bold,
  useFonts,
} from "@expo-google-fonts/inter";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { ConvexReactClient } from "convex/react";
import { Stack, useRouter, useSegments } from "expo-router";
import * as SecureStore from "expo-secure-store";
import * as SplashScreen from "expo-splash-screen";
import React, { useEffect } from "react";
import { GestureHandlerRootView } from "react-native-gesture-handler";
import "../global.css";

// Prevent the splash screen from auto-hiding before asset loading is complete.
SplashScreen.preventAutoHideAsync();

const queryClient = new QueryClient();
const convex = new ConvexReactClient(process.env.EXPO_PUBLIC_CONVEX_URL!, {
  unsavedChangesWarning: false,
});

const secureStorage = {
  getItem: SecureStore.getItemAsync,
  setItem: SecureStore.setItemAsync,
  removeItem: SecureStore.deleteItemAsync,
};

function RootLayoutNav() {
  return (
    <Stack screenOptions={{ headerBackTitle: "Back" }}>
      <Stack.Screen name="onboarding" options={{ headerShown: false }} />
      <Stack.Screen name="(tabs)" options={{ headerShown: false }} />
      <Stack.Screen
        name="plan-ride"
        options={{
          headerShown: false,
          presentation: "card",
          animation: "slide_from_bottom",
        }}
      />
    </Stack>
  );
}

function NavigationWrapper() {
  const { hasCompletedOnboarding, isLoading, isAuthenticated } = useAuth();
  const segments = useSegments();
  const router = useRouter();

  useEffect(() => {
    if (isLoading) return;

    const inOnboarding = segments[0] === "onboarding";

    // If authenticated but hasn't completed onboarding -> onboarding
    // If authenticated and completed onboarding -> tabs
    // If not authenticated -> onboarding (which has login)

    // Simplified:
    // If not authenticated, go to onboarding/login
    // If authenticated but no onboarding flag? (We might store onboarding in user profile now)

    if (!isAuthenticated) {
        if (!inOnboarding) router.replace("/onboarding");
    } else if (isAuthenticated && !hasCompletedOnboarding && !inOnboarding) {
        router.replace("/onboarding");
    } else if (isAuthenticated && hasCompletedOnboarding && inOnboarding) {
        router.replace("/(tabs)");
    }
  }, [hasCompletedOnboarding, isLoading, isAuthenticated, segments, router]);

  return <RootLayoutNav />;
}

export default function RootLayout() {
  const [loaded, error] = useFonts({
    Inter_400Regular,
    Inter_700Bold,
  });

  useEffect(() => {
    if (loaded || error) {
      SplashScreen.hideAsync();
    }
  }, [loaded, error]);

  if (!loaded && !error) {
    return null;
  }

  return (
    <ConvexAuthProvider client={convex} storage={secureStorage}>
      <QueryClientProvider client={queryClient}>
        <GestureHandlerRootView style={{ flex: 1 }}>
          <AuthProvider>
            <RideProvider>
              <NavigationWrapper />
            </RideProvider>
          </AuthProvider>
        </GestureHandlerRootView>
      </QueryClientProvider>
    </ConvexAuthProvider>
  );
}
