import { AuthProvider, useAppleTokenAuth, useAuth } from "@/contexts/AuthContext";
import { RideProvider } from "@/contexts/RideContext";
import { api } from "@/convex/_generated/api";
import {
  Inter_400Regular,
  Inter_700Bold,
  useFonts,
} from "@expo-google-fonts/inter";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { ConvexProviderWithAuth, ConvexReactClient, useMutation } from "convex/react";
import { Stack, useRouter, useSegments } from "expo-router";
import Constants from "expo-constants";
import * as Device from "expo-device";
import * as Notifications from "expo-notifications";
import * as SplashScreen from "expo-splash-screen";
import { GestureHandlerRootView } from "react-native-gesture-handler";
import React, { useEffect, useRef } from "react";
import "../global.css";

// Prevent the splash screen from auto-hiding before asset loading is complete.
SplashScreen.preventAutoHideAsync();

Notifications.setNotificationHandler({
  handleNotification: async () => ({
    shouldPlaySound: true,
    shouldSetBadge: false,
    shouldShowBanner: true,
    shouldShowList: true,
  }),
});

const queryClient = new QueryClient();
const convex = new ConvexReactClient(process.env.EXPO_PUBLIC_CONVEX_URL!, {
  unsavedChangesWarning: false,
});

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
      <Stack.Screen name="edit-profile" options={{ headerShown: false, animation: "slide_from_right" }} />
      <Stack.Screen name="settings" options={{ headerShown: false, animation: "slide_from_right" }} />
      <Stack.Screen name="terms" options={{ headerShown: false, animation: "slide_from_right" }} />
      <Stack.Screen name="payment" options={{ headerShown: false, animation: "slide_from_right" }} />
      <Stack.Screen name="help" options={{ headerShown: false, animation: "slide_from_right" }} />
    </Stack>
  );
}

function NavigationWrapper() {
  const { hasCompletedOnboarding, isAuthenticated, isLoading } = useAuth();
  const segments = useSegments();
  const router = useRouter();

  useEffect(() => {
    if (isLoading) return;

    const inOnboarding = segments[0] === "onboarding";

    if (!hasCompletedOnboarding && !inOnboarding) {
      // First-time user — full onboarding
      router.replace("/onboarding");
    } else if (hasCompletedOnboarding && !isAuthenticated && !inOnboarding) {
      // Returning user with expired token — sign-in only
      router.replace({ pathname: "/onboarding", params: { signInOnly: "true" } });
    } else if (hasCompletedOnboarding && isAuthenticated && inOnboarding) {
      // Authenticated — go to tabs
      router.replace("/(tabs)");
    }
  }, [hasCompletedOnboarding, isAuthenticated, isLoading, segments, router]);

  const updatePushToken = useMutation(api.users.updatePushToken);

  // Register push token
  useEffect(() => {
    if (!isAuthenticated) return;
    if (!Device.isDevice) {
      console.log("Push notifications require a physical device");
      return;
    }

    (async () => {
      try {
        const { status: existingStatus } = await Notifications.getPermissionsAsync();
        let finalStatus = existingStatus;
        if (existingStatus !== "granted") {
          const { status } = await Notifications.requestPermissionsAsync();
          finalStatus = status;
        }
        if (finalStatus !== "granted") {
          console.warn("Push notification permission not granted:", finalStatus);
          return;
        }

        const projectId =
          Constants.expoConfig?.extra?.eas?.projectId ??
          (Constants as any).easConfig?.projectId;
        if (!projectId) {
          console.error("Push notifications: EAS projectId not found in Constants");
          return;
        }

        const tokenData = await Notifications.getExpoPushTokenAsync({ projectId });
        console.log("Expo push token:", tokenData.data);
        await updatePushToken({ expoPushToken: tokenData.data });
        console.log("Push token stored successfully");
      } catch (e) {
        console.error("Push token registration failed:", e);
      }
    })();
  }, [isAuthenticated, updatePushToken]);

  // Handle incoming notifications & taps
  const notificationListener = useRef<Notifications.EventSubscription | undefined>(undefined);
  const notificationResponseListener = useRef<Notifications.EventSubscription | undefined>(undefined);
  useEffect(() => {
    // Log received notifications for debugging
    notificationListener.current =
      Notifications.addNotificationReceivedListener((notification) => {
        console.log("Push notification received:", notification.request.content.title);
      });

    notificationResponseListener.current =
      Notifications.addNotificationResponseReceivedListener((response) => {
        const data = response.notification.request.content.data;
        if (data?.type === "ride-update") {
          router.push("/plan-ride");
        }
      });

    return () => {
      notificationListener.current?.remove();
      notificationResponseListener.current?.remove();
    };
  }, [router]);

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
    <ConvexProviderWithAuth client={convex} useAuth={useAppleTokenAuth}>
      <QueryClientProvider client={queryClient}>
        <GestureHandlerRootView style={{ flex: 1 }}>
          <AuthProvider>
            <RideProvider>
              <NavigationWrapper />
            </RideProvider>
          </AuthProvider>
        </GestureHandlerRootView>
      </QueryClientProvider>
    </ConvexProviderWithAuth>
  );
}
