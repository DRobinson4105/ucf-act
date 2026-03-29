import Colors from "@/constants/colors";
import { setAppleToken, TOKEN_STORAGE_KEY, useAuth } from "@/contexts/AuthContext";
import MaterialCommunityIcons from "@expo/vector-icons/MaterialCommunityIcons";
import AsyncStorage from "@react-native-async-storage/async-storage";
import * as AppleAuthentication from "expo-apple-authentication";
import * as SecureStore from "expo-secure-store";
import * as Location from "expo-location";
import * as Notifications from "expo-notifications";
import { router, useLocalSearchParams } from "expo-router";
import { Bell, Check } from "lucide-react-native";
import React, { useState } from "react";
import {
  ActivityIndicator,
  Platform,
  ScrollView,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

type OnboardingStep = "welcome" | "location" | "notifications" | "complete";

export default function OnboardingScreen() {
  const insets = useSafeAreaInsets();
  const { completeOnboarding } = useAuth();

  const params = useLocalSearchParams<{ signInOnly?: string }>();
  const signInOnly = params.signInOnly === "true";

  const [step, setStep] = useState<OnboardingStep>("welcome");
  const [isLoading, setIsLoading] = useState(false);
  const [authError, setAuthError] = useState<string | null>(null);

  const totalSteps = 3;
  const currentStepNumber = { welcome: 0, location: 1, notifications: 2, complete: 3 }[step];
  const progress = currentStepNumber / totalSteps;

  const handleAppleSignIn = async () => {
    setIsLoading(true);
    setAuthError(null);
    try {
      const credential = await AppleAuthentication.signInAsync({
        requestedScopes: [
          AppleAuthentication.AppleAuthenticationScope.FULL_NAME,
          AppleAuthentication.AppleAuthenticationScope.EMAIL,
        ],
      });

      if (!credential.identityToken) {
        throw new Error("No identity token from Apple");
      }

      // Build name from Apple credential (only sent on first sign-in)
      let name: string | undefined;
      if (credential.fullName) {
        const parts = [
          credential.fullName.givenName,
          credential.fullName.familyName,
        ].filter(Boolean);
        if (parts.length > 0) name = parts.join(" ");
      }

      // Persist the name so AuthContext can pass it to storeUser once auth is confirmed
      if (name) await AsyncStorage.setItem("act_pending_name", name);
      // Set token — ConvexProviderWithAuth picks this up and completes the
      // WebSocket auth handshake; AuthContext's useEffect then calls storeUser
      // once useConvexAuth().isAuthenticated becomes true (server-confirmed).
      await SecureStore.setItemAsync(TOKEN_STORAGE_KEY, credential.identityToken);
      setAppleToken(credential.identityToken);

      if (signInOnly) {
        router.replace("/(tabs)");
      } else {
        setStep("location");
      }
    } catch (err: unknown) {
      const e = err as { code?: string; message?: string };
      // User cancelled — don't show error
      if (e?.code === "ERR_REQUEST_CANCELED") {
        return;
      }
      console.error("Apple sign-in error:", err);
      setAuthError(e?.message || "Sign in failed. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  const handleLocation = async () => {
    setIsLoading(true);
    try {
      await Location.requestForegroundPermissionsAsync();
    } catch {
      // Permission denied is fine — proceed anyway
    } finally {
      setIsLoading(false);
      setStep("notifications");
    }
  };

  const handleNotifications = async () => {
    setIsLoading(true);
    try {
      const { status } = await Notifications.requestPermissionsAsync();
      if (status === "granted") {
        console.log("Notification permissions granted");
      }
      await completeOnboarding();
      setStep("complete");
      setTimeout(() => router.replace("/(tabs)"), 1500);
    } catch (err) {
      console.error("Error completing onboarding:", err);
    } finally {
      setIsLoading(false);
    }
  };

  const renderProgressBar = () => (
    <View style={{ flexDirection: "row", alignItems: "center", gap: 16 }}>
      <View style={{ flex: 1, height: 4, backgroundColor: Colors.surface, borderRadius: 2, overflow: "hidden" }}>
        <View style={{ height: "100%", backgroundColor: Colors.primary, width: `${progress * 100}%` }} />
      </View>
      <Text style={{ fontSize: 14, fontWeight: "600", color: Colors.textSecondary }}>
        {currentStepNumber} of {totalSteps}
      </Text>
    </View>
  );

  const renderWelcome = () => (
    <View style={{ alignItems: "center", paddingVertical: 40 }}>
      <MaterialCommunityIcons name="golf-cart" size={80} color={Colors.primary} style={{ marginBottom: 24 }} />
      <Text style={{ fontSize: 30, fontWeight: "700", color: Colors.text, textAlign: "center", marginBottom: 12 }}>Welcome to ACT</Text>
      <Text style={{ fontSize: 16, color: Colors.textSecondary, textAlign: "center", lineHeight: 24, marginBottom: 40 }}>
        Safe, autonomous rides around campus. Sign in with Apple to get started.
      </Text>

      {Platform.OS === "ios" ? (
        <AppleAuthentication.AppleAuthenticationButton
          buttonType={AppleAuthentication.AppleAuthenticationButtonType.SIGN_IN}
          buttonStyle={AppleAuthentication.AppleAuthenticationButtonStyle.WHITE_OUTLINE}
          cornerRadius={14}
          style={{ width: "100%", height: 52 }}
          onPress={handleAppleSignIn}
        />
      ) : (
        <TouchableOpacity
          style={{
            width: "100%",
            backgroundColor: Colors.accent,
            borderRadius: 14,
            paddingVertical: 16,
            alignItems: "center",
          }}
          onPress={handleAppleSignIn}
          disabled={isLoading}
        >
          <Text style={{ fontSize: 16, fontWeight: "700", color: Colors.black }}>
            Sign in with Apple
          </Text>
        </TouchableOpacity>
      )}

      {isLoading && <ActivityIndicator color={Colors.primary} style={{ marginTop: 16 }} />}
      {authError && (
        <Text style={{ fontSize: 14, color: Colors.error, textAlign: "center", marginTop: 16 }}>{authError}</Text>
      )}
    </View>
  );

  const renderLocation = () => (
    <View style={{ alignItems: "center", paddingVertical: 40 }}>
      <View style={{ width: 96, height: 96, borderRadius: 48, backgroundColor: Colors.surface, alignItems: "center", justifyContent: "center", marginBottom: 24 }}>
        <MaterialCommunityIcons name="map-marker" size={40} color={Colors.primary} />
      </View>
      <Text style={{ fontSize: 30, fontWeight: "700", color: Colors.text, textAlign: "center", marginBottom: 12 }}>Enable location services</Text>
      <Text style={{ fontSize: 16, color: Colors.textSecondary, textAlign: "center", lineHeight: 24, marginBottom: 32 }}>We need your location to:</Text>
      <View style={{ width: "100%", gap: 16 }}>
        {[
          "Show nearby pickup points",
          "Track your ride in real-time",
          "Ensure safe and accurate pickups",
        ].map((item) => (
          <View key={item} style={{ flexDirection: "row", alignItems: "center", gap: 12 }}>
            <Check size={20} color={Colors.success} />
            <Text style={{ flex: 1, fontSize: 16, color: Colors.text }}>{item}</Text>
          </View>
        ))}
      </View>
    </View>
  );

  const renderNotifications = () => (
    <View style={{ alignItems: "center", paddingVertical: 40 }}>
      <View style={{ width: 96, height: 96, borderRadius: 48, backgroundColor: Colors.surface, alignItems: "center", justifyContent: "center", marginBottom: 24 }}>
        <Bell size={40} color={Colors.primary} />
      </View>
      <Text style={{ fontSize: 30, fontWeight: "700", color: Colors.text, textAlign: "center", marginBottom: 12 }}>Stay updated</Text>
      <Text style={{ fontSize: 16, color: Colors.textSecondary, textAlign: "center", lineHeight: 24, marginBottom: 32 }}>
        Allow notifications to receive:
      </Text>
      <View style={{ width: "100%", gap: 16 }}>
        {[
          "Cart arrival updates",
          "Ride status changes",
          "Important safety alerts",
        ].map((item) => (
          <View key={item} style={{ flexDirection: "row", alignItems: "center", gap: 12 }}>
            <Check size={20} color={Colors.success} />
            <Text style={{ flex: 1, fontSize: 16, color: Colors.text }}>{item}</Text>
          </View>
        ))}
      </View>
    </View>
  );

  const renderComplete = () => (
    <View style={{ alignItems: "center", paddingVertical: 40 }}>
      <View style={{ width: 128, height: 128, borderRadius: 64, backgroundColor: Colors.success, alignItems: "center", justifyContent: "center", marginBottom: 24 }}>
        <Check size={60} color={Colors.white} />
      </View>
      <Text style={{ fontSize: 30, fontWeight: "700", color: Colors.text, textAlign: "center", marginBottom: 12 }}>You&apos;re all set!</Text>
      <Text style={{ fontSize: 16, color: Colors.textSecondary, textAlign: "center", lineHeight: 24 }}>
        Welcome to ACT. Let&apos;s get you to your destination.
      </Text>
    </View>
  );

  const renderContent = () => {
    switch (step) {
      case "welcome": return renderWelcome();
      case "location": return renderLocation();
      case "notifications": return renderNotifications();
      case "complete": return renderComplete();
    }
  };

  const handleContinue = () => {
    if (step === "location") handleLocation();
    else if (step === "notifications") handleNotifications();
  };

  const showContinueButton = step === "location" || step === "notifications";

  return (
    <View style={{ flex: 1, backgroundColor: Colors.background }}>
      <View style={{ paddingHorizontal: 20, paddingBottom: 16, paddingTop: insets.top + 16 }}>
        {!signInOnly && currentStepNumber > 0 && step !== "complete" && renderProgressBar()}
      </View>

      <ScrollView
        contentContainerStyle={{ flexGrow: 1, justifyContent: "center", paddingHorizontal: 20 }}
        showsVerticalScrollIndicator={false}
        keyboardShouldPersistTaps="handled"
      >
        {renderContent()}
      </ScrollView>

      {showContinueButton && (
        <View style={{ paddingHorizontal: 20, paddingTop: 20, borderTopWidth: 1, borderTopColor: Colors.border, paddingBottom: insets.bottom + 20 }}>
          <TouchableOpacity
            style={{
              flexDirection: "row",
              alignItems: "center",
              justifyContent: "center",
              backgroundColor: Colors.accent,
              borderRadius: 14,
              paddingVertical: 16,
              gap: 8,
            }}
            onPress={handleContinue}
            disabled={isLoading}
            activeOpacity={0.7}
          >
            {isLoading ? (
              <ActivityIndicator color={Colors.black} />
            ) : (
              <Text style={{ fontSize: 16, fontWeight: "700", color: Colors.black }}>
                Continue
              </Text>
            )}
          </TouchableOpacity>
        </View>
      )}
    </View>
  );
}
