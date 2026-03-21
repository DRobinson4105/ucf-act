import Colors from "@/constants/colors";
import { setAppleToken, TOKEN_STORAGE_KEY, useAuth } from "@/contexts/AuthContext";
import MaterialCommunityIcons from "@expo/vector-icons/MaterialCommunityIcons";
import AsyncStorage from "@react-native-async-storage/async-storage";
import * as AppleAuthentication from "expo-apple-authentication";
import * as SecureStore from "expo-secure-store";
import * as Location from "expo-location";
import { router } from "expo-router";
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

      setStep("location");
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
    <View className="flex-row items-center gap-4">
      <View className="flex-1 h-1 bg-surface rounded-sm overflow-hidden">
        <View className="h-full bg-primary" style={{ width: `${progress * 100}%` }} />
      </View>
      <Text className="text-sm font-semibold text-textSecondary">
        {currentStepNumber} of {totalSteps}
      </Text>
    </View>
  );

  const renderWelcome = () => (
    <View className="items-center py-10">
      <MaterialCommunityIcons name="golf-cart" size={80} color={Colors.primary} style={{ marginBottom: 24 }} />
      <Text className="text-3xl font-bold text-text text-center mb-3">Welcome to ACT</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-10">
        Safe, autonomous rides around campus. Sign in with Apple to get started.
      </Text>

      {Platform.OS === "ios" ? (
        <AppleAuthentication.AppleAuthenticationButton
          buttonType={AppleAuthentication.AppleAuthenticationButtonType.SIGN_IN}
          buttonStyle={AppleAuthentication.AppleAuthenticationButtonStyle.WHITE_OUTLINE}
          cornerRadius={16}
          style={{ width: "100%", height: 56 }}
          onPress={handleAppleSignIn}
        />
      ) : (
        <TouchableOpacity
          className="w-full bg-surface rounded-2xl py-4 items-center border border-border"
          onPress={handleAppleSignIn}
          disabled={isLoading}
        >
          <Text className="text-base font-semibold text-text">Sign in with Apple</Text>
        </TouchableOpacity>
      )}

      {isLoading && <ActivityIndicator color={Colors.primary} style={{ marginTop: 16 }} />}
      {authError && (
        <Text className="text-sm text-error text-center mt-4">{authError}</Text>
      )}
    </View>
  );

  const renderLocation = () => (
    <View className="items-center py-10">
      <View className="w-24 h-24 rounded-full bg-surface items-center justify-center mb-6">
        <MaterialCommunityIcons name="map-marker" size={40} color={Colors.primary} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">Enable location services</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">We need your location to:</Text>
      <View className="w-full gap-4">
        {[
          "Show nearby pickup points",
          "Track your ride in real-time",
          "Ensure safe and accurate pickups",
        ].map((item) => (
          <View key={item} className="flex-row items-center gap-3">
            <Check size={20} color={Colors.success} />
            <Text className="flex-1 text-base text-text">{item}</Text>
          </View>
        ))}
      </View>
    </View>
  );

  const renderNotifications = () => (
    <View className="items-center py-10">
      <View className="w-24 h-24 rounded-full bg-surface items-center justify-center mb-6">
        <Bell size={40} color={Colors.primary} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">Stay updated</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        Allow notifications to receive:
      </Text>
      <View className="w-full gap-4">
        {[
          "Cart arrival updates",
          "Ride status changes",
          "Important safety alerts",
        ].map((item) => (
          <View key={item} className="flex-row items-center gap-3">
            <Check size={20} color={Colors.success} />
            <Text className="flex-1 text-base text-text">{item}</Text>
          </View>
        ))}
      </View>
    </View>
  );

  const renderComplete = () => (
    <View className="items-center py-10">
      <View className="w-32 h-32 rounded-full bg-success items-center justify-center mb-6">
        <Check size={60} color={Colors.white} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">You&apos;re all set!</Text>
      <Text className="text-base text-textSecondary text-center leading-6">
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
    <View className="flex-1 bg-background">
      <View className="px-5 pb-4" style={{ paddingTop: insets.top + 16 }}>
        {currentStepNumber > 0 && step !== "complete" && renderProgressBar()}
      </View>

      <ScrollView
        contentContainerClassName="flex-grow justify-center px-5"
        showsVerticalScrollIndicator={false}
        keyboardShouldPersistTaps="handled"
      >
        {renderContent()}
      </ScrollView>

      {showContinueButton && (
        <View className="px-5 pt-5 border-t border-border" style={{ paddingBottom: insets.bottom + 20 }}>
          <TouchableOpacity
            className="flex-row items-center justify-center bg-primary rounded-2xl py-4.5 gap-2"
            onPress={handleContinue}
            disabled={isLoading}
            activeOpacity={0.7}
          >
            {isLoading ? (
              <ActivityIndicator color={Colors.white} />
            ) : (
              <Text className="text-lg font-bold text-white">Continue</Text>
            )}
          </TouchableOpacity>
        </View>
      )}
    </View>
  );
}
