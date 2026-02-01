import Colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { api } from "@/convex/_generated/api";
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import { useMutation } from "convex/react";
import * as AppleAuthentication from "expo-apple-authentication";
import * as Location from "expo-location";
import { router } from "expo-router";
import { Bell, Check, MapPin } from "lucide-react-native";
import React, { useEffect, useState } from "react";
import {
    ActivityIndicator,
    Platform,
    ScrollView,
    Text,
    TouchableOpacity,
    View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

type OnboardingStep =
  | "welcome"
  | "location"
  | "notifications"
  | "complete";

export default function OnboardingScreen() {
  const insets = useSafeAreaInsets();
  const { loginWithApple, isAuthenticated, user } = useAuth();
  const completeOnboardingMutation = useMutation(api.users.completeOnboarding);

  const [step, setStep] = useState<OnboardingStep>("welcome");
  const [isLoading, setIsLoading] = useState(false);

  // Automatically advance to location step if authenticated but onboarding not done
  useEffect(() => {
    if (isAuthenticated && step === "welcome") {
       setStep("location");
    }
  }, [isAuthenticated, step]);

  const handleAppleLogin = async () => {
    setIsLoading(true);
    await loginWithApple();
    setIsLoading(false);
  };

  const handleNext = async () => {
    if (step === "location") {
      setIsLoading(true);
      try {
        const { status } = await Location.requestForegroundPermissionsAsync();
        console.log("Location permission status:", status);
        setStep("notifications");
      } catch (error) {
        console.error("Error requesting location permission:", error);
        setStep("notifications");
      } finally {
        setIsLoading(false);
      }
    } else if (step === "notifications") {
      setIsLoading(true);
      try {
        // Here we could request push token, but for now just mark as complete
        await completeOnboardingMutation();
        setStep("complete");
        setTimeout(() => {
          router.replace("/(tabs)");
        }, 2000);
      } catch (error) {
        console.error("Error completing onboarding:", error);
      } finally {
        setIsLoading(false);
      }
    }
  };

  const renderWelcome = () => (
    <View className="items-center py-10 w-full">
      <Text className="text-8xl mb-6">
      <MaterialCommunityIcons name="golf-cart" size={80} color={Colors.primary} />
      </Text>
      <Text className="text-3xl font-bold text-text text-center mb-3">Welcome to ACT</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-12">
        Safe, reliable rides around campus. Sign in to get started.
      </Text>

      {Platform.OS === "ios" && (
         <AppleAuthentication.AppleAuthenticationButton
           buttonType={AppleAuthentication.AppleAuthenticationButtonType.SIGN_IN}
           buttonStyle={AppleAuthentication.AppleAuthenticationButtonStyle.BLACK}
           cornerRadius={16}
           style={{ width: '100%', height: 50 }}
           onPress={handleAppleLogin}
         />
      )}
    </View>
  );

  const renderLocation = () => (
    <View className="items-center py-10">
      <View className="w-24 h-24 rounded-full bg-surface items-center justify-center mb-6">
        <MapPin size={40} color={Colors.primary} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">Enable location services</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">We need your location to:</Text>
      <View className="w-full gap-4">
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">Show nearby pickup points</Text>
        </View>
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">Track your ride in real-time</Text>
        </View>
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">
            Ensure safe and accurate pickups
          </Text>
        </View>
      </View>

       <TouchableOpacity
          className="mt-8 w-full flex-row items-center justify-center bg-primary rounded-2xl py-4.5 gap-2"
          onPress={handleNext}
          disabled={isLoading}
        >
            {isLoading ? <ActivityIndicator color="white" /> : <Text className="text-lg font-bold text-white">Continue</Text>}
        </TouchableOpacity>
    </View>
  );

  const renderNotifications = () => (
    <View className="items-center py-10">
      <View className="w-24 h-24 rounded-full bg-surface items-center justify-center mb-6">
        <Bell size={40} color={Colors.primary} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">Stay updated</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">Allow notifications to receive:</Text>
      <View className="w-full gap-4">
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">Driver arrival updates</Text>
        </View>
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">Ride status changes</Text>
        </View>
        <View className="flex-row items-center gap-3">
          <Check size={20} color={Colors.success} />
          <Text className="flex-1 text-base text-text">Important safety alerts</Text>
        </View>
      </View>

       <TouchableOpacity
          className="mt-8 w-full flex-row items-center justify-center bg-primary rounded-2xl py-4.5 gap-2"
          onPress={handleNext}
           disabled={isLoading}
        >
             {isLoading ? <ActivityIndicator color="white" /> : <Text className="text-lg font-bold text-white">Enable Notifications</Text>}
        </TouchableOpacity>
    </View>
  );

  const renderComplete = () => (
    <View className="items-center py-10">
      <View className="w-32 h-32 rounded-full bg-success items-center justify-center mb-6">
        <Check size={60} color={Colors.white} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">You&apos;re all set!</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        Welcome to ACT. Let&apos;s get you to your destination.
      </Text>
    </View>
  );

  const renderContent = () => {
    switch (step) {
      case "welcome":
        return renderWelcome();
      case "location":
        return renderLocation();
      case "notifications":
        return renderNotifications();
      case "complete":
        return renderComplete();
      default:
        return null;
    }
  };

  return (
    <View
      className="flex-1 bg-background px-5"
      style={{ paddingTop: insets.top + 16, paddingBottom: insets.bottom + 20 }}
    >
      <ScrollView
        contentContainerClassName="flex-grow justify-center"
        showsVerticalScrollIndicator={false}
      >
        {renderContent()}
      </ScrollView>
    </View>
  );
}
