import Colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import * as Location from "expo-location";
import { router } from "expo-router";
import { Bell, Check, ChevronRight, MapPin } from "lucide-react-native";
import React, { useState } from "react";
import {
  ActivityIndicator,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

type OnboardingStep =
  | "welcome"
  | "name"
  | "phone"
  | "verify"
  | "location"
  | "notifications"
  | "complete";

export default function OnboardingScreen() {
  const insets = useSafeAreaInsets();
  const { completeOnboarding } = useAuth();

  const [step, setStep] = useState<OnboardingStep>("welcome");
  const [name, setName] = useState("");
  const [phoneNumber, setPhoneNumber] = useState("");
  const [verificationCode, setVerificationCode] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const totalSteps = 6;
  const currentStepNumber = {
    welcome: 0,
    name: 1,
    phone: 2,
    verify: 3,
    location: 4,
    notifications: 5,
    complete: 6,
  }[step];

  const progress = currentStepNumber / totalSteps;

  const handleNext = async () => {
    if (step === "welcome") {
      setStep("name");
    } else if (step === "name") {
      if (name.trim()) {
        setStep("phone");
      }
    } else if (step === "phone") {
      if (phoneNumber.length >= 10) {
        setIsLoading(true);
        setTimeout(() => {
          setIsLoading(false);
          setStep("verify");
        }, 1000);
      }
    } else if (step === "verify") {
      if (verificationCode.length === 6) {
        setIsLoading(true);
        setTimeout(() => {
          setIsLoading(false);
          setStep("location");
        }, 1000);
      }
    } else if (step === "location") {
      setIsLoading(true);
      try {
        const { status } = await Location.requestForegroundPermissionsAsync();
        console.log("Location permission status:", status);
        setIsLoading(false);
        setStep("notifications");
      } catch (error) {
        console.error("Error requesting location permission:", error);
        setIsLoading(false);
        setStep("notifications");
      }
    } else if (step === "notifications") {
      setIsLoading(true);
      try {
        await completeOnboarding({
          name,
          phoneNumber,
        });
        setStep("complete");
        setTimeout(() => {
          setIsLoading(false);
          router.replace("/(tabs)");
        }, 2000);
      } catch (error) {
        console.error("Error completing onboarding:", error);
        setIsLoading(false);
      }
    }
  };

  const canProceed = () => {
    if (step === "name") return name.trim().length > 0;
    if (step === "phone") return phoneNumber.length >= 10;
    if (step === "verify") return verificationCode.length === 6;
    return true;
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
      <Text className="text-8xl mb-6">
      <MaterialCommunityIcons name="golf-cart" size={80} color={Colors.primary} />
      </Text>
      <Text className="text-3xl font-bold text-text text-center mb-3">Welcome to ACT</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        Safe, reliable rides around campus. Let&apos;s get you set up in just a few steps.
      </Text>
    </View>
  );

  const renderName = () => (
    <View className="items-center py-10">
      <Text className="text-3xl font-bold text-text text-center mb-3">What&apos;s your name?</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        This helps drivers identify you during pickup
      </Text>
      <TextInput
        className="w-full bg-surface rounded-2xl px-5 py-4.5 text-lg text-text border border-border"
        value={name}
        onChangeText={setName}
        placeholder="Enter your full name"
        placeholderTextColor={Colors.textSecondary}
        autoFocus
        autoCapitalize="words"
      />
    </View>
  );

  const renderPhone = () => (
    <View className="items-center py-10">
      <Text className="text-3xl font-bold text-text text-center mb-3">Enter your phone number</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        We&apos;ll send you a verification code to confirm your number
      </Text>
      <View className="w-full flex-row items-center bg-surface rounded-2xl px-5 py-4.5 border border-border gap-3">
        <Text className="text-lg font-semibold text-text">+1</Text>
        <TextInput
          className="flex-1 text-lg text-text"
          value={phoneNumber}
          onChangeText={setPhoneNumber}
          placeholder="(555) 123-4567"
          placeholderTextColor={Colors.textSecondary}
          keyboardType="phone-pad"
          maxLength={14}
          autoFocus
        />
      </View>
    </View>
  );

  const renderVerify = () => (
    <View className="items-center py-10">
      <Text className="text-3xl font-bold text-text text-center mb-3">Enter verification code</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        We sent a 6-digit code to {phoneNumber}
      </Text>
      <TextInput
        className="w-full bg-surface rounded-2xl px-5 py-4.5 text-lg text-text border border-border text-center font-semibold tracking-widest text-4xl"
        value={verificationCode}
        onChangeText={setVerificationCode}
        placeholder="000000"
        placeholderTextColor={Colors.textSecondary}
        keyboardType="number-pad"
        maxLength={6}
        autoFocus
      />
      <TouchableOpacity className="mt-4">
        <Text className="text-base font-semibold text-primary">Resend code</Text>
      </TouchableOpacity>
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
    </View>
  );

  const renderComplete = () => (
    <View className="items-center py-10">
      <View className="w-32 h-32 rounded-full bg-success items-center justify-center mb-6">
        <Check size={60} color={Colors.white} />
      </View>
      <Text className="text-3xl font-bold text-text text-center mb-3">You&apos;re all set!</Text>
      <Text className="text-base text-textSecondary text-center leading-6 mb-8">
        Welcome to CampusRide. Let&apos;s get you to your destination.
      </Text>
    </View>
  );

  const renderContent = () => {
    switch (step) {
      case "welcome":
        return renderWelcome();
      case "name":
        return renderName();
      case "phone":
        return renderPhone();
      case "verify":
        return renderVerify();
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
    <KeyboardAvoidingView
      className="flex-1 bg-background"
      behavior={Platform.OS === "ios" ? "padding" : "height"}
    >
      <View className="px-5 pb-4" style={{ paddingTop: insets.top + 16 }}>
        {currentStepNumber > 0 && renderProgressBar()}
      </View>

      <ScrollView
        contentContainerClassName="flex-grow justify-center px-5"
        showsVerticalScrollIndicator={false}
        keyboardShouldPersistTaps="handled"
      >
        {renderContent()}
      </ScrollView>

      {step !== "complete" && (
        <View className="px-5 pt-5 border-t border-border" style={{ paddingBottom: insets.bottom + 20 }}>
          <TouchableOpacity
            className={`flex-row items-center justify-center bg-primary rounded-2xl py-4.5 gap-2 ${
              (!canProceed() || isLoading) ? "bg-surface" : ""
            }`}
            onPress={handleNext}
            disabled={!canProceed() || isLoading}
            activeOpacity={0.7}
          >
            {isLoading ? (
              <ActivityIndicator color={Colors.white} />
            ) : (
              <>
                <Text className="text-lg font-bold text-white">
                  {step === "welcome" ? "Get Started" : "Continue"}
                </Text>
                <ChevronRight size={24} color={Colors.white} />
              </>
            )}
          </TouchableOpacity>
        </View>
      )}
    </KeyboardAvoidingView>
  );
}
