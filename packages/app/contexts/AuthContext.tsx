import createContextHook from "@nkzw/create-context-hook";
import AsyncStorage from "@react-native-async-storage/async-storage";
import { useCallback, useEffect, useMemo, useState } from "react";

interface User {
  id: string;
  name: string;
  email?: string;
  phoneNumber?: string;
  campusId?: string;
  hasCompletedOnboarding?: boolean;
}

const USER_STORAGE_KEY = "act_user";
const ONBOARDING_STORAGE_KEY = "act_onboarding_complete";

export const [AuthProvider, useAuth] = createContextHook(() => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [hasCompletedOnboarding, setHasCompletedOnboarding] = useState(false);

  useEffect(() => {
    loadUser();
  }, []);

  const loadUser = async () => {
    try {
      const [storedUser, onboardingComplete] = await Promise.all([
        AsyncStorage.getItem(USER_STORAGE_KEY),
        AsyncStorage.getItem(ONBOARDING_STORAGE_KEY),
      ]);

      if (storedUser) {
        setUser(JSON.parse(storedUser));
      }

      setHasCompletedOnboarding(onboardingComplete === "true");
    } catch (error) {
      console.error("Error loading user:", error);
    } finally {
      setIsLoading(false);
    }
  };

  const updateUser = useCallback(
    async (updates: Partial<User>) => {
      if (!user) return;

      const updatedUser = { ...user, ...updates };
      setUser(updatedUser);

      try {
        await AsyncStorage.setItem(
          USER_STORAGE_KEY,
          JSON.stringify(updatedUser)
        );
      } catch (error) {
        console.error("Error updating user:", error);
      }
    },
    [user]
  );

  const completeOnboarding = useCallback(async (userData: Omit<User, "id">) => {
    try {
      const newUser: User = {
        id: Date.now().toString(),
        ...userData,
        hasCompletedOnboarding: true,
      };

      await Promise.all([
        AsyncStorage.setItem(USER_STORAGE_KEY, JSON.stringify(newUser)),
        AsyncStorage.setItem(ONBOARDING_STORAGE_KEY, "true"),
      ]);

      setUser(newUser);
      setHasCompletedOnboarding(true);
    } catch (error) {
      console.error("Error completing onboarding:", error);
      throw error;
    }
  }, []);

  const logout = useCallback(async () => {
    try {
      await AsyncStorage.removeItem(USER_STORAGE_KEY);
      setUser(null);
    } catch (error) {
      console.error("Error logging out:", error);
    }
  }, []);

  return useMemo(
    () => ({
      user,
      isLoading,
      hasCompletedOnboarding,
      updateUser,
      completeOnboarding,
      logout,
    }),
    [
      user,
      isLoading,
      hasCompletedOnboarding,
      updateUser,
      completeOnboarding,
      logout,
    ]
  );
});
