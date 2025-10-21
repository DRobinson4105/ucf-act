import createContextHook from "@nkzw/create-context-hook";
import AsyncStorage from "@react-native-async-storage/async-storage";
import { useCallback, useEffect, useMemo, useState } from "react";

interface User {
  id: string;
  name: string;
  email: string;
  campusId?: string;
}

const USER_STORAGE_KEY = "act_user";

export const [AuthProvider, useAuth] = createContextHook(() => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    loadUser();
  }, []);

  const loadUser = async () => {
    try {
      const stored = await AsyncStorage.getItem(USER_STORAGE_KEY);
      if (stored) {
        setUser(JSON.parse(stored));
      } else {
        const mockUser: User = {
          id: "1",
          name: "Campus User",
          email: "user@campus.edu",
          campusId: "STU123456",
        };
        setUser(mockUser);
        await AsyncStorage.setItem(USER_STORAGE_KEY, JSON.stringify(mockUser));
      }
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
      updateUser,
      logout,
    }),
    [user, isLoading, updateUser, logout]
  );
});
