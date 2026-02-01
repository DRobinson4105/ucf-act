import Colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { useRide } from "@/contexts/RideContext";
import {
  ChevronRight,
  CreditCard,
  FileText,
  HelpCircle,
  LogOut,
  Mail,
  Settings,
  User,
} from "lucide-react-native";
import React from "react";
import {
  Alert,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export default function ProfileScreen() {
  const insets = useSafeAreaInsets();
  const { user, logout } = useAuth();
  const { rideHistory } = useRide();

  const handleLogout = () => {
    Alert.alert("Logout", "Are you sure you want to logout?", [
      { text: "Cancel", style: "cancel" },
      {
        text: "Logout",
        style: "destructive",
        onPress: logout,
      },
    ]);
  };

  const MenuItem = ({
    icon,
    title,
    onPress,
    showBorder = true,
  }: {
    icon: React.ReactNode;
    title: string;
    onPress: () => void;
    showBorder?: boolean;
  }) => (
    <TouchableOpacity
      style={[styles.menuItem, !showBorder && styles.menuItemNoBorder]}
      onPress={onPress}
    >
      <View style={styles.menuLeft}>
        <View style={styles.iconContainer}>
          <Text>{icon}</Text>
        </View>
        <Text style={styles.menuTitle}>{title}</Text>
      </View>
      <ChevronRight size={20} color={Colors.textSecondary} />
    </TouchableOpacity>
  );

  return (
    <View style={styles.container}>
      <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
        <Text style={styles.title}>Profile</Text>
      </View>

      <ScrollView
        style={styles.scrollView}
        showsVerticalScrollIndicator={false}
      >
        <View style={styles.profileSection}>
          <View style={styles.avatarContainer}>
            <View style={styles.avatar}>
              <User size={40} color={Colors.white} />
            </View>
          </View>
          <Text style={styles.userName}>{user?.name}</Text>
          <Text style={styles.userEmail}>{user?.email}</Text>
          {user?.campusId && (
            <View style={styles.campusIdBadge}>
              <Text style={styles.campusIdText}>{user.campusId}</Text>
            </View>
          )}

          <View style={styles.statsContainer}>
            <View style={styles.statItem}>
              <Text style={styles.statValue}>{rideHistory.length}</Text>
              <Text style={styles.statLabel}>Total Rides</Text>
            </View>
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Account</Text>
          <View style={styles.menuCard}>
            <MenuItem
              icon={<Mail size={20} color={Colors.accent} />}
              title="Edit Profile"
              onPress={() => console.log("Edit Profile")}
            />
            <MenuItem
              icon={<CreditCard size={20} color={Colors.accent} />}
              title="Payment Methods"
              onPress={() => console.log("Payment Methods")}
              showBorder={false}
            />
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Support</Text>
          <View style={styles.menuCard}>
            <MenuItem
              icon={<HelpCircle size={20} color={Colors.accent} />}
              title="Help Center"
              onPress={() => console.log("Help Center")}
            />
            <MenuItem
              icon={<FileText size={20} color={Colors.accent} />}
              title="Terms & Privacy"
              onPress={() => console.log("Terms & Privacy")}
              showBorder={false}
            />
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Preferences</Text>
          <View style={styles.menuCard}>
            <MenuItem
              icon={<Settings size={20} color={Colors.accent} />}
              title="Settings"
              onPress={() => console.log("Settings")}
              showBorder={false}
            />
          </View>
        </View>

        <TouchableOpacity style={styles.logoutButton} onPress={handleLogout}>
          <LogOut size={20} color={Colors.error} />
          <Text style={styles.logoutText}>Logout</Text>
        </TouchableOpacity>

        <Text style={styles.version}>ACT v1.0.0</Text>
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  header: {
    paddingHorizontal: 20,
    paddingBottom: 20,
    backgroundColor: Colors.surface,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  title: {
    fontSize: 28,
    fontWeight: "700" as const,
    color: Colors.text,
  },
  scrollView: {
    flex: 1,
  },
  profileSection: {
    backgroundColor: Colors.surface,
    paddingVertical: 32,
    alignItems: "center",
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  avatarContainer: {
    marginBottom: 16,
  },
  avatar: {
    width: 80,
    height: 80,
    borderRadius: 40,
    backgroundColor: Colors.accent,
    alignItems: "center",
    justifyContent: "center",
  },
  userName: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 4,
  },
  userEmail: {
    fontSize: 14,
    color: Colors.textSecondary,
    marginBottom: 12,
  },
  campusIdBadge: {
    backgroundColor: Colors.card,
    paddingHorizontal: 16,
    paddingVertical: 6,
    borderRadius: 12,
  },
  campusIdText: {
    fontSize: 13,
    fontWeight: "600" as const,
    color: Colors.accent,
  },
  statsContainer: {
    marginTop: 24,
    flexDirection: "row",
    gap: 32,
  },
  statItem: {
    alignItems: "center",
  },
  statValue: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: Colors.text,
  },
  statLabel: {
    fontSize: 13,
    color: Colors.textSecondary,
    marginTop: 4,
  },
  section: {
    marginTop: 24,
    paddingHorizontal: 16,
  },
  sectionTitle: {
    fontSize: 13,
    fontWeight: "600" as const,
    color: Colors.textSecondary,
    marginBottom: 8,
    paddingHorizontal: 4,
    textTransform: "uppercase",
  },
  menuCard: {
    backgroundColor: Colors.surface,
    borderRadius: 16,
    overflow: "hidden",
    borderWidth: 1,
    borderColor: Colors.border,
  },
  menuItem: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  menuItemNoBorder: {
    borderBottomWidth: 0,
  },
  menuLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  iconContainer: {
    width: 36,
    height: 36,
    borderRadius: 10,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
  },
  menuTitle: {
    fontSize: 15,
    fontWeight: "500" as const,
    color: Colors.text,
  },
  logoutButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 8,
    marginHorizontal: 16,
    marginTop: 32,
    paddingVertical: 16,
    backgroundColor: Colors.surface,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: Colors.error,
  },
  logoutText: {
    fontSize: 15,
    fontWeight: "600" as const,
    color: Colors.error,
  },
  version: {
    fontSize: 12,
    color: Colors.textSecondary,
    textAlign: "center",
    marginTop: 24,
    marginBottom: 32,
  },
});
