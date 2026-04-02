import Colors from "@/constants/colors";
import { ArrowLeft, Moon, Sun } from "lucide-react-native";
import React, { useState } from "react";
import { StyleSheet, Switch, Text, TouchableOpacity, View } from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { router } from "expo-router";

export default function SettingsScreen() {
  const insets = useSafeAreaInsets();
  const [lightMode, setLightMode] = useState(false);

  return (
    <View style={[styles.container, { paddingTop: insets.top }]}>
      <View style={styles.header}>
        <TouchableOpacity onPress={() => router.back()} style={styles.backBtn}>
          <ArrowLeft size={24} color={Colors.text} />
        </TouchableOpacity>
        <Text style={styles.headerTitle}>Settings</Text>
        <View style={{ width: 40 }} />
      </View>

      <View style={styles.content}>
        <Text style={styles.sectionTitle}>APPEARANCE</Text>
        <View style={styles.card}>
          <View style={styles.settingRow}>
            <View style={styles.settingLeft}>
              <View style={styles.iconContainer}>
                {lightMode ? (
                  <Sun size={20} color={Colors.accent} />
                ) : (
                  <Moon size={20} color={Colors.accent} />
                )}
              </View>
              <View>
                <Text style={styles.settingTitle}>Light Mode</Text>
                <Text style={styles.settingDescription}>
                  {lightMode ? "Light theme active" : "Dark theme active"}
                </Text>
              </View>
            </View>
            <Switch
              value={lightMode}
              onValueChange={setLightMode}
              trackColor={{ false: Colors.border, true: Colors.accent }}
              thumbColor={Colors.white}
              ios_backgroundColor={Colors.border}
            />
          </View>
        </View>

        {lightMode && (
          <View style={styles.comingSoonBanner}>
            <Text style={styles.comingSoonText}>
              Light mode isn't coming Keating
            </Text>
          </View>
        )}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  header: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    paddingHorizontal: 20,
    paddingVertical: 16,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  backBtn: {
    width: 40,
    height: 40,
    alignItems: "center",
    justifyContent: "center",
  },
  headerTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: Colors.text,
  },
  content: {
    padding: 20,
  },
  sectionTitle: {
    fontSize: 12,
    fontWeight: "600",
    color: Colors.textSecondary,
    letterSpacing: 0.8,
    marginBottom: 8,
    paddingLeft: 4,
  },
  card: {
    backgroundColor: Colors.surface,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: Colors.border,
    overflow: "hidden",
  },
  settingRow: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    padding: 16,
  },
  settingLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    flex: 1,
  },
  iconContainer: {
    width: 36,
    height: 36,
    borderRadius: 10,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
  },
  settingTitle: {
    fontSize: 15,
    fontWeight: "500",
    color: Colors.text,
  },
  settingDescription: {
    fontSize: 12,
    color: Colors.textSecondary,
    marginTop: 2,
  },
  comingSoonBanner: {
    marginTop: 12,
    backgroundColor: Colors.card,
    borderRadius: 12,
    padding: 14,
    borderWidth: 1,
    borderColor: Colors.border,
  },
  comingSoonText: {
    fontSize: 13,
    color: Colors.textSecondary,
    textAlign: "center",
    lineHeight: 18,
  },
});
