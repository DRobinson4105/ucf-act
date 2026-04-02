import Colors from "@/constants/colors";
import { ArrowLeft, CreditCard } from "lucide-react-native";
import React from "react";
import { StyleSheet, Text, TouchableOpacity, View } from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { router } from "expo-router";

export default function PaymentScreen() {
  const insets = useSafeAreaInsets();

  return (
    <View style={[styles.container, { paddingTop: insets.top }]}>
      <View style={styles.header}>
        <TouchableOpacity onPress={() => router.back()} style={styles.backBtn}>
          <ArrowLeft size={24} color={Colors.text} />
        </TouchableOpacity>
        <Text style={styles.headerTitle}>Payment</Text>
        <View style={{ width: 40 }} />
      </View>

      <View style={styles.content}>
        <View style={styles.freeCard}>
          <View style={styles.freeBadge}>
            <Text style={styles.freeBadgeText}>FREE</Text>
          </View>
          <Text style={styles.freeTitle}>ACT is free during the pilot</Text>
          <Text style={styles.freeDescription}>
            All rides are completely free while during Senior Design at UCF. No
            payment method is required.
          </Text>
        </View>

        <View style={styles.placeholderCard}>
          <View style={styles.placeholderIcon}>
            <CreditCard size={32} color={Colors.textSecondary} />
          </View>
          <Text style={styles.placeholderTitle}>
            Payment methods coming soon
          </Text>
          <Text style={styles.placeholderDescription}>
            When ACT moves beyond the Senior Design course, you'll be able to
            add payment methods here.
          </Text>
        </View>
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
    gap: 16,
  },
  freeCard: {
    backgroundColor: Colors.surface,
    borderRadius: 16,
    padding: 24,
    alignItems: "center",
    borderWidth: 1,
    borderColor: Colors.accent,
  },
  freeBadge: {
    backgroundColor: Colors.accent,
    paddingHorizontal: 16,
    paddingVertical: 6,
    borderRadius: 20,
    marginBottom: 16,
  },
  freeBadgeText: {
    fontSize: 14,
    fontWeight: "800",
    color: Colors.black,
    letterSpacing: 2,
  },
  freeTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: Colors.text,
    textAlign: "center",
    marginBottom: 8,
  },
  freeDescription: {
    fontSize: 14,
    lineHeight: 20,
    color: Colors.textSecondary,
    textAlign: "center",
  },
  placeholderCard: {
    backgroundColor: Colors.surface,
    borderRadius: 16,
    padding: 32,
    alignItems: "center",
    borderWidth: 1,
    borderColor: Colors.border,
  },
  placeholderIcon: {
    width: 64,
    height: 64,
    borderRadius: 32,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 16,
  },
  placeholderTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: Colors.text,
    textAlign: "center",
    marginBottom: 8,
  },
  placeholderDescription: {
    fontSize: 14,
    lineHeight: 20,
    color: Colors.textSecondary,
    textAlign: "center",
  },
});
