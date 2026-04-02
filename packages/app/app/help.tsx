import Colors from "@/constants/colors";
import {
  ArrowLeft,
  ChevronRight,
  Clock,
  HelpCircle,
  Mail,
  MapPin,
  Shield,
  Zap,
} from "lucide-react-native";
import React from "react";
import {
  Linking,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { router } from "expo-router";

const FAQ_ITEMS = [
  {
    icon: Zap,
    question: "How does ACT work?",
    answer:
      "ACT uses autonomous golf carts to give you rides around the UCF campus. Just select your pickup and destination, and a self-driving cart will come to you.",
  },
  {
    icon: Clock,
    question: "How long does a ride take?",
    answer:
      "Most campus rides take 3-8 minutes depending on distance. Wait times for a cart are typically 2-5 minutes.",
  },
  {
    icon: MapPin,
    question: "Where can I ride?",
    answer:
      "ACT operates within the UCF main campus. You can be picked up and dropped off at any campus location or drop a custom pin on the map.",
  },
  {
    icon: Shield,
    question: "Is it safe?",
    answer:
      "ACT carts have multiple safety sensors and cameras. The carts operate at low speeds and can be manually overridden at any time.",
  },
  {
    icon: HelpCircle,
    question: "What if my cart doesn't arrive?",
    answer:
      "You can cancel your ride at any time from the app. If a cart is unavailable, you'll be notified and can request another ride.",
  },
];

export default function HelpScreen() {
  const insets = useSafeAreaInsets();

  return (
    <View style={[styles.container, { paddingTop: insets.top }]}>
      <View style={styles.header}>
        <TouchableOpacity onPress={() => router.back()} style={styles.backBtn}>
          <ArrowLeft size={24} color={Colors.text} />
        </TouchableOpacity>
        <Text style={styles.headerTitle}>Help Center</Text>
        <View style={{ width: 40 }} />
      </View>

      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={styles.scrollContent}
        showsVerticalScrollIndicator={false}
      >
        <Text style={styles.sectionTitle}>FREQUENTLY ASKED QUESTIONS</Text>

        <View style={styles.card}>
          {FAQ_ITEMS.map((item, index) => {
            const Icon = item.icon;
            return (
              <View
                key={index}
                style={[
                  styles.faqItem,
                  index < FAQ_ITEMS.length - 1 && styles.faqItemBorder,
                ]}
              >
                <View style={styles.faqHeader}>
                  <View style={styles.iconContainer}>
                    <Icon size={18} color={Colors.accent} />
                  </View>
                  <Text style={styles.faqQuestion}>{item.question}</Text>
                </View>
                <Text style={styles.faqAnswer}>{item.answer}</Text>
              </View>
            );
          })}
        </View>

        <Text style={[styles.sectionTitle, { marginTop: 32 }]}>CONTACT US</Text>

        <View style={styles.card}>
          <TouchableOpacity
            style={styles.contactRow}
            onPress={() => Linking.openURL("mailto:act@ucf.edu")}
          >
            <View style={styles.contactLeft}>
              <View style={styles.iconContainer}>
                <Mail size={18} color={Colors.accent} />
              </View>
              <View>
                <Text style={styles.contactTitle}>Email Support</Text>
                <Text style={styles.contactSub}>act@ucf.edu</Text>
              </View>
            </View>
            <ChevronRight size={20} color={Colors.textSecondary} />
          </TouchableOpacity>
        </View>

        <View style={styles.emergencyCard}>
          <Text style={styles.emergencyTitle}>Emergency?</Text>
          <Text style={styles.emergencyText}>
            If you're in an unsafe situation, call UCF Police at{" "}
          </Text>
          <TouchableOpacity onPress={() => Linking.openURL("tel:4078235555")}>
            <Text style={styles.emergencyPhone}>(407) 823-5555</Text>
          </TouchableOpacity>
        </View>
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
  scrollView: {
    flex: 1,
  },
  scrollContent: {
    padding: 20,
    paddingBottom: 40,
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
  faqItem: {
    padding: 16,
  },
  faqItemBorder: {
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  faqHeader: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    marginBottom: 8,
  },
  iconContainer: {
    width: 32,
    height: 32,
    borderRadius: 8,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
  },
  faqQuestion: {
    fontSize: 15,
    fontWeight: "600",
    color: Colors.text,
    flex: 1,
  },
  faqAnswer: {
    fontSize: 14,
    lineHeight: 20,
    color: Colors.textSecondary,
    paddingLeft: 44,
  },
  contactRow: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    padding: 16,
  },
  contactLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  contactTitle: {
    fontSize: 15,
    fontWeight: "500",
    color: Colors.text,
  },
  contactSub: {
    fontSize: 13,
    color: Colors.textSecondary,
    marginTop: 2,
  },
  emergencyCard: {
    marginTop: 24,
    backgroundColor: Colors.surface,
    borderRadius: 16,
    padding: 20,
    alignItems: "center",
    borderWidth: 1,
    borderColor: "rgba(239,68,68,0.3)",
  },
  emergencyTitle: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.error,
    marginBottom: 4,
  },
  emergencyText: {
    fontSize: 14,
    color: Colors.textSecondary,
    textAlign: "center",
  },
  emergencyPhone: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.accent,
    marginTop: 4,
  },
});
