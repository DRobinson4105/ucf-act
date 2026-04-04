import Colors from "@/constants/colors";
import { ArrowLeft } from "lucide-react-native";
import React, { useState } from "react";
import {
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { router } from "expo-router";

type Tab = "terms" | "privacy";

export default function TermsScreen() {
  const insets = useSafeAreaInsets();
  const [activeTab, setActiveTab] = useState<Tab>("terms");

  return (
    <View style={[styles.container, { paddingTop: insets.top }]}>
      <View style={styles.header}>
        <TouchableOpacity onPress={() => router.back()} style={styles.backBtn}>
          <ArrowLeft size={24} color={Colors.text} />
        </TouchableOpacity>
        <Text style={styles.headerTitle}>Legal</Text>
        <View style={{ width: 40 }} />
      </View>

      <View style={styles.tabBar}>
        <TouchableOpacity
          style={[styles.tab, activeTab === "terms" && styles.tabActive]}
          onPress={() => setActiveTab("terms")}
        >
          <Text
            style={[
              styles.tabText,
              activeTab === "terms" && styles.tabTextActive,
            ]}
          >
            Terms of Service
          </Text>
        </TouchableOpacity>
        <TouchableOpacity
          style={[styles.tab, activeTab === "privacy" && styles.tabActive]}
          onPress={() => setActiveTab("privacy")}
        >
          <Text
            style={[
              styles.tabText,
              activeTab === "privacy" && styles.tabTextActive,
            ]}
          >
            Privacy Policy
          </Text>
        </TouchableOpacity>
      </View>

      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={styles.scrollContent}
        showsVerticalScrollIndicator={false}
        keyboardShouldPersistTaps="handled"
      >
        {activeTab === "terms" ? <TermsContent /> : <PrivacyContent />}
        <Text style={styles.lastUpdated}>Last updated: March 2026</Text>
      </ScrollView>
    </View>
  );
}

function TermsContent() {
  return (
    <>
      <Text style={styles.sectionHeading}>1. Acceptance of Terms</Text>
      <Text style={styles.body}>
        By downloading, installing, or using the ACT (Autonomous Campus Transit)
        mobile application, you agree to be bound by these Terms of Service. ACT
        is a research and development project operated by a Senior Design team
        at UCF. Use of the service constitutes acceptance of all terms herein.
      </Text>

      <Text style={styles.sectionHeading}>2. Service Description</Text>
      <Text style={styles.body}>
        ACT provides autonomous golf cart transportation within the UCF campus.
        The service is free during the Senior Design course. Rides are subject
        to vehicle availability, weather conditions, and operational hours. ACT
        reserves the right to modify, suspend, or discontinue the service at any
        time.
      </Text>

      <Text style={styles.sectionHeading}>3. User Eligibility</Text>
      <Text style={styles.body}>
        You must be a current UCF student, faculty member, or staff with a valid
        university affiliation.
      </Text>

      <Text style={styles.sectionHeading}>4. User Conduct</Text>
      <Text style={styles.body}>
        While using the ACT service, you agree to: remain seated during transit,
        follow all posted safety instructions, not interfere with vehicle
        sensors or controls, not damage or tamper with the vehicle, and report
        any safety concerns immediately through the app.
      </Text>

      <Text style={styles.sectionHeading}>5. Safety & Liability</Text>
      <Text style={styles.body}>
        ACT vehicles are autonomous and operate under supervised conditions.
        While safety is our priority, you acknowledge that autonomous vehicle
        technology is still under development. UCF and the ACT team are not
        liable for delays, service interruptions, or incidents arising from the
        use of the service to the fullest extent permitted by law.
      </Text>

      <Text style={styles.sectionHeading}>6. Data Collection</Text>
      <Text style={styles.body}>
        We collect ride data, location information, and usage analytics to
        improve the service. See our Privacy Policy for details on data
        handling. By using the service, you consent to this data collection.
      </Text>

      <Text style={styles.sectionHeading}>7. Modifications</Text>
      <Text style={styles.body}>
        We reserve the right to update these terms at any time. Continued use of
        the app after changes constitutes acceptance of the revised terms.
        Material changes will be communicated through the app.
      </Text>
    </>
  );
}

function PrivacyContent() {
  return (
    <>
      <Text style={styles.sectionHeading}>Information We Collect</Text>
      <Text style={styles.body}>
        We collect the following information when you use ACT:{"\n\n"}
        {"\u2022"} Account information (name, email) via Apple Sign-In{"\n"}
        {"\u2022"} Location data during active rides and when the app is in use
        {"\n"}
        {"\u2022"} Ride history (pickup/dropoff locations, timestamps, ratings)
        {"\n"}
        {"\u2022"} Device information (model, OS version) for compatibility
        {"\n"}
        {"\u2022"} Push notification tokens for ride status updates
      </Text>

      <Text style={styles.sectionHeading}>How We Use Your Data</Text>
      <Text style={styles.body}>
        Your data is used to:{"\n\n"}
        {"\u2022"} Provide and improve the ACT ride service{"\n"}
        {"\u2022"} Send ride status notifications{"\n"}
        {"\u2022"} Optimize routes and vehicle dispatching{"\n"}
        {"\u2022"} Conduct research on autonomous transit systems{"\n"}
        {"\u2022"} Ensure safety and investigate incidents
      </Text>

      <Text style={styles.sectionHeading}>Data Sharing</Text>
      <Text style={styles.body}>
        We do not sell your personal information. Data may be shared with: UCF
        research teams (anonymized for academic publications), service providers
        necessary for app operation (Convex, Expo), and sponsors.
      </Text>

      <Text style={styles.sectionHeading}>Data Retention</Text>
      <Text style={styles.body}>
        Ride history is retained for the duration of your account. Location data
        from completed rides is retained for 90 days, then anonymized. You may
        request account deletion by contacting the ACT team, which will remove
        all personally identifiable information.
      </Text>

      <Text style={styles.sectionHeading}>Your Rights</Text>
      <Text style={styles.body}>
        You have the right to: access your personal data, request correction of
        inaccurate data, request deletion of your account and data, opt out of
        non-essential data collection, and receive a copy of your data in a
        portable format.
      </Text>

      <Text style={styles.sectionHeading}>Contact</Text>
      <Text style={styles.body}>
        For privacy-related inquiries, contact the ACT team through the Help
        Center in the app.
      </Text>
    </>
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
  tabBar: {
    flexDirection: "row",
    paddingHorizontal: 20,
    paddingTop: 16,
    gap: 8,
  },
  tab: {
    flex: 1,
    paddingVertical: 10,
    borderRadius: 10,
    backgroundColor: Colors.surface,
    alignItems: "center",
    borderWidth: 1,
    borderColor: Colors.border,
  },
  tabActive: {
    backgroundColor: Colors.accent,
    borderColor: Colors.accent,
  },
  tabText: {
    fontSize: 13,
    fontWeight: "600",
    color: Colors.textSecondary,
  },
  tabTextActive: {
    color: Colors.black,
  },
  scrollView: {
    flex: 1,
  },
  scrollContent: {
    padding: 20,
    paddingBottom: 40,
  },
  sectionHeading: {
    fontSize: 16,
    fontWeight: "700",
    color: Colors.text,
    marginTop: 24,
    marginBottom: 8,
  },
  body: {
    fontSize: 14,
    lineHeight: 22,
    color: Colors.textSecondary,
  },
  lastUpdated: {
    fontSize: 12,
    color: Colors.textSecondary,
    textAlign: "center",
    marginTop: 32,
    opacity: 0.6,
  },
});
