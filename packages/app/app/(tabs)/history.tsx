import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useRide } from "@/contexts/RideContext";
import { Clock, MapPin } from "lucide-react-native";
import React from "react";
import {
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export default function HistoryScreen() {
  const insets = useSafeAreaInsets();
  const { rideHistory } = useRide();

  const getLocationName = (id: string) => {
    return CAMPUS_LOCATIONS.find((l) => l.id === id)?.name || "Unknown";
  };

  const formatDate = (date: Date) => {
    return new Intl.DateTimeFormat("en-US", {
      month: "short",
      day: "numeric",
      hour: "numeric",
      minute: "2-digit",
    }).format(date);
  };

  const getDuration = (start?: Date, end?: Date) => {
    if (!start || !end) return null;
    const diff = Math.floor((end.getTime() - start.getTime()) / 1000 / 60);
    return `${diff} min`;
  };

  return (
    <View style={styles.container}>
      <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
        <Text style={styles.title}>Ride History</Text>
      </View>

      <ScrollView
        style={styles.scrollView}
        showsVerticalScrollIndicator={false}
      >
        {rideHistory.length === 0 ? (
          <View style={styles.emptyState}>
            <Clock size={48} color={Colors.textSecondary} />
            <Text style={styles.emptyTitle}>No rides yet</Text>
            <Text style={styles.emptySubtitle}>
              Your ride history will appear here
            </Text>
          </View>
        ) : (
          <View style={styles.list}>
            {rideHistory.map((ride) => (
              <TouchableOpacity key={ride.id} style={styles.rideCard}>
                <View style={styles.rideHeader}>
                  <View style={styles.vehicleInfo}>
                    <Text style={styles.vehicleId}>{ride.vehicleId}</Text>
                    <View style={styles.statusBadge}>
                      <Text style={styles.statusText}>Completed</Text>
                    </View>
                  </View>
                  <Text style={styles.date}>
                    {formatDate(ride.requestedAt)}
                  </Text>
                </View>

                <View style={styles.routeContainer}>
                  <View style={styles.routeIndicator}>
                    <View style={styles.dot} />
                    <View style={styles.line} />
                    <View style={[styles.dot, styles.dotDestination]} />
                  </View>

                  <View style={styles.locations}>
                    <View style={styles.locationRow}>
                      <MapPin size={16} color={Colors.accent} />
                      <Text style={styles.locationText}>
                        {getLocationName(ride.pickupLocationId)}
                      </Text>
                    </View>

                    <View style={styles.locationRow}>
                      <MapPin size={16} color={Colors.primary} />
                      <Text style={styles.locationText}>
                        {getLocationName(ride.dropoffLocationId)}
                      </Text>
                    </View>
                  </View>
                </View>

                {ride.pickupAt && ride.completedAt && (
                  <View style={styles.footer}>
                    <Clock size={14} color={Colors.textSecondary} />
                    <Text style={styles.duration}>
                      {getDuration(ride.pickupAt, ride.completedAt)}
                    </Text>
                  </View>
                )}
              </TouchableOpacity>
            ))}
          </View>
        )}
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
    backgroundColor: Colors.white,
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
  emptyState: {
    alignItems: "center",
    justifyContent: "center",
    paddingTop: 80,
    paddingHorizontal: 40,
  },
  emptyTitle: {
    fontSize: 20,
    fontWeight: "600" as const,
    color: Colors.text,
    marginTop: 16,
  },
  emptySubtitle: {
    fontSize: 14,
    color: Colors.textSecondary,
    marginTop: 8,
    textAlign: "center",
  },
  list: {
    padding: 16,
    gap: 12,
  },
  rideCard: {
    backgroundColor: Colors.white,
    borderRadius: 16,
    padding: 16,
    borderWidth: 1,
    borderColor: Colors.border,
  },
  rideHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 16,
  },
  vehicleInfo: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  vehicleId: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.primary,
  },
  statusBadge: {
    backgroundColor: Colors.success,
    paddingHorizontal: 8,
    paddingVertical: 2,
    borderRadius: 8,
  },
  statusText: {
    fontSize: 11,
    fontWeight: "600" as const,
    color: Colors.white,
  },
  date: {
    fontSize: 13,
    color: Colors.textSecondary,
  },
  routeContainer: {
    flexDirection: "row",
    gap: 12,
  },
  routeIndicator: {
    alignItems: "center",
    paddingTop: 4,
  },
  dot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: Colors.accent,
  },
  dotDestination: {
    backgroundColor: Colors.primary,
  },
  line: {
    width: 2,
    flex: 1,
    backgroundColor: Colors.border,
    marginVertical: 4,
  },
  locations: {
    flex: 1,
    gap: 24,
  },
  locationRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  locationText: {
    fontSize: 15,
    color: Colors.text,
    flex: 1,
  },
  footer: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    marginTop: 12,
    paddingTop: 12,
    borderTopWidth: 1,
    borderTopColor: Colors.border,
  },
  duration: {
    fontSize: 13,
    color: Colors.textSecondary,
  },
});
