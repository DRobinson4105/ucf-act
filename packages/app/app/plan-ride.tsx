import CampusMap from "@/components/CampusMap";
import SwipeableBottomSheet from "@/components/SwipeableBottomSheet";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useRide } from "@/contexts/RideContext";
import { router, useLocalSearchParams } from "expo-router";
import {
    ArrowLeft,
    Car,
    Clock,
    MapPin,
    Navigation,
    Search,
    X,
} from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
    Animated,
    FlatList,
    Keyboard,
    ScrollView,
    StyleSheet,
    Text,
    TextInput,
    TouchableOpacity,
    View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

type SelectionMode = "pickup" | "dropoff" | null;
type ViewMode = "planning" | "choosing" | "finding" | "tracking";

interface RideOption {
  id: string;
  name: string;
  description: string;
  estimatedTime: string;
  capacity: number;
  price: string;
}

const RIDE_OPTIONS: RideOption[] = [
  {
    id: "act-standard",
    name: "ACT Cart",
    description: "Self-driving campus shuttle",
    estimatedTime: "3-5 min",
    capacity: 4,
    price: "Free",
  },
];

interface RideTrackingContentProps {
  pickupLocation: (typeof CAMPUS_LOCATIONS)[0] | null;
  dropoffLocation: (typeof CAMPUS_LOCATIONS)[0] | null | undefined;
  onBack: () => void;
}

function RideTrackingContent({
  pickupLocation,
  dropoffLocation,
  onBack,
}: RideTrackingContentProps) {
  const { currentRide, cancelRide } = useRide();

  const getStatusInfo = () => {
    if (!currentRide) {
      return {
        title: "No active ride",
        subtitle: "",
        showTimer: false,
      };
    }

    switch (currentRide.status) {
      case "pending":
        return {
          title: "Requesting Cart...",
          subtitle: "Finding an available ACT cart",
          showTimer: false,
        };
      case "assigned":
        return {
          title: `${currentRide.vehicle.id} is on the way`,
          subtitle: `Arriving in ${currentRide.estimatedWaitTime} min`,
          showTimer: true,
        };
      case "arriving":
        return {
          title: "Cart Arriving",
          subtitle: "Your ACT cart is almost there!",
          showTimer: true,
        };
      case "in-progress":
        return {
          title: "Trip in Progress",
          subtitle: "Enjoy your ride!",
          showTimer: false,
        };
      default:
        return {
          title: "Ride Status",
          subtitle: "",
          showTimer: false,
        };
    }
  };

  const handleCancelRide = () => {
    cancelRide();
    onBack();
  };

  const statusInfo = getStatusInfo();

  return (
    <ScrollView
      style={styles.sheetContent}
      showsVerticalScrollIndicator={false}
    >
      {statusInfo.showTimer && currentRide && (
        <View style={styles.timerContainer}>
          <Text style={styles.timerLabel}>The cart will arrive in</Text>
          <Text style={styles.timerValue}>
            {String(currentRide.estimatedWaitTime || 0).padStart(2, "0")}:
            {Math.floor(Math.random() * 60)
              .toString()
              .padStart(2, "0")}{" "}
            Mins
          </Text>
        </View>
      )}

      <View style={styles.vehicleCard}>
        <View style={styles.vehicleInfo}>
          <View>
            <Text style={styles.licensePlate}>
              {currentRide?.vehicle?.licensePlate || "ACT-001"}
            </Text>
            <Text style={styles.vehicleDetails}>
              {currentRide?.vehicle?.model || "ACT Golf Cart"} •{" "}
              {currentRide?.vehicle?.color || "White"}
            </Text>
            <Text style={styles.autonomousLabel}>🤖 Self-Driving Cart</Text>
          </View>
          <View style={styles.vehicleIcon}>
            <Text style={styles.vehicleEmoji}>🚗</Text>
          </View>
        </View>
      </View>

      <View style={styles.rideLocationInfo}>
        <View style={styles.rideLocationItem}>
          <View style={styles.locationDot} />
          <View style={styles.locationTextContainer}>
            <Text style={styles.locationLabel}>Pickup Location</Text>
            <Text style={styles.locationText}>
              {pickupLocation?.name || "Your Current Location"}
            </Text>
          </View>
        </View>
        <View style={styles.locationItem}>
          <MapPin size={20} color={Colors.text} />
          <View style={styles.locationTextContainer}>
            <Text style={styles.locationLabel}>Destination</Text>
            <Text style={styles.locationText}>{dropoffLocation?.name}</Text>
          </View>
        </View>
      </View>

      <TouchableOpacity style={styles.cancelButton} onPress={handleCancelRide}>
        <X size={20} color={Colors.error} />
        <Text style={styles.cancelButtonText}>Cancel Ride</Text>
      </TouchableOpacity>
    </ScrollView>
  );
}

function LoadingDot() {
  const opacity = useRef(new Animated.Value(0.3)).current;

  useEffect(() => {
    const animation = Animated.loop(
      Animated.sequence([
        Animated.timing(opacity, {
          toValue: 1,
          duration: 600,
          useNativeDriver: true,
        }),
        Animated.timing(opacity, {
          toValue: 0.3,
          duration: 600,
          useNativeDriver: true,
        }),
      ])
    );
    animation.start();
    return () => animation.stop();
  }, [opacity]);

  return <Animated.View style={[styles.loadingDot, { opacity }]} />;
}

export default function PlanRideScreen() {
  const insets = useSafeAreaInsets();
  const params = useLocalSearchParams();
  const { requestRide, currentRide } = useRide();
  const [pickupLocation, setPickupLocation] =
    useState<string>("Current Location");
  const [dropoffLocation, setDropoffLocation] = useState<string>("");
  const [selectedPickupId, setSelectedPickupId] = useState<string | null>(
    "current-location"
  );
  const [selectedDropoffId, setSelectedDropoffId] = useState<string | null>(
    null
  );
  const [pickupSearchQuery, setPickupSearchQuery] = useState("");
  const [dropoffSearchQuery, setDropoffSearchQuery] = useState("");
  const [isExpanded, setIsExpanded] = useState(false);
  const [selectionMode, setSelectionMode] = useState<SelectionMode>("dropoff");
  const [previousPickupLocation, setPreviousPickupLocation] = useState("");
  const [previousDropoffLocation, setPreviousDropoffLocation] = useState("");
  const [viewMode, setViewMode] = useState<ViewMode>("planning");
  const [selectedRide, setSelectedRide] = useState<string>(RIDE_OPTIONS[0].id);
  const sheetRef = useRef<{ expand: () => void } | null>(null);
  const destinationInputRef = useRef<TextInput>(null);
  const pickupInputRef = useRef<TextInput>(null);

  useEffect(() => {
    if (params.destinationId) {
      const destination = CAMPUS_LOCATIONS.find(
        (l) => l.id === params.destinationId
      );
      if (destination) {
        setDropoffLocation(destination.name);
        setSelectedDropoffId(destination.id);
        setSelectionMode(null);
      }
    }
  }, [params.destinationId]);

  const filteredPickupLocations = CAMPUS_LOCATIONS.filter(
    (location) =>
      location.name.toLowerCase().includes(pickupSearchQuery.toLowerCase()) ||
      location.shortName.toLowerCase().includes(pickupSearchQuery.toLowerCase())
  );

  const filteredDropoffLocations = CAMPUS_LOCATIONS.filter(
    (location) =>
      location.name.toLowerCase().includes(dropoffSearchQuery.toLowerCase()) ||
      location.shortName
        .toLowerCase()
        .includes(dropoffSearchQuery.toLowerCase())
  );

  const handlePickupSelect = (location: (typeof CAMPUS_LOCATIONS)[0]) => {
    setPickupLocation(location.name);
    setSelectedPickupId(location.id);
    setPickupSearchQuery("");
    setPreviousPickupLocation(location.name);
    setSelectionMode(null);
    Keyboard.dismiss();
  };

  const handleSetCurrentLocation = () => {
    setPickupLocation("Current Location");
    setSelectedPickupId("current-location");
    setPickupSearchQuery("");
    setPreviousPickupLocation("Current Location");
    setSelectionMode(null);
    Keyboard.dismiss();
  };

  const handleDropoffSelect = (location: (typeof CAMPUS_LOCATIONS)[0]) => {
    setDropoffLocation(location.name);
    setSelectedDropoffId(location.id);
    setDropoffSearchQuery("");
    setPreviousDropoffLocation(location.name);
    setSelectionMode(null);
    Keyboard.dismiss();
  };

  const handleConfirm = () => {
    if (selectedPickupId && selectedDropoffId) {
      setViewMode("choosing");
      setIsExpanded(true);
      sheetRef.current?.expand();
    }
  };

  const handleConfirmRide = () => {
    if (selectedPickupId && selectedDropoffId) {
      setViewMode("finding");
      requestRide(selectedPickupId, selectedDropoffId);
    }
  };

  const handleBackFromChoosing = () => {
    setViewMode("planning");
  };

  useEffect(() => {
    if (
      viewMode === "finding" &&
      currentRide &&
      currentRide.status === "assigned"
    ) {
      setViewMode("tracking");
    }
  }, [viewMode, currentRide]);

  const handleMapPress = (locationId: string) => {
    if (!isExpanded && viewMode === "planning") {
      if (selectionMode === "pickup") {
        const location = CAMPUS_LOCATIONS.find((l) => l.id === locationId);
        if (location) {
          setPickupLocation(location.name);
          setSelectedPickupId(locationId);
          setPickupSearchQuery("");
          setPreviousPickupLocation(location.name);
          setSelectionMode(null);
        }
      } else if (selectionMode === "dropoff") {
        const location = CAMPUS_LOCATIONS.find((l) => l.id === locationId);
        if (location) {
          setDropoffLocation(location.name);
          setSelectedDropoffId(locationId);
          setDropoffSearchQuery("");
          setPreviousDropoffLocation(location.name);
          setSelectionMode(null);
        }
      }
    }
  };

  const getButtonText = () => {
    if (!selectedDropoffId) {
      return "Select Destination";
    }
    if (!selectedPickupId) {
      return "Select Pickup";
    }
    return "Confirm Ride";
  };

  const isButtonDisabled =
    !selectedPickupId ||
    !selectedDropoffId ||
    selectedPickupId === selectedDropoffId;

  const pickupLocationObj =
    selectedPickupId === "current-location"
      ? null
      : CAMPUS_LOCATIONS.find((l) => l.id === selectedPickupId) || null;
  const dropoffLocationObj = CAMPUS_LOCATIONS.find(
    (l) => l.id === selectedDropoffId
  );

  return (
    <View style={styles.container}>
      <CampusMap
        selectedPickup={selectedPickupId}
        selectedDropoff={selectedDropoffId}
        onSelectPickup={handleMapPress}
        onSelectDropoff={handleMapPress}
        selectingType={selectionMode || "pickup"}
        vehiclePosition={currentRide?.vehiclePosition}
        showRoute={
          viewMode === "tracking" ||
          (pickupLocationObj !== null && dropoffLocationObj !== null)
        }
        fullScreen
        interactive={true}
      />

      <TouchableOpacity
        style={[styles.backButton, { top: insets.top + 12 }]}
        onPress={() => router.back()}
      >
        <ArrowLeft size={24} color={Colors.text} />
      </TouchableOpacity>

      <SwipeableBottomSheet
        initiallyExpanded={false}
        onSwipeUp={() => setIsExpanded(true)}
        onSwipeDown={() => {
          setIsExpanded(false);
          if (viewMode === "choosing") {
            setViewMode("planning");
          }
        }}
        expandRef={sheetRef}
      >
        {viewMode === "finding" ? (
          <View style={styles.findingContent}>
            <View style={styles.loadingContainer}>
              <LoadingDot />
            </View>
            <Text style={styles.findingTitle}>Finding available cart...</Text>
            <Text style={styles.findingSubtitle}>
              This will only take a moment
            </Text>
          </View>
        ) : viewMode === "tracking" ? (
          <RideTrackingContent
            pickupLocation={pickupLocationObj}
            dropoffLocation={dropoffLocationObj}
            onBack={() => setViewMode("planning")}
          />
        ) : viewMode === "choosing" ? (
          <ScrollView
            style={styles.sheetContent}
            showsVerticalScrollIndicator={false}
          >
            <TouchableOpacity
              style={styles.backButtonInline}
              onPress={handleBackFromChoosing}
            >
              <ArrowLeft size={24} color={Colors.text} />
              <Text style={styles.backText}>Change route</Text>
            </TouchableOpacity>

            <Text style={styles.title}>Choose a ride</Text>

            <View style={styles.optionsContainer}>
              {RIDE_OPTIONS.map((option) => (
                <TouchableOpacity
                  key={option.id}
                  style={[
                    styles.rideOption,
                    selectedRide === option.id && styles.rideOptionSelected,
                  ]}
                  onPress={() => setSelectedRide(option.id)}
                >
                  <View style={styles.rideIconContainer}>
                    <Car size={32} color={Colors.accent} />
                    <View style={styles.capacityBadge}>
                      <Text style={styles.capacityText}>{option.capacity}</Text>
                    </View>
                  </View>

                  <View style={styles.rideInfo}>
                    <View style={styles.rideHeader}>
                      <Text style={styles.rideName}>{option.name}</Text>
                      <Text style={styles.ridePrice}>{option.price}</Text>
                    </View>
                    <View style={styles.rideDetails}>
                      <Clock size={14} color={Colors.textSecondary} />
                      <Text style={styles.rideTime}>
                        {option.estimatedTime}
                      </Text>
                      <Text style={styles.rideDot}>•</Text>
                      <Text style={styles.rideDescription}>
                        {option.description}
                      </Text>
                    </View>
                  </View>
                </TouchableOpacity>
              ))}
            </View>

            <TouchableOpacity
              style={styles.confirmButton}
              onPress={handleConfirmRide}
            >
              <Text style={styles.confirmButtonText}>
                Request {RIDE_OPTIONS.find((o) => o.id === selectedRide)?.name}
              </Text>
            </TouchableOpacity>
          </ScrollView>
        ) : !isExpanded ? (
          <View style={styles.collapsedContent}>
            <TouchableOpacity
              style={styles.collapsedSearchBar}
              onPress={() => {
                if (dropoffLocation) {
                  setDropoffLocation("");
                  setSelectedDropoffId(null);
                  setDropoffSearchQuery("");
                  setPreviousDropoffLocation("");
                }
                setIsExpanded(true);
                sheetRef.current?.expand();
                setTimeout(() => {
                  destinationInputRef.current?.focus();
                }, 300);
              }}
              activeOpacity={0.7}
            >
              <View style={styles.collapsedInputRow}>
                <View style={styles.collapsedSquare} />
                <Text
                  style={[
                    styles.collapsedInputText,
                    !dropoffLocation && styles.collapsedPlaceholder,
                  ]}
                  numberOfLines={1}
                >
                  {dropoffLocation || "Where to?"}
                </Text>
              </View>
            </TouchableOpacity>
            <TouchableOpacity
              style={[
                styles.collapsedButton,
                isButtonDisabled && styles.collapsedButtonDisabled,
              ]}
              onPress={handleConfirm}
              disabled={isButtonDisabled}
            >
              <Text
                style={[
                  styles.collapsedButtonText,
                  isButtonDisabled && styles.collapsedButtonTextDisabled,
                ]}
              >
                {getButtonText()}
              </Text>
            </TouchableOpacity>
          </View>
        ) : (
          <ScrollView
            style={styles.sheetContent}
            showsVerticalScrollIndicator={false}
            keyboardShouldPersistTaps="handled"
          >
            <View style={styles.searchContainer}>
              <View style={styles.inputRow}>
                <View style={styles.iconDot} />
                <TextInput
                  ref={pickupInputRef}
                  style={styles.textInput}
                  placeholder="Pickup location"
                  placeholderTextColor="#9CA3AF"
                  value={pickupSearchQuery || pickupLocation}
                  onChangeText={(text) => {
                    setPickupSearchQuery(text);
                    setSelectionMode("pickup");
                  }}
                  onFocus={() => {
                    if (pickupLocation && !pickupSearchQuery) {
                      setPreviousPickupLocation(pickupLocation);
                      setPickupLocation("");
                      setSelectedPickupId(null);
                    }
                    setSelectionMode("pickup");
                  }}
                  onBlur={() => {
                    if (!pickupSearchQuery || !selectedPickupId) {
                      setPickupLocation(
                        previousPickupLocation || "Current Location"
                      );
                      setPickupSearchQuery("");
                      setSelectedPickupId(
                        previousPickupLocation === "Current Location"
                          ? "current-location"
                          : CAMPUS_LOCATIONS.find(
                              (l) => l.name === previousPickupLocation
                            )?.id || "current-location"
                      );
                    }
                    setSelectionMode(null);
                  }}
                  onSubmitEditing={() => {
                    if (filteredPickupLocations.length > 0) {
                      handlePickupSelect(filteredPickupLocations[0]);
                    }
                  }}
                  returnKeyType="search"
                  blurOnSubmit={false}
                />
                <TouchableOpacity
                  onPress={handleSetCurrentLocation}
                  style={styles.currentLocationButton}
                >
                  <Navigation size={20} color="#6B7280" />
                </TouchableOpacity>
              </View>

              <View style={styles.inputSeparator} />

              <View style={styles.inputRow}>
                <View style={styles.iconSquare} />
                <TextInput
                  ref={destinationInputRef}
                  style={styles.textInput}
                  placeholder="Where to?"
                  placeholderTextColor="#9CA3AF"
                  value={dropoffSearchQuery || dropoffLocation}
                  onChangeText={(text) => {
                    setDropoffSearchQuery(text);
                    setSelectionMode("dropoff");
                  }}
                  onFocus={() => {
                    if (dropoffLocation && !dropoffSearchQuery) {
                      setPreviousDropoffLocation(dropoffLocation);
                      setDropoffLocation("");
                      setSelectedDropoffId(null);
                    }
                    setSelectionMode("dropoff");
                  }}
                  onBlur={() => {
                    if (!dropoffSearchQuery || !selectedDropoffId) {
                      setDropoffLocation(previousDropoffLocation);
                      setDropoffSearchQuery("");
                      setSelectedDropoffId(
                        CAMPUS_LOCATIONS.find(
                          (l) => l.name === previousDropoffLocation
                        )?.id || null
                      );
                    }
                    setSelectionMode(null);
                  }}
                  onSubmitEditing={() => {
                    if (filteredDropoffLocations.length > 0) {
                      handleDropoffSelect(filteredDropoffLocations[0]);
                    }
                  }}
                  returnKeyType="search"
                  blurOnSubmit={false}
                />
                <Search size={20} color="#6B7280" />
              </View>
            </View>

            {(pickupSearchQuery || dropoffSearchQuery) && selectionMode && (
              <View style={styles.locationList}>
                <FlatList
                  data={
                    selectionMode === "pickup"
                      ? filteredPickupLocations
                      : filteredDropoffLocations
                  }
                  keyExtractor={(item) => item.id}
                  scrollEnabled={false}
                  keyboardShouldPersistTaps="always"
                  renderItem={({ item }) => (
                    <TouchableOpacity
                      style={styles.locationItem}
                      onPress={() => {
                        if (selectionMode === "pickup") {
                          handlePickupSelect(item);
                        } else {
                          handleDropoffSelect(item);
                        }
                      }}
                    >
                      <View style={styles.locationIcon}>
                        <Clock size={20} color="#6B7280" />
                      </View>
                      <View style={styles.locationInfo}>
                        <Text style={styles.locationName}>{item.name}</Text>
                        <Text style={styles.locationAddress}>
                          {item.shortName}
                        </Text>
                      </View>
                    </TouchableOpacity>
                  )}
                />
              </View>
            )}

            <TouchableOpacity
              style={[
                styles.confirmButton,
                isButtonDisabled && styles.confirmButtonDisabled,
              ]}
              onPress={handleConfirm}
              disabled={isButtonDisabled}
            >
              <Text
                style={[
                  styles.confirmButtonText,
                  isButtonDisabled && styles.confirmButtonTextDisabled,
                ]}
              >
                {getButtonText()}
              </Text>
            </TouchableOpacity>
          </ScrollView>
        )}
      </SwipeableBottomSheet>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.background,
  },
  backButton: {
    position: "absolute",
    top: 60,
    left: 20,
    width: 48,
    height: 48,
    borderRadius: 24,
    backgroundColor: Colors.surface,
    alignItems: "center",
    justifyContent: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.5,
    shadowRadius: 8,
    elevation: 10,
    zIndex: 100,
  },

  collapsedContent: {
    paddingHorizontal: 20,
    paddingTop: 8,
    paddingBottom: 24,
    gap: 12,
  },
  collapsedSearchBar: {
    backgroundColor: Colors.card,
    borderRadius: 12,
    padding: 18,
  },
  collapsedInputRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  collapsedInputText: {
    flex: 1,
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.text,
  },
  collapsedPlaceholder: {
    color: Colors.textSecondary,
  },
  collapsedSquare: {
    width: 8,
    height: 8,
    borderRadius: 2,
    backgroundColor: Colors.accent,
  },
  collapsedButton: {
    backgroundColor: Colors.accent,
    borderRadius: 12,
    paddingVertical: 18,
    alignItems: "center",
    justifyContent: "center",
  },
  collapsedButtonDisabled: {
    backgroundColor: Colors.border,
    opacity: 0.5,
  },
  collapsedButtonText: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.white,
  },
  collapsedButtonTextDisabled: {
    color: Colors.textSecondary,
  },
  collapsedTitle: {
    fontSize: 22,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 8,
  },
  collapsedSubtitle: {
    fontSize: 15,
    color: Colors.textSecondary,
    lineHeight: 20,
  },
  sheetContent: {
    paddingHorizontal: 20,
    paddingBottom: 40,
  },
  searchContainer: {
    backgroundColor: Colors.card,
    borderRadius: 12,
    padding: 16,
    marginTop: 8,
    marginBottom: 20,
  },
  inputRow: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  iconDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: Colors.text,
  },
  iconSquare: {
    width: 8,
    height: 8,
    borderRadius: 2,
    backgroundColor: Colors.text,
  },
  textInput: {
    flex: 1,
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.text,
    paddingVertical: 8,
  },
  inputSeparator: {
    height: 1,
    backgroundColor: Colors.border,
    marginVertical: 12,
    marginLeft: 20,
  },
  locationList: {
    marginTop: 8,
  },
  locationItem: {
    flexDirection: "row",
    alignItems: "center",
    paddingVertical: 16,
    gap: 16,
    borderBottomWidth: 1,
    borderBottomColor: Colors.border,
  },
  locationIcon: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: Colors.card,
    alignItems: "center",
    justifyContent: "center",
  },
  locationInfo: {
    flex: 1,
  },
  locationName: {
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.text,
    marginBottom: 4,
  },
  locationAddress: {
    fontSize: 14,
    color: Colors.textSecondary,
  },
  confirmButton: {
    backgroundColor: Colors.accent,
    borderRadius: 12,
    paddingVertical: 16,
    alignItems: "center",
    justifyContent: "center",
    marginTop: 20,
    marginBottom: 20,
  },
  confirmButtonDisabled: {
    backgroundColor: Colors.border,
    opacity: 0.5,
  },
  confirmButtonText: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.white,
  },
  confirmButtonTextDisabled: {
    color: Colors.textSecondary,
  },
  currentLocationButton: {
    padding: 4,
  },
  backButtonInline: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
    marginBottom: 16,
    marginTop: 8,
  },
  backText: {
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.text,
  },
  title: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 20,
  },
  optionsContainer: {
    marginBottom: 20,
  },
  rideOption: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: Colors.card,
    borderRadius: 16,
    padding: 16,
    marginBottom: 12,
    borderWidth: 2,
    borderColor: "transparent",
    gap: 16,
  },
  rideOptionSelected: {
    borderColor: Colors.accent,
    backgroundColor: Colors.surface,
  },
  rideIconContainer: {
    position: "relative",
  },
  capacityBadge: {
    position: "absolute",
    top: -4,
    right: -4,
    backgroundColor: Colors.accent,
    borderRadius: 10,
    width: 20,
    height: 20,
    alignItems: "center",
    justifyContent: "center",
  },
  capacityText: {
    fontSize: 11,
    fontWeight: "700" as const,
    color: Colors.black,
  },
  rideInfo: {
    flex: 1,
  },
  rideHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 6,
  },
  rideName: {
    fontSize: 18,
    fontWeight: "700" as const,
    color: Colors.text,
  },
  ridePrice: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.accent,
  },
  rideDetails: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
  },
  rideTime: {
    fontSize: 14,
    color: Colors.textSecondary,
  },
  rideDot: {
    fontSize: 14,
    color: Colors.textSecondary,
  },
  rideDescription: {
    fontSize: 14,
    color: Colors.textSecondary,
    flex: 1,
  },
  findingContent: {
    paddingHorizontal: 20,
    paddingVertical: 40,
    alignItems: "center",
  },
  loadingContainer: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 12,
    marginBottom: 24,
  },
  loadingDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: Colors.accent,
  },
  findingTitle: {
    fontSize: 22,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 8,
    textAlign: "center",
  },
  findingSubtitle: {
    fontSize: 15,
    color: Colors.textSecondary,
    textAlign: "center",
  },
  timerContainer: {
    backgroundColor: "#1A1A1A",
    borderRadius: 16,
    padding: 16,
    marginBottom: 16,
    marginTop: 8,
  },
  timerLabel: {
    fontSize: 14,
    color: Colors.white,
    marginBottom: 4,
  },
  timerValue: {
    fontSize: 20,
    fontWeight: "700" as const,
    color: Colors.white,
  },
  vehicleCard: {
    backgroundColor: Colors.card,
    borderRadius: 16,
    padding: 16,
    marginBottom: 16,
  },
  vehicleInfo: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
  },
  licensePlate: {
    fontSize: 18,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 4,
  },
  vehicleDetails: {
    fontSize: 14,
    color: Colors.textSecondary,
    marginBottom: 4,
  },
  autonomousLabel: {
    fontSize: 13,
    color: Colors.accent,
    fontWeight: "600" as const,
  },
  vehicleIcon: {
    width: 60,
    height: 60,
    backgroundColor: Colors.surface,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
  },
  vehicleEmoji: {
    fontSize: 32,
  },
  rideLocationInfo: {
    gap: 16,
    marginBottom: 20,
  },
  rideLocationItem: {
    flexDirection: "row",
    alignItems: "flex-start",
    gap: 12,
  },
  locationDot: {
    width: 20,
    height: 20,
    borderRadius: 10,
    backgroundColor: Colors.text,
    marginTop: 2,
  },
  locationTextContainer: {
    flex: 1,
  },
  locationLabel: {
    fontSize: 12,
    color: Colors.textSecondary,
    marginBottom: 2,
    fontWeight: "600" as const,
  },
  locationText: {
    fontSize: 15,
    color: Colors.text,
    fontWeight: "600" as const,
  },
  cancelButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 8,
    backgroundColor: Colors.surface,
    borderRadius: 12,
    paddingVertical: 16,
    borderWidth: 1,
    borderColor: Colors.error,
    marginBottom: 20,
  },
  cancelButtonText: {
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.error,
  },
});
