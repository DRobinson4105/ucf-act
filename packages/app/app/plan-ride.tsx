import CampusMap from "@/components/CampusMap";
import SwipeableBottomSheet from "@/components/SwipeableBottomSheet";
import { CAMPUS_LOCATIONS } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { useRide } from "@/contexts/RideContext";
import { api } from "@/convex/_generated/api";
import { router, useLocalSearchParams } from "expo-router";
import * as Location from "expo-location";
import { MaterialCommunityIcons } from "@expo/vector-icons";
import {
    ArrowLeft,
    Clock,
    MapPin,
    Navigation,
    Search,
    X,
} from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
    Animated,
    Dimensions,
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
import { useQuery } from "convex/react";

const { height: SCREEN_HEIGHT } = Dimensions.get("window");

type SelectionMode = "pickup" | "dropoff" | null;
type ViewMode = "planning" | "choosing" | "finding" | "tracking" | "review";

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
  pickupLocation: { name: string } | null;
  dropoffLocation: { name: string } | null | undefined;
}

function RideTrackingContent({
  pickupLocation,
  dropoffLocation,
}: RideTrackingContentProps) {
  const { currentRide, cancelRide, boardRide } = useRide();

  const getStatusInfo = () => {
    switch (currentRide?.status) {
      case "requested":
        return { title: "Finding your cart…", subtitle: "Looking for an available ACT cart nearby" };
      case "assigned":
        return { title: "Cart on the way", subtitle: "Your ACT cart is heading to your pickup" };
      case "arriving":
        return { title: "Cart has arrived!", subtitle: "Head to your pickup and tap when you're there" };
      case "in_progress":
        return { title: "On your way", subtitle: "Sit back and enjoy your autonomous ride" };
      default:
        return { title: "Ride status", subtitle: "" };
    }
  };

  const handleCancelRide = () => {
    cancelRide();
  };

  const statusInfo = getStatusInfo();
  const isArriving = currentRide?.status === "arriving";
  const canCancel = !!currentRide &&
    ["requested", "assigned", "arriving", "in_progress"].includes(currentRide.status);

  return (
    <ScrollView style={styles.sheetContent} showsVerticalScrollIndicator={false}>
      <View style={styles.statusCard}>
        <Text style={styles.statusTitle}>{statusInfo.title}</Text>
        <Text style={styles.statusSubtitle}>{statusInfo.subtitle}</Text>
      </View>

      {isArriving && (
        <TouchableOpacity style={styles.boardButton} onPress={boardRide} activeOpacity={0.8}>
          <Text style={styles.boardButtonText}>I'm Here</Text>
        </TouchableOpacity>
      )}

      <View style={styles.vehicleCard}>
        <View style={styles.vehicleInfo}>
          <View>
            <Text style={styles.licensePlate}>ACT-001</Text>
            <Text style={styles.vehicleDetails}>ACT Golf Cart • White</Text>
            <Text style={styles.autonomousLabel}>Autonomous · Free</Text>
          </View>
          <View style={styles.vehicleIcon}>
            <MaterialCommunityIcons name="golf-cart" size={28} color={Colors.accent} />
          </View>
        </View>
      </View>

      <View style={styles.rideLocationInfo}>
        <View style={styles.rideLocationItem}>
          <View style={styles.locationDot} />
          <View style={styles.locationTextContainer}>
            <Text style={styles.locationLabel}>Pickup</Text>
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

      {canCancel && (
        <TouchableOpacity style={styles.cancelButton} onPress={handleCancelRide}>
          <X size={20} color={Colors.error} />
          <Text style={styles.cancelButtonText}>Cancel Ride</Text>
        </TouchableOpacity>
      )}
    </ScrollView>
  );
}


function ThreeDotsLoader() {
  const anim0 = useRef(new Animated.Value(0.3)).current;
  const anim1 = useRef(new Animated.Value(0.3)).current;
  const anim2 = useRef(new Animated.Value(0.3)).current;

  useEffect(() => {
    const makeAnim = (anim: Animated.Value, delay: number) =>
      Animated.loop(
        Animated.sequence([
          Animated.delay(delay),
          Animated.timing(anim, { toValue: 1, duration: 380, useNativeDriver: true }),
          Animated.timing(anim, { toValue: 0.3, duration: 380, useNativeDriver: true }),
          Animated.delay(760 - delay),
        ])
      );
    const a0 = makeAnim(anim0, 0);
    const a1 = makeAnim(anim1, 190);
    const a2 = makeAnim(anim2, 380);
    a0.start(); a1.start(); a2.start();
    return () => { a0.stop(); a1.stop(); a2.stop(); };
  }, [anim0, anim1, anim2]);

  return (
    <View style={{ flexDirection: "row", gap: 10, alignItems: "center", justifyContent: "center" }}>
      {[anim0, anim1, anim2].map((anim, i) => (
        <Animated.View
          key={i}
          style={{ width: 11, height: 11, borderRadius: 6, backgroundColor: Colors.accent, opacity: anim }}
        />
      ))}
    </View>
  );
}

export default function PlanRideScreen() {
  const insets = useSafeAreaInsets();
  const params = useLocalSearchParams();
  const { requestRide, currentRide } = useRide();
  const allCarts = useQuery(api.carts.getAll, {});
  const [pickupLocation, setPickupLocation] =
    useState<string>("Current Location");
  const [dropoffLocation, setDropoffLocation] = useState<string>("");
  const [selectedPickupId, setSelectedPickupId] = useState<string | null>(
    "current-location"
  );
  const [selectedDropoffId, setSelectedDropoffId] = useState<string | null>(
    null
  );
  const [customPickupCoord, setCustomPickupCoord] = useState<{ latitude: number; longitude: number } | undefined>();
  const [customDropoffCoord, setCustomDropoffCoord] = useState<{ latitude: number; longitude: number } | undefined>();
  const [userLocation, setUserLocation] = useState<{ latitude: number; longitude: number } | null>(null);

  useEffect(() => {
    (async () => {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status === "granted") {
        const loc = await Location.getCurrentPositionAsync({});
        setUserLocation({ latitude: loc.coords.latitude, longitude: loc.coords.longitude });
      }
    })();
  }, []);
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
    if (!selectedPickupId || !selectedDropoffId) return;

    const resolvePickup = () => {
      if (selectedPickupId === "custom" && customPickupCoord) {
        return { ...customPickupCoord, name: "Dropped Pin" };
      }
      if (selectedPickupId === "current-location" && userLocation) {
        return { ...userLocation, name: "Current Location" };
      }
      const loc = CAMPUS_LOCATIONS.find((l) => l.id === selectedPickupId);
      return loc ? { latitude: loc.latitude, longitude: loc.longitude, name: loc.name } : null;
    };

    const resolveDropoff = () => {
      if (selectedDropoffId === "custom" && customDropoffCoord) {
        return { ...customDropoffCoord, name: "Dropped Pin" };
      }
      const loc = CAMPUS_LOCATIONS.find((l) => l.id === selectedDropoffId);
      return loc ? { latitude: loc.latitude, longitude: loc.longitude, name: loc.name } : null;
    };

    const pickup = resolvePickup();
    const dropoff = resolveDropoff();
    if (!pickup || !dropoff) return;

    setViewMode("finding");
    requestRide(pickup, dropoff);
  };

  const handleBackFromChoosing = () => {
    setViewMode("planning");
  };

  const prevRideIdRef = useRef<string | undefined>(undefined);
  const prevRideStatusRef = useRef<string | undefined>(undefined);

  // Sync viewMode with ride lifecycle:
  // - Ride appears while on planning/finding → switch to tracking
  // - Ride disappears after in_progress (completed) → dismiss screen
  // - Ride disappears otherwise (cancelled) → back to planning
  const { rideHistory } = useRide();
  useEffect(() => {
    if (currentRide) {
      prevRideIdRef.current = currentRide.id;
      prevRideStatusRef.current = currentRide.status;
      if (viewMode === "planning" || viewMode === "finding") {
        setViewMode("tracking");
      }
      return;
    }

    // currentRide is null — ride ended
    if (prevRideIdRef.current) {
      const wasInProgress = prevRideStatusRef.current === "in_progress";
      prevRideIdRef.current = undefined;
      prevRideStatusRef.current = undefined;

      if (wasInProgress) {
        // Completed ride → dismiss to home (review modal handles itself)
        router.dismiss();
      } else if (viewMode === "tracking" || viewMode === "finding") {
        // Cancelled ride → back to planning
        setViewMode("planning");
      }
    }
  }, [viewMode, currentRide, rideHistory]);

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

  const handleMapTap = (coord: { latitude: number; longitude: number }) => {
    if (isExpanded || viewMode !== "planning") return;
    if (selectionMode === "pickup") {
      setCustomPickupCoord(coord);
      setSelectedPickupId("custom");
      setPickupLocation("Dropped Pin");
      setPickupSearchQuery("");
      setPreviousPickupLocation("Dropped Pin");
      setSelectionMode(null);
    } else if (selectionMode === "dropoff") {
      setCustomDropoffCoord(coord);
      setSelectedDropoffId("custom");
      setDropoffLocation("Dropped Pin");
      setDropoffSearchQuery("");
      setPreviousDropoffLocation("Dropped Pin");
      setSelectionMode(null);
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

  const isButtonDisabled = (() => {
    if (!selectedPickupId || !selectedDropoffId) return true;
    if (selectedPickupId === "custom" && !customPickupCoord) return true;
    if (selectedDropoffId === "custom" && !customDropoffCoord) return true;
    // Allow both "custom" as long as their coords differ
    if (selectedPickupId === selectedDropoffId && selectedPickupId !== "custom") return true;
    return false;
  })();

  const pickupLocationObj = React.useMemo(() => {
    if (selectedPickupId === "current-location") return null;
    if (selectedPickupId === "custom" && customPickupCoord) {
      return { id: "custom", name: "Dropped Pin", shortName: "Pin", type: "academic" as const, ...customPickupCoord };
    }
    return CAMPUS_LOCATIONS.find((l) => l.id === selectedPickupId) || null;
  }, [selectedPickupId, customPickupCoord]);

  const dropoffLocationObj = React.useMemo(() => {
    if (selectedDropoffId === "custom" && customDropoffCoord) {
      return { id: "custom", name: "Dropped Pin", shortName: "Pin", type: "academic" as const, ...customDropoffCoord };
    }
    return CAMPUS_LOCATIONS.find((l) => l.id === selectedDropoffId) || null;
  }, [selectedDropoffId, customDropoffCoord]);

  const vehicleTarget = currentRide?.status === "in_progress" && dropoffLocationObj
    ? { latitude: dropoffLocationObj.latitude, longitude: dropoffLocationObj.longitude }
    : undefined;

  const assignedCart = currentRide?.cartId
    ? allCarts?.find((c) => c._id === currentRide.cartId)
    : undefined;
  const convexRoute = (assignedCart as any)?.currentRoute as Array<{ latitude: number; longitude: number }> | undefined;

  return (
    <View style={styles.container}>
      <CampusMap
        selectedPickup={selectedPickupId}
        selectedDropoff={selectedDropoffId}
        onSelectPickup={handleMapPress}
        onSelectDropoff={handleMapPress}
        selectingType={selectionMode || "pickup"}
        cartId={currentRide?.cartId}
        vehicleTarget={vehicleTarget}
        showRoute={
          viewMode === "tracking" ||
          (pickupLocationObj !== null && dropoffLocationObj !== null)
        }
        fullScreen
        interactive={true}
        customPickupCoord={customPickupCoord}
        customDropoffCoord={customDropoffCoord}
        onMapPress={viewMode === "planning" && !isExpanded ? handleMapTap : undefined}
        hideUserLocation={currentRide?.status === "in_progress"}
        allCarts={allCarts ?? []}
        convexRoute={convexRoute}
        rideStatus={
          viewMode === "tracking"
            ? (currentRide?.status === "in_progress"
                ? "in_progress"
                : currentRide?.status === "arriving"
                  ? "arriving"
                  : "assigned")
            : pickupLocationObj && dropoffLocationObj
              ? "planning"
              : "none"
        }
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
        maxHeight={viewMode === "choosing" ? SCREEN_HEIGHT * 0.5 : undefined}
      >
        {viewMode === "finding" ? (
          <View style={styles.findingContent}>
            <View style={styles.loadingContainer}>
              <ThreeDotsLoader />
            </View>
            <Text style={styles.findingTitle}>Finding your cart…</Text>
            <Text style={styles.findingSubtitle}>
              This will only take a moment
            </Text>
          </View>
        ) : viewMode === "tracking" ? (
          <RideTrackingContent
            pickupLocation={pickupLocationObj}
            dropoffLocation={dropoffLocationObj}
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
                    <MaterialCommunityIcons name="golf-cart" size={32} color={Colors.accent} />
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
            <Text style={styles.dropPinHint}>
              Or tap anywhere on the map to drop a pin
            </Text>
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
                  placeholderTextColor={Colors.textSecondary}
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
                  <Navigation size={20} color={Colors.textSecondary} />
                </TouchableOpacity>
              </View>

              <View style={styles.inputSeparator} />

              <View style={styles.inputRow}>
                <View style={styles.iconSquare} />
                <TextInput
                  ref={destinationInputRef}
                  style={styles.textInput}
                  placeholder="Where to?"
                  placeholderTextColor={Colors.textSecondary}
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
                <Search size={20} color={Colors.textSecondary} />
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
                        <Clock size={20} color={Colors.textSecondary} />
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
    paddingTop: 4,
    paddingBottom: 16,
    gap: 8,
  },
  collapsedSearchBar: {
    backgroundColor: Colors.card,
    borderRadius: 12,
    padding: 14,
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
  dropPinHint: {
    fontSize: 12,
    color: Colors.textSecondary,
    textAlign: "center",
  },
  collapsedButton: {
    backgroundColor: Colors.accent,
    borderRadius: 14,
    paddingVertical: 16,
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
    color: Colors.black,
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
    paddingBottom: 16,
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
    borderRadius: 14,
    paddingVertical: 16,
    alignItems: "center",
    justifyContent: "center",
    marginTop: 8,
    marginBottom: 12,
  },
  confirmButtonDisabled: {
    backgroundColor: Colors.border,
    opacity: 0.5,
  },
  confirmButtonText: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.black,
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
    marginBottom: 8,
    marginTop: 4,
  },
  backText: {
    fontSize: 16,
    fontWeight: "600" as const,
    color: Colors.text,
  },
  title: {
    fontSize: 20,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 12,
  },
  optionsContainer: {
    marginBottom: 8,
  },
  rideOption: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: Colors.card,
    borderRadius: 16,
    padding: 12,
    marginBottom: 8,
    borderWidth: 2,
    borderColor: "transparent",
    gap: 12,
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
    backgroundColor: Colors.surface,
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
    backgroundColor: "transparent",
    borderRadius: 12,
    paddingVertical: 14,
    borderWidth: 1,
    borderColor: "rgba(239,68,68,0.35)",
    marginBottom: 20,
  },
  cancelButtonText: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: Colors.error,
  },
  statusCard: {
    backgroundColor: Colors.card,
    borderRadius: 16,
    padding: 16,
    marginBottom: 12,
    marginTop: 8,
  },
  statusTitle: {
    fontSize: 18,
    fontWeight: "700" as const,
    color: Colors.text,
    marginBottom: 4,
  },
  statusSubtitle: {
    fontSize: 14,
    color: Colors.textSecondary,
  },
  boardButton: {
    backgroundColor: Colors.accent,
    borderRadius: 14,
    paddingVertical: 16,
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 12,
  },
  boardButtonText: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: Colors.black,
  },
});

