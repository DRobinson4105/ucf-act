import { CAMPUS_LOCATIONS, UCF_CENTER } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { SHEET_MIN_HEIGHT } from "./SwipeableBottomSheet";
import { findPath, PathNode } from "@/utils/pathfinding";
import { useAnimatedPolyline } from "@/utils/animatePolyline";
import * as Location from "expo-location";
import { MaterialCommunityIcons } from "@expo/vector-icons";
import { Navigation } from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
  Animated,
  Platform,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import MapView, { Marker, Polyline, PROVIDER_DEFAULT } from "react-native-maps";

interface CampusMapProps {
  selectedPickup: string | null;
  selectedDropoff: string | null;
  onSelectPickup: (locationId: string) => void;
  onSelectDropoff: (locationId: string) => void;
  selectingType: "pickup" | "dropoff";
  vehiclePosition?: { latitude: number; longitude: number };
  cartId?: string; // assigned cart Id — if set, shows only this cart
  allCarts?: Array<{ _id: string; name: string; status: string; location: { latitude: number; longitude: number; heading?: number } }>; // live cart list from parent
  vehicleTarget?: { latitude: number; longitude: number }; // override route goal (e.g. destination during in_progress)
  showRoute?: boolean;
  interactive?: boolean;
  fullScreen?: boolean;
  customPickupCoord?: { latitude: number; longitude: number };
  customDropoffCoord?: { latitude: number; longitude: number };
  onMapPress?: (coord: { latitude: number; longitude: number }) => void;
  hideUserLocation?: boolean; // hide blue user dot (e.g. during in_progress — use cart as reference instead)
  convexRoute?: Array<{ latitude: number; longitude: number }>; // live route from Convex (cart bridge) — overrides local A* pathfinding
  rideStatus?: "none" | "planning" | "assigned" | "arriving" | "in_progress";
  overlay?: React.ReactNode; // absolutely-positioned elements rendered ABOVE the native MapView
}

export default function CampusMap({
  selectedPickup,
  selectedDropoff,
  onSelectPickup,
  onSelectDropoff,
  selectingType,
  vehiclePosition: vehiclePositionProp,
  cartId,
  allCarts,
  vehicleTarget,
  showRoute = false,
  interactive = true,
  fullScreen = false,
  customPickupCoord,
  customDropoffCoord,
  onMapPress,
  hideUserLocation = false,
  convexRoute,
  rideStatus = "none",
  overlay,
}: CampusMapProps) {
  const assignedCart = cartId ? allCarts?.find((c) => c._id === cartId) : null;
  const vehiclePosition = assignedCart?.location ?? vehiclePositionProp;

  // Idle carts to show on the map before a ride is assigned
  const idleCarts = cartId ? [] : (allCarts ?? []).filter((c) => c.status === "idle" && c.location);
  const mapRef = useRef<MapView>(null);
  const [userLocation, setUserLocation] = useState<{
    latitude: number;
    longitude: number;
  } | null>(null);
  const [walkingPath, setWalkingPath] = useState<PathNode[]>([]);
  const [vehiclePath, setVehiclePath] = useState<PathNode[]>([]);
  const fullVehiclePathRef = useRef<PathNode[]>([]);
  const pathOpacity = useRef(new Animated.Value(1)).current;
  const [animatedOpacity, setAnimatedOpacity] = useState(1);
  const listenerRef = useRef<string | null>(null);
  const animatedWalkingPath = useAnimatedPolyline(walkingPath, 800);
  const animatedVehiclePath = useAnimatedPolyline(vehiclePath, 600);

  const pickupLocation = React.useMemo(() => {
    if (selectedPickup === "current-location") {
      return userLocation
        ? { id: "current-location", name: "Current Location", latitude: userLocation.latitude, longitude: userLocation.longitude }
        : null;
    }
    if (selectedPickup === "custom" && customPickupCoord) {
      return { id: "custom", name: "Custom Location", ...customPickupCoord };
    }
    return CAMPUS_LOCATIONS.find((l) => l.id === selectedPickup);
  }, [selectedPickup, userLocation, customPickupCoord]);

  const dropoffLocation = React.useMemo(() => {
    if (selectedDropoff === "custom" && customDropoffCoord) {
      return { id: "custom", name: "Custom Location", ...customDropoffCoord };
    }
    return CAMPUS_LOCATIONS.find((l) => l.id === selectedDropoff);
  }, [selectedDropoff, customDropoffCoord]);

  useEffect(() => {
    (async () => {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status === "granted") {
        const location = await Location.getCurrentPositionAsync({});
        const coords = {
          latitude: location.coords.latitude,
          longitude: location.coords.longitude,
        };
        setUserLocation(coords);
        mapRef.current?.animateToRegion(
          { ...coords, latitudeDelta: 0.02, longitudeDelta: 0.02 },
          500
        );
      }
    })();
  }, []);

  useEffect(() => {
    let isCancelled = false;

    if (pickupLocation && dropoffLocation) {
      const start: PathNode = {
        latitude: pickupLocation.latitude,
        longitude: pickupLocation.longitude,
      };
      const goal: PathNode = {
        latitude: dropoffLocation.latitude,
        longitude: dropoffLocation.longitude,
      };

      console.log(
        "Finding path from",
        pickupLocation.name || "pickup",
        "to",
        dropoffLocation.name
      );

      findPath(start, goal, userLocation ?? undefined)
        .then((path) => {
          if (!isCancelled) {
            setWalkingPath(path);
            console.log("Path set with", path.length, "nodes");

            // Remove old listener if it exists
            if (listenerRef.current) {
              pathOpacity.removeListener(listenerRef.current);
            }

            // Stop any existing animation before starting a new one
            pathOpacity.stopAnimation();
            pathOpacity.setValue(1);

            // Add listener to update opacity state
            listenerRef.current = pathOpacity.addListener(({ value }) => {
              setAnimatedOpacity(value);
            });

            Animated.loop(
              Animated.sequence([
                Animated.timing(pathOpacity, {
                  toValue: 0.4,
                  duration: 1200,
                  useNativeDriver: true,
                }),
                Animated.timing(pathOpacity, {
                  toValue: 1,
                  duration: 1200,
                  useNativeDriver: true,
                }),
              ])
            ).start();
          }
        })
        .catch((error) => {
          console.error("Error finding path:", error);
          if (!isCancelled) {
            setWalkingPath([start, goal]);
          }
        });
    } else {
      setWalkingPath([]);
    }

    return () => {
      isCancelled = true;
      pathOpacity.stopAnimation();
      if (listenerRef.current) {
        pathOpacity.removeListener(listenerRef.current);
        listenerRef.current = null;
      }
    };
  }, [pickupLocation, dropoffLocation, pathOpacity]);

  // Fetch full vehicle path only when the goal changes, not on every position update
  const routeGoal = vehicleTarget ?? (pickupLocation ? { latitude: pickupLocation.latitude, longitude: pickupLocation.longitude } : null);
  const routeGoalKey = routeGoal ? `${routeGoal.latitude},${routeGoal.longitude}` : null;

  // When a live Convex route is available, sync it directly into the vehicle path
  useEffect(() => {
    if (!showRoute) return;
    if (convexRoute && convexRoute.length >= 2) {
      fullVehiclePathRef.current = convexRoute;
      setVehiclePath(convexRoute);
    } else if (convexRoute && convexRoute.length === 0) {
      fullVehiclePathRef.current = [];
      setVehiclePath([]);
    }
  }, [showRoute, convexRoute]);

  useEffect(() => {
    let isCancelled = false;

    // Skip local A* if the caller provides a live route from Convex
    if (convexRoute !== undefined) return;

    if (!showRoute || !routeGoal || !vehiclePosition) {
      fullVehiclePathRef.current = [];
      setVehiclePath([]);
      return;
    }

    const goal: PathNode = { latitude: routeGoal.latitude, longitude: routeGoal.longitude };
    const start: PathNode = { latitude: vehiclePosition.latitude, longitude: vehiclePosition.longitude };

    findPath(start, goal, userLocation ?? undefined)
      .then((path) => {
        if (!isCancelled) {
          console.log("Vehicle path fetched with", path.length, "waypoints");
          fullVehiclePathRef.current = path;
          setVehiclePath(path);
        }
      })
      .catch((error) => {
        console.error("Error finding vehicle path:", error);
        if (!isCancelled) {
          fullVehiclePathRef.current = [start, goal];
          setVehiclePath([start, goal]);
        }
      });

    return () => { isCancelled = true; };
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [showRoute, routeGoalKey]);

  // Trim the stored path from the nearest waypoint as the cart moves — no re-fetch
  useEffect(() => {
    if (!vehiclePosition || fullVehiclePathRef.current.length === 0) return;

    const { latitude: vLat, longitude: vLon } = vehiclePosition;
    let nearestIdx = 0;
    let nearestDist = Infinity;

    fullVehiclePathRef.current.forEach((pt, idx) => {
      const dLat = pt.latitude - vLat;
      const dLon = pt.longitude - vLon;
      const dist = dLat * dLat + dLon * dLon;
      if (dist < nearestDist) {
        nearestDist = dist;
        nearestIdx = idx;
      }
    });

    // Start from the waypoint AFTER the nearest one so the polyline
    // begins at the edge of the cart marker instead of under it
    const startIdx = Math.min(nearestIdx + 1, fullVehiclePathRef.current.length - 1);
    setVehiclePath(fullVehiclePathRef.current.slice(startIdx));
  }, [vehiclePosition]);

  const handleMarkerPress = (locationId: string) => {
    if (!interactive) return;
    if (selectingType === "pickup") onSelectPickup(locationId);
    else onSelectDropoff(locationId);
  };

  const handleCenterToUser = async () => {
    if (userLocation && mapRef.current) {
      mapRef.current.animateToRegion(
        {
          latitude: userLocation.latitude,
          longitude: userLocation.longitude,
          latitudeDelta: 0.01,
          longitudeDelta: 0.01,
        },
        500
      );
    } else {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status === "granted") {
        const location = await Location.getCurrentPositionAsync({});
        const coords = {
          latitude: location.coords.latitude,
          longitude: location.coords.longitude,
        };
        setUserLocation(coords);
        if (mapRef.current) {
          mapRef.current.animateToRegion(
            {
              ...coords,
              latitudeDelta: 0.01,
              longitudeDelta: 0.01,
            },
            500
          );
        }
      }
    }
  };

  return (
    <View pointerEvents="box-none" style={fullScreen ? { flex: 1, width: "100%", height: "100%" } : { marginHorizontal: 16 }}>
      <View style={fullScreen ? { width: "100%", height: "100%" } : { width: "100%", height: 400, borderRadius: 16, overflow: "hidden" }}>
        <MapView
          ref={mapRef}
          style={{ flex: 1 }}
          provider={PROVIDER_DEFAULT}
          mapType={Platform.OS === "web" ? undefined : "standard"}
          // customMapStyle={Platform.OS === "web" ? undefined : mapStyle}
          initialRegion={{
            latitude: userLocation?.latitude ?? UCF_CENTER.latitude,
            longitude: userLocation?.longitude ?? UCF_CENTER.longitude,
            latitudeDelta: 0.02,
            longitudeDelta: 0.02,
          }}
          scrollEnabled={interactive}
          zoomEnabled={interactive}
          rotateEnabled={false}
          pitchEnabled={false}
          showsUserLocation={false}
          showsMyLocationButton={false}
          showsCompass={false}
          showsPointsOfInterest={false}
          showsBuildings={false}
          onPress={onMapPress ? (e) => onMapPress(e.nativeEvent.coordinate) : undefined}
        >
          {showRoute && animatedVehiclePath.length > 0 && (
            <Polyline
              coordinates={animatedVehiclePath}
              strokeColor={Colors.accent}
              strokeWidth={3}
              lineCap="round"
              lineJoin="round"
            />
          )}

          {animatedWalkingPath.length > 0 && !vehiclePosition && (
            <Polyline
              coordinates={animatedWalkingPath}
              strokeColor={`rgba(45,212,191,${animatedOpacity})`}
              strokeWidth={3}
              lineCap="round"
              lineJoin="round"
            />
          )}

          {CAMPUS_LOCATIONS.map((location) => {
            const isPickup = selectedPickup === location.id;
            const isDropoff = selectedDropoff === location.id;
            const isSelected = isPickup || isDropoff;

            // Determine marker visual state for key — forces re-mount when state changes
            // so react-native-maps re-renders the marker view
            const markerState = isPickup ? "pickup" : isDropoff ? "dropoff" : "default";

            // When route is showing, dim unselected markers but NEVER hide them
            // Hiding causes the bug where changing one endpoint makes the other disappear
            const dimmed = showRoute && !isSelected;

            const markerColor = isPickup
              ? Colors.accent
              : isDropoff
                ? Colors.destination
                : Colors.muted;

            return (
              <Marker
                key={`${location.id}-${markerState}`}
                coordinate={{
                  latitude: location.latitude,
                  longitude: location.longitude,
                }}
                onPress={() => handleMarkerPress(location.id)}
                title={location.name}
                tracksViewChanges={false}
                opacity={dimmed ? 0.25 : 1}
              >
                {isSelected ? (
                  <View style={{ alignItems: "center" }}>
                    <View
                      style={{
                        width: 28,
                        height: 28,
                        borderRadius: 14,
                        backgroundColor: markerColor,
                        alignItems: "center",
                        justifyContent: "center",
                        borderWidth: 3,
                        borderColor: "#FFFFFF",
                        shadowColor: markerColor,
                        shadowOffset: { width: 0, height: 2 },
                        shadowOpacity: 0.5,
                        shadowRadius: 6,
                        elevation: 6,
                      }}
                    >
                      <View
                        style={{
                          width: 8,
                          height: 8,
                          borderRadius: 4,
                          backgroundColor: "#FFFFFF",
                        }}
                      />
                    </View>
                    <View
                      style={{
                        width: 0,
                        height: 0,
                        borderLeftWidth: 5,
                        borderRightWidth: 5,
                        borderTopWidth: 6,
                        borderLeftColor: "transparent",
                        borderRightColor: "transparent",
                        borderTopColor: "#FFFFFF",
                        marginTop: -1,
                      }}
                    />
                  </View>
                ) : (
                  <View
                    style={{
                      width: 10,
                      height: 10,
                      borderRadius: 5,
                      backgroundColor: Colors.muted,
                      borderWidth: 1.5,
                      borderColor: "rgba(255,255,255,0.3)",
                    }}
                  />
                )}
              </Marker>
            );
          })}

          {customPickupCoord && selectedPickup === "custom" && (
            <Marker coordinate={customPickupCoord} title="Custom Pickup" tracksViewChanges={false}>
              <View style={{ alignItems: "center" }}>
                <View
                  style={{
                    width: 28,
                    height: 28,
                    borderRadius: 14,
                    backgroundColor: Colors.accent,
                    alignItems: "center",
                    justifyContent: "center",
                    borderWidth: 3,
                    borderColor: "#FFFFFF",
                    shadowColor: Colors.accent,
                    shadowOffset: { width: 0, height: 2 },
                    shadowOpacity: 0.5,
                    shadowRadius: 6,
                    elevation: 6,
                  }}
                >
                  <View style={{ width: 8, height: 8, borderRadius: 4, backgroundColor: "#FFFFFF" }} />
                </View>
                <View
                  style={{
                    width: 0, height: 0,
                    borderLeftWidth: 5, borderRightWidth: 5, borderTopWidth: 6,
                    borderLeftColor: "transparent", borderRightColor: "transparent",
                    borderTopColor: "#FFFFFF",
                    marginTop: -1,
                  }}
                />
              </View>
            </Marker>
          )}

          {customDropoffCoord && selectedDropoff === "custom" && (
            <Marker coordinate={customDropoffCoord} title="Custom Drop-off" tracksViewChanges={false}>
              <View style={{ alignItems: "center" }}>
                <View
                  style={{
                    width: 28,
                    height: 28,
                    borderRadius: 14,
                    backgroundColor: Colors.destination,
                    alignItems: "center",
                    justifyContent: "center",
                    borderWidth: 3,
                    borderColor: "#FFFFFF",
                    shadowColor: Colors.destination,
                    shadowOffset: { width: 0, height: 2 },
                    shadowOpacity: 0.5,
                    shadowRadius: 6,
                    elevation: 6,
                  }}
                >
                  <View style={{ width: 8, height: 8, borderRadius: 4, backgroundColor: "#FFFFFF" }} />
                </View>
                <View
                  style={{
                    width: 0, height: 0,
                    borderLeftWidth: 5, borderRightWidth: 5, borderTopWidth: 6,
                    borderLeftColor: "transparent", borderRightColor: "transparent",
                    borderTopColor: "#FFFFFF",
                    marginTop: -1,
                  }}
                />
              </View>
            </Marker>
          )}

          {/* Idle carts — shown when no ride is assigned yet */}
          {(rideStatus === "none" || rideStatus === "planning") && idleCarts.map((cart) => (
            <Marker
              key={cart._id}
              coordinate={{ latitude: cart.location.latitude, longitude: cart.location.longitude }}
              title={cart.name}
            >
              <View style={{
                width: 44,
                height: 44,
                borderRadius: 22,
                backgroundColor: "rgba(26,26,26,0.9)",
                alignItems: "center",
                justifyContent: "center",
                borderWidth: 2,
                borderColor: Colors.border,
                overflow: "hidden",
              }}>
                <MaterialCommunityIcons name="golf-cart" size={20} color={Colors.accent} />
              </View>
            </Marker>
          ))}

          {/* Assigned cart — shown after a ride is matched */}
          {vehiclePosition && (
            <Marker coordinate={vehiclePosition} title="Your Cart" tracksViewChanges={false}>
              <View style={{
                width: 44,
                height: 44,
                borderRadius: 22,
                backgroundColor: "rgba(26,26,26,0.95)",
                alignItems: "center",
                justifyContent: "center",
                borderWidth: 2,
                borderColor: Colors.accent,
                overflow: "hidden",
              }}>
                <MaterialCommunityIcons name="golf-cart" size={22} color={Colors.accent} />
              </View>
            </Marker>
          )}

          {userLocation && !hideUserLocation && (
            <Marker coordinate={userLocation} title="Your Location">
              <View style={{
                width: 22,
                height: 22,
                borderRadius: 11,
                backgroundColor: "rgba(59,130,246,0.2)",
                alignItems: "center",
                justifyContent: "center",
                borderWidth: 3,
                borderColor: "rgba(59,130,246,0.4)",
              }}>
                <View style={{
                  width: 10,
                  height: 10,
                  borderRadius: 5,
                  backgroundColor: "#3B82F6",
                  shadowColor: "#3B82F6",
                  shadowOffset: { width: 0, height: 0 },
                  shadowOpacity: 0.4,
                  shadowRadius: 4,
                }} />
              </View>
            </Marker>
          )}
        </MapView>

        {fullScreen && (
          <TouchableOpacity
            style={{
              position: "absolute",
              bottom: SHEET_MIN_HEIGHT + 20,
              right: 20,
              width: 56,
              height: 56,
              borderRadius: 28,
              backgroundColor: Colors.surface,
              alignItems: "center",
              justifyContent: "center",
              shadowColor: "#000",
              shadowOffset: { width: 0, height: 2 },
              shadowOpacity: 0.4,
              shadowRadius: 8,
              elevation: 8,
              zIndex: 50,
            }}
            onPress={handleCenterToUser}
            activeOpacity={0.8}
          >
            <Navigation size={24} color={Colors.text} />
          </TouchableOpacity>
        )}

        {overlay}
      </View>

      {!showRoute && !fullScreen && (
        <View style={{
          flexDirection: "row",
          justifyContent: "center",
          gap: 24,
          marginTop: 16,
          paddingVertical: 12,
          backgroundColor: Colors.surface,
          borderRadius: 12,
          borderWidth: 1,
          borderColor: Colors.border,
        }}>
          <View style={{ flexDirection: "row", alignItems: "center", gap: 8 }}>
            <View style={{ width: 10, height: 10, borderRadius: 5, backgroundColor: Colors.accent }} />
            <Text style={{ fontSize: 12, color: Colors.textSecondary, fontWeight: "500" }}>Pickup</Text>
          </View>
          <View style={{ flexDirection: "row", alignItems: "center", gap: 8 }}>
            <View style={{ width: 10, height: 10, borderRadius: 5, backgroundColor: Colors.destination }} />
            <Text style={{ fontSize: 12, color: Colors.textSecondary, fontWeight: "500" }}>Drop-off</Text>
          </View>
        </View>
      )}
    </View>
  );
}
