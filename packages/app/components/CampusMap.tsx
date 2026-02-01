import { CAMPUS_LOCATIONS, UCF_CENTER } from "@/constants/campus-locations";
import Colors from "@/constants/colors";
import { mapStyle } from "@/constants/mapStyle";
import { findPath, PathNode } from "@/utils/pathfinding";
import * as Location from "expo-location";
import { MapPin, Navigation } from "lucide-react-native";
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
  showRoute?: boolean;
  interactive?: boolean;
  fullScreen?: boolean;
}

export default function CampusMap({
  selectedPickup,
  selectedDropoff,
  onSelectPickup,
  onSelectDropoff,
  selectingType,
  vehiclePosition,
  showRoute = false,
  interactive = true,
  fullScreen = false,
}: CampusMapProps) {
  const mapRef = useRef<MapView>(null);
  const [userLocation, setUserLocation] = useState<{
    latitude: number;
    longitude: number;
  } | null>(null);
  const [walkingPath, setWalkingPath] = useState<PathNode[]>([]);
  const [vehiclePath, setVehiclePath] = useState<PathNode[]>([]);
  const pathOpacity = useRef(new Animated.Value(1)).current;
  const [animatedOpacity, setAnimatedOpacity] = useState(1);
  const listenerRef = useRef<string | null>(null);

  const pickupLocation = React.useMemo(() =>
    selectedPickup === "current-location"
      ? userLocation
        ? {
            id: "current-location",
            name: "Current Location",
            latitude: userLocation.latitude,
            longitude: userLocation.longitude,
          }
        : null
      : CAMPUS_LOCATIONS.find((l) => l.id === selectedPickup),
    [selectedPickup, userLocation]
  );

  const dropoffLocation = React.useMemo(() => CAMPUS_LOCATIONS.find(
    (l) => l.id === selectedDropoff
  ), [selectedDropoff]);

  useEffect(() => {
    (async () => {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status === "granted") {
        const location = await Location.getCurrentPositionAsync({});
        setUserLocation({
          latitude: location.coords.latitude,
          longitude: location.coords.longitude,
        });
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

      findPath(start, goal)
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

  useEffect(() => {
    let isCancelled = false;

    if (showRoute && pickupLocation && vehiclePosition) {
      const start: PathNode = {
        latitude: vehiclePosition.latitude,
        longitude: vehiclePosition.longitude,
      };
      const goal: PathNode = {
        latitude: pickupLocation.latitude,
        longitude: pickupLocation.longitude,
      };

      console.log(
        "Finding vehicle path from cart to pickup location using pedestrian pathfinding"
      );

      findPath(start, goal)
        .then((path) => {
          if (!isCancelled) {
            console.log("Vehicle path found with", path.length, "waypoints");
            setVehiclePath(path);
          }
        })
        .catch((error) => {
          console.error("Error finding vehicle path:", error);
          if (!isCancelled) {
            console.log("Fallback: using direct line for vehicle path");
            setVehiclePath([start, goal]);
          }
        });
    } else {
      setVehiclePath([]);
    }

    return () => {
      isCancelled = true;
    };
  }, [showRoute, pickupLocation, vehiclePosition]);

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
    <View className={fullScreen ? "flex-1 w-full h-full" : "mx-4"}>
      <View className={fullScreen ? "w-full h-full" : "w-full h-[400px] rounded-2xl overflow-hidden"}>
        <MapView
          ref={mapRef}
          style={{ flex: 1 }}
          provider={PROVIDER_DEFAULT}
          mapType={Platform.OS === "web" ? undefined : "standard"}
          customMapStyle={Platform.OS === "web" ? undefined : mapStyle}
          initialRegion={{
            latitude: UCF_CENTER.latitude,
            longitude: UCF_CENTER.longitude,
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
        >
          {showRoute && vehiclePath.length > 0 && (
            <Polyline
              coordinates={vehiclePath}
              strokeColor={`${Colors.accent}80`}
              strokeWidth={4}
              lineCap="round"
              lineJoin="round"
              lineDashPattern={[10, 10]}
            />
          )}

          {walkingPath.length > 0 && (
            <>
              <Polyline
                coordinates={walkingPath}
                strokeColor={`rgba(${parseInt(Colors.primary.slice(1, 3), 16)}, ${parseInt(Colors.primary.slice(3, 5), 16)}, ${parseInt(Colors.primary.slice(5, 7), 16)}, ${animatedOpacity})`}
                strokeWidth={6}
                lineCap="round"
                lineJoin="round"
              />
            </>
          )}

          {CAMPUS_LOCATIONS.map((location) => {
            const isPickup = selectedPickup === location.id;
            const isDropoff = selectedDropoff === location.id;
            const isVisible = showRoute ? isPickup || isDropoff : true;
            if (!isVisible) return null;

            return (
              <Marker
                key={location.id}
                coordinate={{
                  latitude: location.latitude,
                  longitude: location.longitude,
                }}
                onPress={() => handleMarkerPress(location.id)}
                title={location.name}
              >
                <View className="items-center justify-center">
                  <View
                    className="w-10 h-10 rounded-full items-center justify-center border-4 border-white/90 shadow-md"
                    style={{
                        backgroundColor: isPickup
                          ? Colors.accent
                          : isDropoff
                            ? Colors.primary
                            : "#52525b",
                      }}
                  >
                    <MapPin size={14} color="#ffffff" fill="#ffffff" />
                  </View>
                  <View
                    className="w-3 h-1 rounded-full opacity-30 -mt-1"
                    style={{
                        backgroundColor: isPickup
                          ? Colors.accent
                          : isDropoff
                            ? Colors.primary
                            : "#52525b",
                      }}
                  />
                </View>
              </Marker>
            );
          })}

          {vehiclePosition && (
            <Marker coordinate={vehiclePosition} title="Driver">
              <View className="w-[50px] h-[50px] rounded-full bg-accent/15 items-center justify-center border-4 border-accent shadow-lg shadow-accent">
                <View
                  className="w-5 h-5 rounded-full shadow-md shadow-accent"
                  style={{ backgroundColor: Colors.accent }}
                />
              </View>
            </Marker>
          )}

          {userLocation && (
            <Marker coordinate={userLocation} title="Your Location">
              <View className="w-11 h-11 rounded-full bg-accent/20 items-center justify-center border-4 border-accent/30 shadow-md shadow-accent">
                <View className="w-4 h-4 rounded-full bg-accent shadow-sm shadow-accent" />
              </View>
            </Marker>
          )}
        </MapView>

        {fullScreen && (
          <TouchableOpacity
            className="absolute bottom-[230px] right-5 w-14 h-14 rounded-full bg-surface items-center justify-center shadow-lg z-50"
            onPress={handleCenterToUser}
            activeOpacity={0.8}
          >
            <Navigation size={24} color={Colors.text} fill={Colors.accent} />
          </TouchableOpacity>
        )}
      </View>

      {!showRoute && !fullScreen && (
        <View className="flex-row justify-center gap-6 mt-4 py-3 bg-white rounded-xl shadow-sm">
          <View className="flex-row items-center gap-2">
            <View
              className="w-3 h-3 rounded-full"
              style={{ backgroundColor: Colors.accent }}
            />
            <Text className="text-xs text-textSecondary font-medium">Pickup</Text>
          </View>
          <View className="flex-row items-center gap-2">
            <View
              className="w-3 h-3 rounded-full"
              style={{ backgroundColor: Colors.primary }}
            />
            <Text className="text-xs text-textSecondary font-medium">Drop-off</Text>
          </View>
        </View>
      )}
    </View>
  );
}
