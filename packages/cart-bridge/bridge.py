"""
ACT Cart Bridge — ROS2 node that connects the cart to the Convex backend.

Responsibilities:
- Subscribes to /fix (GPS) and /odometry/global (heading) for telemetry
- Pushes telemetry to Convex every TELEMETRY_INTERVAL_S seconds
- Polls Convex for ride assignments every POLL_INTERVAL_S seconds
- When a ride is assigned, publishes the route to act_global_path_manager
  via /ui/global_route_wgs84_json
- Detects ride completion (proximity to destination) and notifies Convex

Environment variables required:
  CONVEX_SITE_URL      — e.g. https://adept-rabbit-12.convex.site
  CART_BRIDGE_SECRET   — shared secret for HTTP auth
  CART_ID              — Convex Id<"carts"> for this cart
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from convex_client import push_telemetry, poll_assignment, arrived_at_pickup, complete_ride, get_ride_status
from pathfinding import find_path

TELEMETRY_INTERVAL_S = 2.0
POLL_INTERVAL_S = 5.0
# meters — consider ride complete when within this distance
COMPLETION_DISTANCE_M = 8.0

TERMINAL_STATUSES = {"completed", "cancelled"}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return distance in meters between two WGS84 points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle in degrees (0 = East, CCW positive)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class CartBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("cart_bridge")

        # Current telemetry state
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._heading: Optional[float] = None

        # Current ride
        self._active_ride: Optional[dict] = None
        # "to_pickup" → navigating to origin; "to_dropoff" → navigating to destination
        self._phase: str = "to_pickup"
        self._notified_arrival: bool = False

        # Subscriptions
        self.create_subscription(
	    NavSatFix,
	    "/fix",
	    self._gps_callback,
	    qos_profile_sensor_data,
	)
        self.create_subscription(Odometry, "/odometry/global", self._odom_callback, 10)

        # Publisher — sends route to act_global_path_manager
        self._route_pub = self.create_publisher(
            String,
            "/ui/global_route_wgs84_json",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Timers
        self.create_timer(TELEMETRY_INTERVAL_S, self._telemetry_timer)
        self.create_timer(POLL_INTERVAL_S, self._poll_timer)

        self.get_logger().info("Cart bridge started")

    # ---- ROS2 callbacks -------------------------------------------------- #

    def _gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:  # No fix
            return
        self._lat = msg.latitude
        self._lon = msg.longitude

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._heading = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)

    # ---- Timers ---------------------------------------------------------- #

    def _telemetry_timer(self) -> None:
        if self._lat is None or self._lon is None:
            return
        try:
            push_telemetry(
                lat=self._lat,
                lon=self._lon,
                heading=self._heading,
                status="busy" if self._active_ride else "idle",
            )
        except Exception as e:
            self.get_logger().error(f"Telemetry push failed: {e}")

        # Phase-aware proximity check
        if self._active_ride and self._lat is not None:
            if self._phase == "to_pickup":
                origin = self._active_ride.get("origin", {})
                olat, olon = origin.get("latitude"), origin.get("longitude")
                if olat is not None and olon is not None:
                    dist = haversine_m(self._lat, self._lon, olat, olon)
                    if dist <= COMPLETION_DISTANCE_M and not self._notified_arrival:
                        self._notify_arrived_pickup()
            elif self._phase == "to_dropoff":
                dest = self._active_ride.get("destination", {})
                dlat, dlon = dest.get("latitude"), dest.get("longitude")
                if dlat is not None and dlon is not None:
                    dist = haversine_m(self._lat, self._lon, dlat, dlon)
                    if dist <= COMPLETION_DISTANCE_M:
                        self._finish_ride()

    def _poll_timer(self) -> None:
        # If a ride is active, check for status changes
        if self._active_ride is not None:
            try:
                status = get_ride_status(self._active_ride["_id"])
                if status in TERMINAL_STATUSES:
                    self.get_logger().info(
                        f"Ride {self._active_ride['_id']} is {status} externally — clearing"
                    )
                    self._publish_clear(self._active_ride["_id"])
                    self._active_ride = None
                    self._phase = "to_pickup"
                    self._notified_arrival = False
                elif status == "in_progress" and self._phase == "to_pickup":
                    # User has boarded — switch to phase 2: pickup → dropoff
                    self._phase = "to_dropoff"
                    self._publish_route_phase2()
            except Exception as e:
                self.get_logger().error(f"Ride status check failed: {e}")
            return

        try:
            ride = poll_assignment()
            if ride:
                self._active_ride = ride
                self._phase = "to_pickup"
                self._notified_arrival = False
                self.get_logger().info(
                    f"Ride assigned: {ride['_id']} — "
                    f"origin ({ride['origin']['latitude']}, {ride['origin']['longitude']}) "
                    f"→ dest ({ride['destination']['latitude']}, {ride['destination']['longitude']})"
                )
                self._publish_route_phase1(ride)
        except Exception as e:
            self.get_logger().error(f"Assignment poll failed: {e}")

    # ---- Ride management ------------------------------------------------- #

    def _publish_route_phase1(self, ride: dict) -> None:
        """Phase 1: cart current position → pickup (origin)."""
        if self._lat is None or self._lon is None:
            self.get_logger().warn("No GPS fix yet, cannot publish phase 1 route")
            return
        origin = ride["origin"]
        waypoints = find_path(self._lat, self._lon, origin["latitude"], origin["longitude"])
        self._publish_waypoints(ride["_id"], waypoints)
        self.get_logger().info(f"Published phase 1 route: cart → pickup ({len(waypoints)} waypoints)")

    def _publish_route_phase2(self) -> None:
        """Phase 2: pickup (origin) → dropoff (destination)."""
        ride = self._active_ride
        if not ride:
            return
        origin = ride["origin"]
        dest = ride["destination"]
        waypoints = find_path(origin["latitude"], origin["longitude"], dest["latitude"], dest["longitude"])
        self._publish_waypoints(ride["_id"], waypoints)
        self.get_logger().info(f"Published phase 2 route: pickup → dropoff ({len(waypoints)} waypoints)")

    def _publish_waypoints(self, route_id: str, waypoints: list) -> None:
        msg = String()
        msg.data = json.dumps({"route_id": route_id, "waypoints": waypoints})
        self._route_pub.publish(msg)

    def _notify_arrived_pickup(self) -> None:
        if not self._active_ride:
            return
        try:
            arrived_at_pickup(self._active_ride["_id"])
            self._notified_arrival = True
            self.get_logger().info("Notified Convex: arrived at pickup — waiting for user to board")
        except Exception as e:
            self.get_logger().error(f"Failed to notify arrived at pickup: {e}")

    def _publish_clear(self, ride_id: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "route_id": ride_id,
            "waypoints": [],
        })
        self._route_pub.publish(msg)

    def _finish_ride(self) -> None:
        if not self._active_ride:
            return
        ride_id = self._active_ride["_id"]
        try:
            complete_ride(ride_id)
            self.get_logger().info(f"Ride {ride_id} completed")
        except Exception as e:
            self.get_logger().error(f"Failed to complete ride {ride_id}: {e}")
        finally:
            self._publish_clear(ride_id)
            self._active_ride = None
            self._phase = "to_pickup"
            self._notified_arrival = False


def main() -> None:
    rclpy.init()
    node = CartBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
