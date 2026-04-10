"""
ACT Cart Simulator -- runs on any machine without ROS2.

Two-phase ride flow:
  Phase 1: Cart drives idle position -> pickup origin
           Calls /api/cart/arrived-pickup when within COMPLETION_DIST_M
           Waits until user boards (ride status becomes "in_progress")
  Phase 2: Cart drives pickup origin -> destination
           Calls /api/cart/ride-complete when within COMPLETION_DIST_M

Usage:
  cd packages/cart-bridge
  source .venv/bin/activate  (or: pip install httpx python-dotenv)
  python simulate.py
"""

import math
import time
from typing import Optional

from dotenv import load_dotenv
load_dotenv()

from convex_client import (
    push_telemetry,
    push_route,
    poll_assignment,
    arrived_at_pickup,
    get_ride_status,
    complete_ride,
    CART_ID,
)
from pathfinding import find_path

# Starting position (UCF campus area) -- used when idle
IDLE_LAT = 28.6024
IDLE_LON = -81.2001

TELEMETRY_INTERVAL = 2.0    # seconds
POLL_INTERVAL = 5.0         # seconds
BOARD_POLL_INTERVAL = 2.0   # seconds -- how often to check if user has boarded
SPEED_MPS = 4.0             # metres per second (~9 mph golf cart speed)
SPEED_MPH = SPEED_MPS * 2.23694  # converted for HMI display
COMPLETION_DIST_M = 8.0     # metres -- trigger action when within this distance


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def compute_path_distances(waypoints: list[dict]) -> list[float]:
    """Compute cumulative distances along the path (in metres)."""
    dists = [0.0]
    for i in range(1, len(waypoints)):
        d = haversine_m(
            waypoints[i - 1]["latitude"], waypoints[i - 1]["longitude"],
            waypoints[i]["latitude"], waypoints[i]["longitude"],
        )
        dists.append(dists[-1] + d)
    return dists


def interpolate_along_path(
    waypoints: list[dict],
    cum_dists: list[float],
    distance_m: float,
) -> tuple[float, float, float]:
    """
    Given a path and a distance along it, return (lat, lon, heading).
    Clamps to end of path if distance exceeds total.
    """
    total = cum_dists[-1]
    distance_m = max(0.0, min(distance_m, total))

    for i in range(1, len(cum_dists)):
        if cum_dists[i] >= distance_m:
            seg_len = cum_dists[i] - cum_dists[i - 1]
            t = (distance_m - cum_dists[i - 1]) / seg_len if seg_len > 0 else 0.0
            lat = waypoints[i - 1]["latitude"] + (waypoints[i]["latitude"] - waypoints[i - 1]["latitude"]) * t
            lon = waypoints[i - 1]["longitude"] + (waypoints[i]["longitude"] - waypoints[i - 1]["longitude"]) * t
            dlat = waypoints[i]["latitude"] - waypoints[i - 1]["latitude"]
            dlon = waypoints[i]["longitude"] - waypoints[i - 1]["longitude"]
            heading = math.degrees(math.atan2(dlon, dlat)) % 360
            return lat, lon, heading

    last = waypoints[-1]
    return last["latitude"], last["longitude"], 0.0


# Phase constants
PHASE_IDLE = "idle"
PHASE_TO_PICKUP = "to_pickup"
PHASE_WAITING_BOARD = "waiting"
PHASE_TO_DEST = "to_dest"


class CartSimulator:
    def __init__(self) -> None:
        self.lat = IDLE_LAT
        self.lon = IDLE_LON
        self.heading = 0.0
        self.speed_mph = 0.0

        self.active_ride: Optional[dict] = None
        self.phase: str = PHASE_IDLE
        self.drive_start: Optional[float] = None

        # Path-following state
        self.path: list[dict] = []
        self.path_dists: list[float] = []

        self._last_telemetry = 0.0
        self._last_poll = 0.0
        self._last_board_check = 0.0

        print(f"[SIM] Cart simulator started (CART_ID={CART_ID})")
        print(f"[SIM] Idle at ({self.lat:.5f}, {self.lon:.5f})")

    def run(self) -> None:
        while True:
            now = time.time()

            if now - self._last_telemetry >= TELEMETRY_INTERVAL:
                self._step_position(now)
                self._send_telemetry()
                self._last_telemetry = now

            if self.phase == PHASE_IDLE and now - self._last_poll >= POLL_INTERVAL:
                self._poll(now)
                self._last_poll = now

            if self.phase == PHASE_WAITING_BOARD and now - self._last_board_check >= BOARD_POLL_INTERVAL:
                self._check_boarding(now)
                self._last_board_check = now

            time.sleep(0.5)

    def _step_position(self, now: float) -> None:
        """Move along the pre-computed path at SPEED_MPS."""
        if self.phase not in (PHASE_TO_PICKUP, PHASE_TO_DEST):
            self.speed_mph = 0.0
            return
        if self.drive_start is None or not self.path:
            self.speed_mph = 0.0
            return

        elapsed = now - self.drive_start
        distance_traveled = elapsed * SPEED_MPS
        total = self.path_dists[-1] if self.path_dists else 0
        self.speed_mph = SPEED_MPH if distance_traveled < total else 0.0
        self.lat, self.lon, self.heading = interpolate_along_path(
            self.path, self.path_dists, distance_traveled
        )

    def _send_telemetry(self) -> None:
        status = "idle" if self.phase == PHASE_IDLE else "busy"
        try:
            push_telemetry(
                lat=self.lat,
                lon=self.lon,
                heading=self.heading,
                battery_level=95.0,
                status=status,
                speed=self.speed_mph,
            )
            print(f"[SIM] Telemetry ({self.phase}) -> ({self.lat:.5f}, {self.lon:.5f}) hdg={self.heading:.0f}° spd={self.speed_mph:.1f}mph")
        except Exception as e:
            print(f"[SIM] Telemetry failed: {e}")

        if self.phase == PHASE_TO_PICKUP:
            origin = self.active_ride["origin"]
            dist = haversine_m(self.lat, self.lon, origin["latitude"], origin["longitude"])
            print(f"[SIM] Distance to pickup: {dist:.1f}m")
            if dist <= COMPLETION_DIST_M:
                self._arrive_at_pickup()
            else:
                self._check_cancellation()

        elif self.phase == PHASE_TO_DEST:
            dest = self.active_ride["destination"]
            dist = haversine_m(self.lat, self.lon, dest["latitude"], dest["longitude"])
            print(f"[SIM] Distance to destination: {dist:.1f}m")
            if dist <= COMPLETION_DIST_M:
                self._finish_ride()
            else:
                self._check_cancellation()

    def _poll(self, now: float) -> None:
        try:
            ride = poll_assignment()
            if ride:
                self.active_ride = ride
                self.phase = PHASE_TO_PICKUP
                self.drive_start = now
                origin = ride["origin"]

                print(f"[SIM] Ride assigned: {ride['_id']}")
                print(f"[SIM]   Pickup:      ({origin['latitude']:.5f}, {origin['longitude']:.5f})")
                print(f"[SIM]   Destination: ({ride['destination']['latitude']:.5f}, {ride['destination']['longitude']:.5f})")

                # Compute path using campus graph
                self.path = find_path(self.lat, self.lon, origin["latitude"], origin["longitude"])
                self.path_dists = compute_path_distances(self.path)
                total_m = self.path_dists[-1] if self.path_dists else 0
                eta_s = total_m / SPEED_MPS if SPEED_MPS > 0 else 0

                print(f"[SIM] Phase 1: driving to pickup ({len(self.path)} waypoints, {total_m:.0f}m, ~{eta_s:.0f}s)")

                try:
                    push_route(self.path)
                    print(f"[SIM] Route pushed: cart -> pickup")
                except Exception as e:
                    print(f"[SIM] Route push failed: {e}")
            else:
                print("[SIM] No pending rides.")
        except Exception as e:
            print(f"[SIM] Poll failed: {e}")

    def _arrive_at_pickup(self) -> None:
        ride_id = self.active_ride["_id"]
        origin = self.active_ride["origin"]
        self.lat = origin["latitude"]
        self.lon = origin["longitude"]
        self.speed_mph = 0.0
        try:
            arrived_at_pickup(ride_id)
            print(f"[SIM] Arrived at pickup! Notified server. Waiting for user to board...")
            self.phase = PHASE_WAITING_BOARD
            self._last_board_check = time.time()
        except Exception as e:
            print(f"[SIM] arrived_at_pickup failed: {e}")

    def _check_cancellation(self) -> None:
        ride_id = self.active_ride["_id"]
        try:
            status = get_ride_status(ride_id)
            if status in ("cancelled", "completed", None):
                print(f"[SIM] Ride {ride_id} was {status} -- aborting and returning to idle.")
                self._abort_ride()
        except Exception as e:
            print(f"[SIM] Cancellation check failed: {e}")

    def _abort_ride(self) -> None:
        try:
            push_route([])
        except Exception:
            pass
        self.active_ride = None
        self.phase = PHASE_IDLE
        self.drive_start = None
        self.path = []
        self.path_dists = []
        self.lat = IDLE_LAT
        self.lon = IDLE_LON
        print("[SIM] Ride aborted. Back to idle.")

    def _check_boarding(self, now: float) -> None:
        ride_id = self.active_ride["_id"]
        try:
            status = get_ride_status(ride_id)
            print(f"[SIM] Waiting for board... ride status={status}")
            if status == "cancelled":
                print(f"[SIM] Ride cancelled while waiting at pickup.")
                self._abort_ride()
                return
            if status == "in_progress":
                print(f"[SIM] User boarded! Phase 2: driving to destination...")
                self.phase = PHASE_TO_DEST
                self.drive_start = now
                dest = self.active_ride["destination"]

                # Compute path from current position to destination
                self.path = find_path(self.lat, self.lon, dest["latitude"], dest["longitude"])
                self.path_dists = compute_path_distances(self.path)
                total_m = self.path_dists[-1] if self.path_dists else 0
                eta_s = total_m / SPEED_MPS if SPEED_MPS > 0 else 0

                print(f"[SIM] Phase 2: ({len(self.path)} waypoints, {total_m:.0f}m, ~{eta_s:.0f}s)")

                try:
                    push_route(self.path)
                    print(f"[SIM] Route pushed: pickup -> dropoff")
                except Exception as e:
                    print(f"[SIM] Route push failed: {e}")
        except Exception as e:
            print(f"[SIM] Board check failed: {e}")

    def _finish_ride(self) -> None:
        ride_id = self.active_ride["_id"]
        dest = self.active_ride["destination"]
        self.lat = dest["latitude"]
        self.lon = dest["longitude"]
        self.speed_mph = 0.0
        try:
            complete_ride(ride_id)
            print(f"[SIM] Ride {ride_id} completed!")
        except Exception as e:
            print(f"[SIM] Complete ride failed: {e}")
        try:
            push_route([])
        except Exception:
            pass
        finally:
            self.active_ride = None
            self.phase = PHASE_IDLE
            self.drive_start = None
            self.path = []
            self.path_dists = []
            self.lat = IDLE_LAT
            self.lon = IDLE_LON
            print("[SIM] Back to idle.")


if __name__ == "__main__":
    CartSimulator().run()
