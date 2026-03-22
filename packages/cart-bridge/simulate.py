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

# Starting position (UCF campus area) -- used when idle
IDLE_LAT = 28.6024
IDLE_LON = -81.2001

TELEMETRY_INTERVAL = 2.0    # seconds
POLL_INTERVAL = 5.0         # seconds
BOARD_POLL_INTERVAL = 2.0   # seconds -- how often to check if user has boarded
DRIVE_DURATION = 30.0       # seconds to travel between any two points
COMPLETION_DIST_M = 8.0     # metres -- trigger action when within this distance


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def make_waypoints(lat1: float, lon1: float, lat2: float, lon2: float, n: int = 20) -> list:
    return [
        {"latitude": lerp(lat1, lat2, i / (n - 1)), "longitude": lerp(lon1, lon2, i / (n - 1))}
        for i in range(n)
    ]


# Phase constants
PHASE_IDLE = "idle"
PHASE_TO_PICKUP = "to_pickup"      # driving idle -> origin
PHASE_WAITING_BOARD = "waiting"    # at pickup, waiting for user to board
PHASE_TO_DEST = "to_dest"          # driving origin -> destination


class CartSimulator:
    def __init__(self) -> None:
        self.lat = IDLE_LAT
        self.lon = IDLE_LON
        self.heading = 0.0

        self.active_ride: Optional[dict] = None
        self.phase: str = PHASE_IDLE
        self.drive_start: Optional[float] = None
        self.phase2_start_lat: float = IDLE_LAT
        self.phase2_start_lon: float = IDLE_LON

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

            # Poll for new assignments only when idle
            if self.phase == PHASE_IDLE and now - self._last_poll >= POLL_INTERVAL:
                self._poll(now)
                self._last_poll = now

            # Poll for user boarding while waiting at pickup
            if self.phase == PHASE_WAITING_BOARD and now - self._last_board_check >= BOARD_POLL_INTERVAL:
                self._check_boarding(now)
                self._last_board_check = now

            time.sleep(0.5)

    def _step_position(self, now: float) -> None:
        """Interpolate position based on current phase."""
        if self.phase == PHASE_TO_PICKUP:
            if self.drive_start is None:
                return
            elapsed = now - self.drive_start
            t = min(elapsed / DRIVE_DURATION, 1.0)
            origin = self.active_ride["origin"]
            self.lat = lerp(IDLE_LAT, origin["latitude"], t)
            self.lon = lerp(IDLE_LON, origin["longitude"], t)
            dlat = origin["latitude"] - IDLE_LAT
            dlon = origin["longitude"] - IDLE_LON
            self.heading = math.degrees(math.atan2(dlon, dlat)) % 360

        elif self.phase == PHASE_TO_DEST:
            if self.drive_start is None:
                return
            elapsed = now - self.drive_start
            t = min(elapsed / DRIVE_DURATION, 1.0)
            origin = self.active_ride["origin"]
            dest = self.active_ride["destination"]
            self.lat = lerp(self.phase2_start_lat, dest["latitude"], t)
            self.lon = lerp(self.phase2_start_lon, dest["longitude"], t)
            dlat = dest["latitude"] - origin["latitude"]
            dlon = dest["longitude"] - origin["longitude"]
            self.heading = math.degrees(math.atan2(dlon, dlat)) % 360

    def _send_telemetry(self) -> None:
        status = "idle" if self.phase == PHASE_IDLE else "busy"
        try:
            push_telemetry(
                lat=self.lat,
                lon=self.lon,
                heading=self.heading,
                battery_level=95.0,
                status=status,
            )
            print(f"[SIM] Telemetry ({self.phase}) -> ({self.lat:.5f}, {self.lon:.5f}) hdg={self.heading:.0f}")
        except Exception as e:
            print(f"[SIM] Telemetry failed: {e}")

        if self.phase == PHASE_TO_PICKUP:
            origin = self.active_ride["origin"]
            dist = haversine_m(self.lat, self.lon, origin["latitude"], origin["longitude"])
            print(f"[SIM] Distance to pickup: {dist:.1f}m")
            if dist <= COMPLETION_DIST_M:
                self._arrive_at_pickup()

        elif self.phase == PHASE_TO_DEST:
            dest = self.active_ride["destination"]
            dist = haversine_m(self.lat, self.lon, dest["latitude"], dest["longitude"])
            print(f"[SIM] Distance to destination: {dist:.1f}m")
            if dist <= COMPLETION_DIST_M:
                self._finish_ride()

    def _poll(self, now: float) -> None:
        try:
            ride = poll_assignment()
            if ride:
                self.active_ride = ride
                self.phase = PHASE_TO_PICKUP
                self.drive_start = now
                print(f"[SIM] Ride assigned: {ride['_id']}")
                print(f"[SIM]   Pickup:      ({ride['origin']['latitude']:.5f}, {ride['origin']['longitude']:.5f})")
                print(f"[SIM]   Destination: ({ride['destination']['latitude']:.5f}, {ride['destination']['longitude']:.5f})")
                print(f"[SIM] Phase 1: driving to pickup...")
                try:
                    wps = make_waypoints(self.lat, self.lon, ride["origin"]["latitude"], ride["origin"]["longitude"])
                    push_route(wps)
                    print(f"[SIM] Route pushed: cart → pickup ({len(wps)} waypoints)")
                except Exception as e:
                    print(f"[SIM] Route push failed: {e}")
            else:
                print("[SIM] No pending rides.")
        except Exception as e:
            print(f"[SIM] Poll failed: {e}")

    def _arrive_at_pickup(self) -> None:
        """Cart has reached pickup. Notify server and wait for user to board."""
        ride_id = self.active_ride["_id"]
        origin = self.active_ride["origin"]
        # Snap to exact pickup coords
        self.lat = origin["latitude"]
        self.lon = origin["longitude"]
        try:
            arrived_at_pickup(ride_id)
            print(f"[SIM] Arrived at pickup! Notified server. Waiting for user to board...")
            self.phase = PHASE_WAITING_BOARD
            self._last_board_check = time.time()
        except Exception as e:
            print(f"[SIM] arrived_at_pickup failed: {e}")

    def _check_boarding(self, now: float) -> None:
        """Poll ride status until user boards (status -> in_progress)."""
        ride_id = self.active_ride["_id"]
        try:
            status = get_ride_status(ride_id)
            print(f"[SIM] Waiting for board... ride status={status}")
            if status == "in_progress":
                print(f"[SIM] User boarded! Phase 2: driving to destination...")
                self.phase = PHASE_TO_DEST
                self.phase2_start_lat = self.lat
                self.phase2_start_lon = self.lon
                self.drive_start = now
                try:
                    dest = self.active_ride["destination"]
                    wps = make_waypoints(self.lat, self.lon, dest["latitude"], dest["longitude"])
                    push_route(wps)
                    print(f"[SIM] Route pushed: pickup → dropoff ({len(wps)} waypoints)")
                except Exception as e:
                    print(f"[SIM] Route push failed: {e}")
        except Exception as e:
            print(f"[SIM] Board check failed: {e}")

    def _finish_ride(self) -> None:
        ride_id = self.active_ride["_id"]
        dest = self.active_ride["destination"]
        # Snap to exact destination coords
        self.lat = dest["latitude"]
        self.lon = dest["longitude"]
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
            self.lat = IDLE_LAT
            self.lon = IDLE_LON
            print("[SIM] Back to idle.")


if __name__ == "__main__":
    CartSimulator().run()
