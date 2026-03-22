"""HTTP client for communicating with the Convex site endpoints."""

import os
from typing import Optional

import httpx

CONVEX_SITE_URL = os.environ["CONVEX_SITE_URL"]
CART_BRIDGE_SECRET = os.environ["CART_BRIDGE_SECRET"]
CART_ID = os.environ["CART_ID"]

_HEADERS = {
    "Authorization": f"Bearer {CART_BRIDGE_SECRET}",
    "Content-Type": "application/json",
}


def push_telemetry(
    lat: float,
    lon: float,
    heading: Optional[float] = None,
    battery_level: Optional[float] = None,
    status: Optional[str] = None,
    speed: Optional[float] = None,
) -> None:
    """Push current cart location, speed, and status to Convex."""
    payload: dict = {
        "cartId": CART_ID,
        "latitude": lat,
        "longitude": lon,
    }
    if heading is not None:
        payload["heading"] = heading
    if battery_level is not None:
        payload["batteryLevel"] = battery_level
    if status is not None:
        payload["status"] = status
    if speed is not None:
        payload["speed"] = speed

    resp = httpx.post(
        f"{CONVEX_SITE_URL}/api/cart/telemetry",
        json=payload,
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()


def push_route(waypoints: list) -> None:
    """Push the current route waypoints to Convex for HMI display."""
    resp = httpx.post(
        f"{CONVEX_SITE_URL}/api/cart/route",
        json={"cartId": CART_ID, "waypoints": waypoints},
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()


def poll_assignment() -> Optional[dict]:
    """
    Poll Convex for the next pending ride assignment.
    Returns the ride document if one was assigned to this cart, else None.
    """
    resp = httpx.get(
        f"{CONVEX_SITE_URL}/api/cart/assignments",
        params={"cartId": CART_ID},
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()
    return resp.json().get("ride")


def arrived_at_pickup(ride_id: str) -> None:
    """Notify Convex that the cart has reached the pickup point (sets status -> arriving)."""
    resp = httpx.post(
        f"{CONVEX_SITE_URL}/api/cart/arrived-pickup",
        json={"rideId": ride_id},
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()


def get_ride_status(ride_id: str) -> Optional[str]:
    """Fetch the current status of a ride (for polling user boarding)."""
    resp = httpx.get(
        f"{CONVEX_SITE_URL}/api/cart/ride-status",
        params={"rideId": ride_id},
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()
    return resp.json().get("status")


def complete_ride(ride_id: str) -> None:
    """Notify Convex that the current ride has been completed."""
    resp = httpx.post(
        f"{CONVEX_SITE_URL}/api/cart/ride-complete",
        json={"rideId": ride_id},
        headers=_HEADERS,
        timeout=5,
    )
    resp.raise_for_status()
