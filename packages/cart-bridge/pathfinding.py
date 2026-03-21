"""
OSM-based pathfinding for the cart bridge.

Uses the Overpass API to fetch drivable ways around UCF campus and runs
A* to return a full list of WGS84 waypoints between two points.
"""

import heapq
import math
import time
from typing import Optional

import httpx

# UCF campus center
UCF_CENTER_LAT = 28.6024
UCF_CENTER_LON = -81.2001
MAX_CAMPUS_DISTANCE_M = 2500.0

OVERPASS_SERVERS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.openstreetmap.ru/api/interpreter",
]

OVERPASS_QUERY_TEMPLATE = """
[out:json][timeout:30];
(
  way["highway"]({minLat},{minLon},{maxLat},{maxLon});
);
out body;
>;
out skel qt;
"""

# Simple in-process cache: (graph_nodes, graph_edges, timestamp)
_cache: Optional[tuple[dict, dict, float]] = None
_CACHE_TTL_S = 300.0  # 5 minutes


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _within_campus(lat: float, lon: float) -> bool:
    return _haversine_m(lat, lon, UCF_CENTER_LAT, UCF_CENTER_LON) <= MAX_CAMPUS_DISTANCE_M


def _fetch_graph(min_lat: float, min_lon: float, max_lat: float, max_lon: float) -> tuple[dict, dict]:
    """Fetch OSM ways from Overpass and build adjacency graph. Returns (nodes, edges)."""
    global _cache

    now = time.monotonic()
    if _cache is not None and now - _cache[2] < _CACHE_TTL_S:
        return _cache[0], _cache[1]

    query = OVERPASS_QUERY_TEMPLATE.format(
        minLat=min_lat, minLon=min_lon, maxLat=max_lat, maxLon=max_lon
    )

    last_error: Optional[Exception] = None
    for server in OVERPASS_SERVERS:
        try:
            resp = httpx.post(server, content=query.encode(), timeout=35)
            resp.raise_for_status()
            data = resp.json()
            break
        except Exception as e:
            last_error = e
    else:
        raise RuntimeError(f"All Overpass servers failed: {last_error}")

    raw_nodes: dict[str, tuple[float, float]] = {}  # id → (lat, lon)
    ways: list[list[str]] = []

    for el in data.get("elements", []):
        if el["type"] == "node":
            raw_nodes[str(el["id"])] = (el["lat"], el["lon"])
        elif el["type"] == "way":
            ways.append([str(n) for n in el.get("nodes", [])])

    # Filter to campus bounds
    nodes: dict[str, tuple[float, float]] = {
        nid: coords
        for nid, coords in raw_nodes.items()
        if _within_campus(coords[0], coords[1])
    }

    edges: dict[str, list[str]] = {}
    for way_nodes in ways:
        for i in range(len(way_nodes) - 1):
            a, b = way_nodes[i], way_nodes[i + 1]
            if a not in nodes or b not in nodes:
                continue
            edges.setdefault(a, []).append(b)
            edges.setdefault(b, []).append(a)

    _cache = (nodes, edges, now)
    return nodes, edges


def _nearest_node(nodes: dict[str, tuple[float, float]], lat: float, lon: float) -> Optional[str]:
    best_id: Optional[str] = None
    best_dist = float("inf")
    for nid, (nlat, nlon) in nodes.items():
        d = _haversine_m(lat, lon, nlat, nlon)
        if d < best_dist:
            best_dist = d
            best_id = nid
    return best_id


def _astar(
    nodes: dict[str, tuple[float, float]],
    edges: dict[str, list[str]],
    start_id: str,
    goal_id: str,
) -> list[str]:
    goal_lat, goal_lon = nodes[goal_id]

    # (f, g, node_id, parent_id)
    open_heap: list[tuple[float, float, str, Optional[str]]] = []
    g_scores: dict[str, float] = {start_id: 0.0}
    came_from: dict[str, Optional[str]] = {}

    start_lat, start_lon = nodes[start_id]
    h0 = _haversine_m(start_lat, start_lon, goal_lat, goal_lon)
    heapq.heappush(open_heap, (h0, 0.0, start_id, None))
    closed: set[str] = set()

    while open_heap:
        f, g, current, parent = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)
        came_from[current] = parent

        if current == goal_id:
            # Reconstruct path
            path: list[str] = []
            node: Optional[str] = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            return path

        cur_lat, cur_lon = nodes[current]
        for neighbor in edges.get(current, []):
            if neighbor in closed or neighbor not in nodes:
                continue
            nb_lat, nb_lon = nodes[neighbor]
            tentative_g = g + _haversine_m(cur_lat, cur_lon, nb_lat, nb_lon)
            if tentative_g < g_scores.get(neighbor, float("inf")):
                g_scores[neighbor] = tentative_g
                h = _haversine_m(nb_lat, nb_lon, goal_lat, goal_lon)
                heapq.heappush(open_heap, (tentative_g + h, tentative_g, neighbor, current))

    return []


def find_path(
    start_lat: float, start_lon: float, goal_lat: float, goal_lon: float
) -> list[dict[str, float]]:
    """
    Return a list of {latitude, longitude} dicts representing the routed path.
    Falls back to [start, goal] if pathfinding fails or points are off-campus.
    """
    fallback = [
        {"latitude": start_lat, "longitude": start_lon},
        {"latitude": goal_lat, "longitude": goal_lon},
    ]

    if not _within_campus(start_lat, start_lon) or not _within_campus(goal_lat, goal_lon):
        return fallback

    padding = 0.01  # ~1km padding around the route bbox
    min_lat = min(start_lat, goal_lat) - padding
    max_lat = max(start_lat, goal_lat) + padding
    min_lon = min(start_lon, goal_lon) - padding
    max_lon = max(start_lon, goal_lon) + padding

    # Clamp to campus bounds
    campus_pad = MAX_CAMPUS_DISTANCE_M / 111_000
    min_lat = max(min_lat, UCF_CENTER_LAT - campus_pad)
    max_lat = min(max_lat, UCF_CENTER_LAT + campus_pad)
    min_lon = max(min_lon, UCF_CENTER_LON - campus_pad)
    max_lon = min(max_lon, UCF_CENTER_LON + campus_pad)

    try:
        nodes, edges = _fetch_graph(min_lat, min_lon, max_lat, max_lon)
    except Exception as e:
        print(f"[pathfinding] Overpass fetch failed: {e} — using direct line")
        return fallback

    if not nodes:
        return fallback

    start_id = _nearest_node(nodes, start_lat, start_lon)
    goal_id = _nearest_node(nodes, goal_lat, goal_lon)

    if not start_id or not goal_id:
        return fallback

    path_ids = _astar(nodes, edges, start_id, goal_id)

    if not path_ids:
        return fallback

    return [{"latitude": nodes[nid][0], "longitude": nodes[nid][1]} for nid in path_ids]
