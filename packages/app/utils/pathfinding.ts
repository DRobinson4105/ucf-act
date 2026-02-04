import { UCF_CENTER } from "@/constants/campus-locations";

export interface PathNode {
  latitude: number;
  longitude: number;
}

// Maximum distance from UCF campus center in meters (2km = 2000m)
const MAX_CAMPUS_DISTANCE = 2000;

interface OSMNode {
  id: string;
  lat: number;
  lon: number;
}

interface OSMWay {
  id: string;
  nodes: string[];
  tags: Record<string, string>;
}

interface OSMGraph {
  nodes: Map<string, OSMNode>;
  edges: Map<string, string[]>;
}

let cachedGraph: OSMGraph | null = null;
let cacheTimestamp: number = 0;
const CACHE_DURATION = 5 * 60 * 1000;

const OVERPASS_SERVERS = [
  "https://overpass-api.de/api/interpreter",
  "https://overpass.kumi.systems/api/interpreter",
  "https://overpass.openstreetmap.ru/api/interpreter",
];

let currentServerIndex = 0;

function getDistance(
  node1: { latitude: number; longitude: number },
  node2: { latitude: number; longitude: number }
): number {
  const R = 6371e3;
  const φ1 = (node1.latitude * Math.PI) / 180;
  const φ2 = (node2.latitude * Math.PI) / 180;
  const Δφ = ((node2.latitude - node1.latitude) * Math.PI) / 180;
  const Δλ = ((node2.longitude - node1.longitude) * Math.PI) / 180;

  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

async function fetchWithRetry(
  url: string,
  options: RequestInit,
  maxRetries: number = 3,
  initialDelay: number = 1000
): Promise<Response> {
  let lastError: Error | null = null;

  for (let i = 0; i < maxRetries; i++) {
    try {
      console.log(`Attempt ${i + 1}/${maxRetries} to fetch from ${url}`);
      const response = await Promise.race([
        fetch(url, options),
        new Promise<never>((_, reject) =>
          setTimeout(() => reject(new Error("Request timeout")), 30000)
        ),
      ]);
      return response;
    } catch (error) {
      lastError = error as Error;
      console.log(`Attempt ${i + 1} failed:`, error);

      if (i < maxRetries - 1) {
        const delay = initialDelay * Math.pow(2, i);
        console.log(`Waiting ${delay}ms before retry...`);
        await new Promise((resolve) => setTimeout(resolve, delay));
      }
    }
  }

  throw lastError || new Error("All retry attempts failed");
}

async function fetchPedestrianPaths(
  minLat: number,
  minLon: number,
  maxLat: number,
  maxLon: number
): Promise<OSMGraph> {
  const now = Date.now();
  if (cachedGraph && now - cacheTimestamp < CACHE_DURATION) {
    console.log("Using cached OSM graph");
    return cachedGraph;
  }

  console.log("Fetching pedestrian paths from OpenStreetMap...");

  const overpassQuery = `
    [out:json][timeout:30];
    (
      way["highway"~"^(footway|path|pedestrian|steps|cycleway|service|residential|living_street)$"](${minLat},${minLon},${maxLat},${maxLon});
      way["footway"](${minLat},${minLon},${maxLat},${maxLon});
    );
    out body;
    >;
    out skel qt;
  `;

  let lastError: Error | null = null;

  for (
    let serverAttempt = 0;
    serverAttempt < OVERPASS_SERVERS.length;
    serverAttempt++
  ) {
    const serverUrl =
      OVERPASS_SERVERS[
        (currentServerIndex + serverAttempt) % OVERPASS_SERVERS.length
      ];

    try {
      console.log(`Trying server: ${serverUrl}`);

      const response = await fetchWithRetry(
        serverUrl,
        {
          method: "POST",
          body: overpassQuery,
          headers: {
            "Content-Type": "application/x-www-form-urlencoded",
          },
        },
        2,
        1500
      );

      if (!response.ok) {
        await response.text();
        console.error(`Server ${serverUrl} error:`, response.status);
        throw new Error(`Overpass API error: ${response.status}`);
      }

      const responseText = await response.text();
      let data;
      try {
        data = JSON.parse(responseText);
      } catch {
        console.error(
          "JSON parse error. Response text:",
          responseText.substring(0, 200)
        );
        throw new Error("Failed to parse Overpass API response");
      }

      console.log(`Received ${data.elements.length} elements from OSM`);

      const nodes = new Map<string, OSMNode>();
      const ways: OSMWay[] = [];

      data.elements.forEach((element: any) => {
        if (element.type === "node") {
          nodes.set(String(element.id), {
            id: String(element.id),
            lat: element.lat,
            lon: element.lon,
          });
        } else if (element.type === "way") {
          ways.push({
            id: String(element.id),
            nodes: element.nodes.map((n: number) => String(n)),
            tags: element.tags || {},
          });
        }
      });

      const edges = new Map<string, string[]>();

      ways.forEach((way) => {
        for (let i = 0; i < way.nodes.length - 1; i++) {
          const node1 = way.nodes[i];
          const node2 = way.nodes[i + 1];

          if (!edges.has(node1)) {
            edges.set(node1, []);
          }
          if (!edges.has(node2)) {
            edges.set(node2, []);
          }

          edges.get(node1)!.push(node2);
          edges.get(node2)!.push(node1);
        }
      });

      // Filter graph to only include nodes within campus bounds
      const filteredNodes = new Map<string, OSMNode>();
      const filteredEdges = new Map<string, string[]>();

      nodes.forEach((node, nodeId) => {
        if (isWithinCampusBounds(node.lat, node.lon)) {
          filteredNodes.set(nodeId, node);
        }
      });

      // Only include edges between nodes that are both within campus bounds
      edges.forEach((neighborIds, nodeId) => {
        if (!filteredNodes.has(nodeId)) return;

        const validNeighbors = neighborIds.filter((neighborId) =>
          filteredNodes.has(neighborId)
        );
        if (validNeighbors.length > 0) {
          filteredEdges.set(nodeId, validNeighbors);
        }
      });

      const graph: OSMGraph = { nodes: filteredNodes, edges: filteredEdges };
      cachedGraph = graph;
      cacheTimestamp = now;
      currentServerIndex =
        (currentServerIndex + serverAttempt) % OVERPASS_SERVERS.length;

      console.log(
        `Built graph with ${filteredNodes.size} nodes and ${filteredEdges.size} edge points (filtered to campus bounds)`
      );
      return graph;
    } catch (error) {
      lastError = error as Error;
      console.error(`Server ${serverUrl} failed:`, error);
    }
  }

  console.error("All Overpass servers failed");
  throw lastError || new Error("All Overpass servers unavailable");
}

function isWithinCampusBounds(lat: number, lon: number): boolean {
  const distance = getDistance(
    { latitude: UCF_CENTER.latitude, longitude: UCF_CENTER.longitude },
    { latitude: lat, longitude: lon }
  );
  return distance <= MAX_CAMPUS_DISTANCE;
}

function findNearestNode(
  graph: OSMGraph,
  lat: number,
  lon: number
): OSMNode | null {
  let nearest: OSMNode | null = null;
  let minDistance = Infinity;

  graph.nodes.forEach((node) => {
    // Only consider nodes within campus bounds
    if (!isWithinCampusBounds(node.lat, node.lon)) {
      return;
    }

    const distance = getDistance(
      { latitude: lat, longitude: lon },
      { latitude: node.lat, longitude: node.lon }
    );
    if (distance < minDistance) {
      minDistance = distance;
      nearest = node;
    }
  });

  return nearest;
}

interface AStarNode {
  id: string;
  g: number;
  h: number;
  f: number;
  parent: AStarNode | null;
}

function runAStar(
  graph: OSMGraph,
  startId: string,
  goalId: string,
  goalLat: number,
  goalLon: number
): string[] {
  const openSet: AStarNode[] = [];
  const closedSet = new Set<string>();
  const gScores = new Map<string, number>();

  const startNode = graph.nodes.get(startId);
  if (!startNode) return [];

  const startAStarNode: AStarNode = {
    id: startId,
    g: 0,
    h: getDistance(
      { latitude: startNode.lat, longitude: startNode.lon },
      { latitude: goalLat, longitude: goalLon }
    ),
    f: 0,
    parent: null,
  };
  startAStarNode.f = startAStarNode.g + startAStarNode.h;

  openSet.push(startAStarNode);
  gScores.set(startId, 0);

  while (openSet.length > 0) {
    openSet.sort((a, b) => a.f - b.f);
    const current = openSet.shift()!;

    if (current.id === goalId) {
      const path: string[] = [];
      let node: AStarNode | null = current;
      while (node) {
        path.unshift(node.id);
        node = node.parent;
      }
      return path;
    }

    closedSet.add(current.id);

    const neighbors = graph.edges.get(current.id) || [];
    const currentNode = graph.nodes.get(current.id);
    if (!currentNode) continue;

    for (const neighborId of neighbors) {
      if (closedSet.has(neighborId)) continue;

      const neighborNode = graph.nodes.get(neighborId);
      if (!neighborNode) continue;

      // Skip nodes that are outside campus bounds
      if (!isWithinCampusBounds(neighborNode.lat, neighborNode.lon)) {
        continue;
      }

      const tentativeG =
        current.g +
        getDistance(
          { latitude: currentNode.lat, longitude: currentNode.lon },
          { latitude: neighborNode.lat, longitude: neighborNode.lon }
        );

      const existingG = gScores.get(neighborId);
      if (existingG !== undefined && tentativeG >= existingG) {
        continue;
      }

      gScores.set(neighborId, tentativeG);

      const h = getDistance(
        { latitude: neighborNode.lat, longitude: neighborNode.lon },
        { latitude: goalLat, longitude: goalLon }
      );

      const neighborAStarNode: AStarNode = {
        id: neighborId,
        g: tentativeG,
        h: h,
        f: tentativeG + h,
        parent: current,
      };

      const existingNodeIndex = openSet.findIndex((n) => n.id === neighborId);
      if (existingNodeIndex !== -1) {
        if (tentativeG < openSet[existingNodeIndex].g) {
          openSet[existingNodeIndex] = neighborAStarNode;
        }
      } else {
        openSet.push(neighborAStarNode);
      }
    }
  }

  return [];
}

export async function findPath(
  start: PathNode,
  goal: PathNode
): Promise<PathNode[]> {
  try {
    // Check if start and goal are within campus bounds
    if (!isWithinCampusBounds(start.latitude, start.longitude)) {
      console.log("Start location is outside campus bounds, returning direct line");
      return [start, goal];
    }

    if (!isWithinCampusBounds(goal.latitude, goal.longitude)) {
      console.log("Goal location is outside campus bounds, returning direct line");
      return [start, goal];
    }

    const distance = getDistance(start, goal);
    const padding = Math.max(0.005, Math.min(0.02, distance / 111000));

    // Calculate bounding box but constrain it to campus area
    const campusPadding = MAX_CAMPUS_DISTANCE / 111000; // Convert meters to degrees (approx)
    const campusBbox = {
      minLat: UCF_CENTER.latitude - campusPadding,
      maxLat: UCF_CENTER.latitude + campusPadding,
      minLon: UCF_CENTER.longitude - campusPadding,
      maxLon: UCF_CENTER.longitude + campusPadding,
    };

    const routeBbox = {
      minLat: Math.min(start.latitude, goal.latitude) - padding,
      maxLat: Math.max(start.latitude, goal.latitude) + padding,
      minLon: Math.min(start.longitude, goal.longitude) - padding,
      maxLon: Math.max(start.longitude, goal.longitude) + padding,
    };

    // Intersect route bbox with campus bbox to ensure we don't search outside campus
    const bbox = {
      minLat: Math.max(routeBbox.minLat, campusBbox.minLat),
      maxLat: Math.min(routeBbox.maxLat, campusBbox.maxLat),
      minLon: Math.max(routeBbox.minLon, campusBbox.minLon),
      maxLon: Math.min(routeBbox.maxLon, campusBbox.maxLon),
    };

    console.log(
      `Finding path over ${(distance / 1000).toFixed(2)}km with bbox padding ${padding.toFixed(4)}`
    );

    let graph: OSMGraph;
    try {
      graph = await fetchPedestrianPaths(
        bbox.minLat,
        bbox.minLon,
        bbox.maxLat,
        bbox.maxLon
      );
    } catch (fetchError) {
      console.error("Failed to fetch pedestrian paths:", fetchError);
      console.log("Falling back to direct line due to API error");
      return [start, goal];
    }

    if (graph.nodes.size === 0) {
      console.log("No walkable paths found in area, returning direct line");
      return [start, goal];
    }

    const startNode = findNearestNode(graph, start.latitude, start.longitude);
    const goalNode = findNearestNode(graph, goal.latitude, goal.longitude);

    if (!startNode || !goalNode) {
      console.log("Could not find nearest nodes, returning direct line");
      return [start, goal];
    }

    console.log("Running A* pathfinding...");
    const pathNodeIds = runAStar(
      graph,
      startNode.id,
      goalNode.id,
      goal.latitude,
      goal.longitude
    );

    if (pathNodeIds.length === 0) {
      console.log("No path found between nodes, returning direct line");
      return [start, goal];
    }

    const pathNodes: PathNode[] = pathNodeIds.map((nodeId) => {
      const node = graph.nodes.get(nodeId)!;
      return {
        latitude: node.lat,
        longitude: node.lon,
      };
    });

    console.log(`Path found with ${pathNodes.length} nodes`);
    return pathNodes;
  } catch (error) {
    console.error("Unexpected error in findPath:", error);
    return [start, goal];
  }
}
