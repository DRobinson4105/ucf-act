const fs = require("node:fs/promises");
const path = require("node:path");

// 28.601761, -81.200566 SU
// 28.599244, -81.201566 millican
const START = {
  latitude: 28.601761,
  longitude: -81.200566,
};

const GOAL = {
  latitude: 28.599244,
  longitude: -81.201566,
};

const OUTPUT_FILE = "./out/path.json";

const CACHE_DURATION = 5 * 60 * 1000;

const OVERPASS_SERVERS = [
  "https://overpass-api.de/api/interpreter",
  "https://overpass.kumi.systems/api/interpreter",
  "https://overpass.openstreetmap.ru/api/interpreter",
];

let cachedGraph = null;
let cacheTimestamp = 0;
let currentServerIndex = 0;

function getDistance(a, b) {
  const R = 6371e3;
  const p1 = (a.latitude * Math.PI) / 180;
  const p2 = (b.latitude * Math.PI) / 180;
  const dp = ((b.latitude - a.latitude) * Math.PI) / 180;
  const dl = ((b.longitude - a.longitude) * Math.PI) / 180;

  const x =
    Math.sin(dp / 2) * Math.sin(dp / 2) +
    Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) * Math.sin(dl / 2);
  return R * 2 * Math.atan2(Math.sqrt(x), Math.sqrt(1 - x));
}

async function fetchWithRetry(url, options, maxRetries = 3, initialDelay = 1000) {
  let lastError = null;

  for (let i = 0; i < maxRetries; i++) {
    try {
      console.log(`Attempt ${i + 1}/${maxRetries} to fetch from ${url}`);
      const response = await Promise.race([
        fetch(url, options),
        new Promise((_, reject) =>
          setTimeout(() => reject(new Error("Request timeout")), 30000)
        ),
      ]);
      return response;
    } catch (error) {
      lastError = error;
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

async function fetchPedestrianPaths(minLat, minLon, maxLat, maxLon) {
  const now = Date.now();
  if (cachedGraph && now - cacheTimestamp < CACHE_DURATION) {
    console.log("Using cached OSM graph");
    return cachedGraph;
  }

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

  let lastError = null;

  for (let serverAttempt = 0; serverAttempt < OVERPASS_SERVERS.length; serverAttempt++) {
    const serverUrl =
      OVERPASS_SERVERS[(currentServerIndex + serverAttempt) % OVERPASS_SERVERS.length];

    try {
      console.log(`Trying server: ${serverUrl}`);

      const response = await fetchWithRetry(
        serverUrl,
        {
          method: "POST",
          body: overpassQuery,
          headers: { "Content-Type": "application/x-www-form-urlencoded" },
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
        console.error("JSON parse error. Response text:", responseText.substring(0, 200));
        throw new Error("Failed to parse Overpass API response");
      }

      console.log(`Received ${data.elements.length} elements from OSM`);

      const nodes = new Map();
      const edges = new Map();

      for (const e of data.elements) {
        if (e.type === "node") {
          nodes.set(String(e.id), { id: String(e.id), lat: e.lat, lon: e.lon });
        }
      }

      for (const e of data.elements) {
        if (e.type !== "way") continue;
        for (let j = 0; j < e.nodes.length - 1; j++) {
          const a = String(e.nodes[j]);
          const b = String(e.nodes[j + 1]);
          if (!edges.has(a)) edges.set(a, []);
          if (!edges.has(b)) edges.set(b, []);
          edges.get(a).push(b);
          edges.get(b).push(a);
        }
      }

      const graph = { nodes, edges };
      cachedGraph = graph;
      cacheTimestamp = now;
      currentServerIndex =
        (currentServerIndex + serverAttempt) % OVERPASS_SERVERS.length;

      return graph;
    } catch (error) {
      lastError = error;
      console.error(`Server ${serverUrl} failed:`, error);
    }
  }

  console.error("All Overpass servers failed");
  throw lastError || new Error("All Overpass servers unavailable");
}

function nearestNode(graph, lat, lon) {
  let best = null;
  let d = Infinity;

  for (const n of graph.nodes.values()) {
    const dist = getDistance(
      { latitude: lat, longitude: lon },
      { latitude: n.lat, longitude: n.lon }
    );
    if (dist < d) {
      d = dist;
      best = n;
    }
  }
  return best;
}

function runAStar(graph, startId, goalId, goalLat, goalLon) {
  const openSet = [];
  const closedSet = new Set();
  const gScores = new Map();

  const startNode = graph.nodes.get(startId);
  if (!startNode) return [];

  const startAStarNode = {
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
    const current = openSet.shift();

    if (current.id === goalId) {
      const pathIds = [];
      let node = current;
      while (node) {
        pathIds.unshift(node.id);
        node = node.parent;
      }
      return pathIds;
    }

    closedSet.add(current.id);

    const neighbors = graph.edges.get(current.id) || [];
    const currentNode = graph.nodes.get(current.id);
    if (!currentNode) continue;

    for (const neighborId of neighbors) {
      if (closedSet.has(neighborId)) continue;

      const neighborNode = graph.nodes.get(neighborId);
      if (!neighborNode) continue;

      const tentativeG =
        current.g +
        getDistance(
          { latitude: currentNode.lat, longitude: currentNode.lon },
          { latitude: neighborNode.lat, longitude: neighborNode.lon }
        );

      const existingG = gScores.get(neighborId);
      if (existingG !== undefined && tentativeG >= existingG) continue;

      gScores.set(neighborId, tentativeG);

      const h = getDistance(
        { latitude: neighborNode.lat, longitude: neighborNode.lon },
        { latitude: goalLat, longitude: goalLon }
      );

      const neighborAStarNode = {
        id: neighborId,
        g: tentativeG,
        h,
        f: tentativeG + h,
        parent: current,
      };

      const idx = openSet.findIndex((n) => n.id === neighborId);
      if (idx !== -1) {
        if (tentativeG < openSet[idx].g) openSet[idx] = neighborAStarNode;
      } else {
        openSet.push(neighborAStarNode);
      }
    }
  }

  return [];
}

async function findPath(start, goal) {
  try {
    const distance = getDistance(start, goal);
    const padding = Math.max(0.005, Math.min(0.02, distance / 111000));

    const bbox = {
      minLat: Math.min(start.latitude, goal.latitude) - padding,
      maxLat: Math.max(start.latitude, goal.latitude) + padding,
      minLon: Math.min(start.longitude, goal.longitude) - padding,
      maxLon: Math.max(start.longitude, goal.longitude) + padding,
    };

    let graph;
    try {
      graph = await fetchPedestrianPaths(bbox.minLat, bbox.minLon, bbox.maxLat, bbox.maxLon);
    } catch (fetchError) {
      console.error("Failed to fetch pedestrian paths:", fetchError);
      console.log("Falling back to direct line due to API error");
      return [start, goal];
    }

    if (graph.nodes.size === 0) {
      console.log("No walkable paths found in area, returning direct line");
      return [start, goal];
    }

    const s = nearestNode(graph, start.latitude, start.longitude);
    const g = nearestNode(graph, goal.latitude, goal.longitude);

    if (!s || !g) {
      console.log("Could not find nearest nodes, returning direct line");
      return [start, goal];
    }

    const ids = runAStar(graph, s.id, g.id, goal.latitude, goal.longitude);

    if (ids.length === 0) {
      console.log("No path found between nodes, returning direct line");
      return [start, goal];
    }

    const pathNodes = ids.map((id) => {
      const n = graph.nodes.get(id);
      return { latitude: n.lat, longitude: n.lon };
    });

    console.log(`Path found with ${pathNodes.length} nodes`);
    return pathNodes;
  } catch (error) {
    console.error("Unexpected error in findPath:", error);
    return [start, goal];
  }
}

async function main() {
  const route = await findPath(START, GOAL);
  const json = JSON.stringify(route, null, 2);

  const out = path.resolve(OUTPUT_FILE);
  await fs.mkdir(path.dirname(out), { recursive: true });
  await fs.writeFile(out, json, "utf8");

  console.log(out);
}

main().catch((e) => {
  console.error(e && e.message ? e.message : String(e));
  process.exit(1);
});
