import { UCF_CENTER } from "@/constants/campus-locations";
import campusGraph from "@/constants/campus-graph.json";

export interface PathNode {
  latitude: number;
  longitude: number;
}

interface GraphNode {
  id: string;
  lat: number;
  lon: number;
}

let adjacency: Map<string, string[]> | null = null;
let nodeMap: Map<string, GraphNode> | null = null;

function ensureGraph() {
  if (adjacency) return;
  nodeMap = new Map();
  adjacency = new Map();

  for (const n of campusGraph.nodes) {
    nodeMap.set(n.id, n);
    adjacency.set(n.id, []);
  }

  for (const [a, b] of campusGraph.edges) {
    adjacency.get(a)?.push(b);
    adjacency.get(b)?.push(a);
  }
}

function getDistance(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 6371e3;
  const p1 = (lat1 * Math.PI) / 180;
  const p2 = (lat2 * Math.PI) / 180;
  const dp = ((lat2 - lat1) * Math.PI) / 180;
  const dl = ((lon2 - lon1) * Math.PI) / 180;
  const a =
    Math.sin(dp / 2) ** 2 +
    Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function findNearest(lat: number, lon: number): GraphNode | null {
  ensureGraph();
  let best: GraphNode | null = null;
  let bestDist = Infinity;

  for (const node of nodeMap!.values()) {
    const d = getDistance(lat, lon, node.lat, node.lon);
    if (d < bestDist) {
      bestDist = d;
      best = node;
    }
  }

  return best;
}

interface AStarEntry {
  id: string;
  g: number;
  f: number;
  parent: string | null;
}

function runAStar(startId: string, goalId: string): string[] {
  ensureGraph();
  const goalNode = nodeMap!.get(goalId)!;
  const openList: AStarEntry[] = [];
  const closed = new Set<string>();
  const gScores = new Map<string, number>();

  const startNode = nodeMap!.get(startId)!;
  const h0 = getDistance(startNode.lat, startNode.lon, goalNode.lat, goalNode.lon);
  openList.push({ id: startId, g: 0, f: h0, parent: null });
  gScores.set(startId, 0);

  const parents = new Map<string, string | null>();
  parents.set(startId, null);

  while (openList.length > 0) {
    let bestIdx = 0;
    for (let i = 1; i < openList.length; i++) {
      if (openList[i].f < openList[bestIdx].f) bestIdx = i;
    }
    const current = openList.splice(bestIdx, 1)[0];

    if (current.id === goalId) {
      const path: string[] = [];
      let id: string | null = goalId;
      while (id !== null) {
        path.unshift(id);
        id = parents.get(id) ?? null;
      }
      return path;
    }

    closed.add(current.id);
    const currentNode = nodeMap!.get(current.id)!;

    for (const neighborId of adjacency!.get(current.id) ?? []) {
      if (closed.has(neighborId)) continue;

      const neighbor = nodeMap!.get(neighborId);
      if (!neighbor) continue;

      const tentG =
        current.g +
        getDistance(currentNode.lat, currentNode.lon, neighbor.lat, neighbor.lon);

      const existingG = gScores.get(neighborId);
      if (existingG !== undefined && tentG >= existingG) continue;

      gScores.set(neighborId, tentG);
      parents.set(neighborId, current.id);

      const h = getDistance(neighbor.lat, neighbor.lon, goalNode.lat, goalNode.lon);
      const existing = openList.findIndex((e) => e.id === neighborId);
      if (existing !== -1) {
        openList[existing] = { id: neighborId, g: tentG, f: tentG + h, parent: current.id };
      } else {
        openList.push({ id: neighborId, g: tentG, f: tentG + h, parent: current.id });
      }
    }
  }

  return [];
}

const MAX_CAMPUS_DISTANCE = 2000;

function isWithinCampus(lat: number, lon: number): boolean {
  return getDistance(lat, lon, UCF_CENTER.latitude, UCF_CENTER.longitude) <= MAX_CAMPUS_DISTANCE;
}

export async function findPath(
  start: PathNode,
  goal: PathNode,
  _center?: { latitude: number; longitude: number }
): Promise<PathNode[]> {
  if (!isWithinCampus(start.latitude, start.longitude) ||
      !isWithinCampus(goal.latitude, goal.longitude)) {
    return [start, goal];
  }

  ensureGraph();

  const startNode = findNearest(start.latitude, start.longitude);
  const goalNode = findNearest(goal.latitude, goal.longitude);

  if (!startNode || !goalNode) return [start, goal];

  const pathIds = runAStar(startNode.id, goalNode.id);
  if (pathIds.length === 0) return [start, goal];

  const path: PathNode[] = pathIds.map((id) => {
    const n = nodeMap!.get(id)!;
    return { latitude: n.lat, longitude: n.lon };
  });

  const first = path[0];
  if (first.latitude !== start.latitude || first.longitude !== start.longitude) {
    path.unshift(start);
  }
  const last = path[path.length - 1];
  if (last.latitude !== goal.latitude || last.longitude !== goal.longitude) {
    path.push(goal);
  }

  return path;
}
