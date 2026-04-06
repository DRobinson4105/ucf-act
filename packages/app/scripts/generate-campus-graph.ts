/**
 * Build-time script to fetch UCF campus walkway/road graph from OpenStreetMap
 * and save as a static JSON file for instant client-side pathfinding.
 *
 * Usage: npx tsx packages/app/scripts/generate-campus-graph.ts
 */

const UCF_CENTER = { latitude: 28.6024, longitude: -81.2001 };
const MAX_CAMPUS_DISTANCE_M = 2000;
const OUTPUT_PATH = new URL("../constants/campus-graph.json", import.meta.url);

interface OSMElement {
  type: "node" | "way";
  id: number;
  lat?: number;
  lon?: number;
  nodes?: number[];
  tags?: Record<string, string>;
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
    Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

async function main() {
  const pad = MAX_CAMPUS_DISTANCE_M / 111000;
  const bbox = {
    minLat: UCF_CENTER.latitude - pad,
    maxLat: UCF_CENTER.latitude + pad,
    minLon: UCF_CENTER.longitude - pad,
    maxLon: UCF_CENTER.longitude + pad,
  };

  const query = `
    [out:json][timeout:60];
    (
      way["highway"](${bbox.minLat},${bbox.minLon},${bbox.maxLat},${bbox.maxLon});
    );
    out body;
    >;
    out skel qt;
  `;

  console.log("Fetching OSM data for UCF campus...");

  const res = await fetch("https://overpass-api.de/api/interpreter", {
    method: "POST",
    body: query,
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
  });

  if (!res.ok) throw new Error(`Overpass API error: ${res.status}`);
  const data = await res.json();

  console.log(`Received ${data.elements.length} elements`);

  const rawNodes = new Map<number, { lat: number; lon: number }>();
  const ways: { nodes: number[] }[] = [];

  for (const el of data.elements as OSMElement[]) {
    if (el.type === "node" && el.lat !== undefined && el.lon !== undefined) {
      rawNodes.set(el.id, { lat: el.lat, lon: el.lon });
    } else if (el.type === "way" && el.nodes) {
      ways.push({ nodes: el.nodes });
    }
  }

  const validNodeIds = new Set<number>();
  for (const [id, { lat, lon }] of rawNodes) {
    if (getDistance(lat, lon, UCF_CENTER.latitude, UCF_CENTER.longitude) <= MAX_CAMPUS_DISTANCE_M) {
      validNodeIds.add(id);
    }
  }

  const edgeSet = new Set<string>();
  const edges: [string, string][] = [];
  const usedNodeIds = new Set<number>();

  for (const way of ways) {
    for (let i = 0; i < way.nodes.length - 1; i++) {
      const a = way.nodes[i];
      const b = way.nodes[i + 1];
      if (!validNodeIds.has(a) || !validNodeIds.has(b)) continue;

      const key = a < b ? `${a}-${b}` : `${b}-${a}`;
      if (edgeSet.has(key)) continue;
      edgeSet.add(key);

      edges.push([String(a), String(b)]);
      usedNodeIds.add(a);
      usedNodeIds.add(b);
    }
  }

  const nodes: { id: string; lat: number; lon: number }[] = [];
  for (const id of usedNodeIds) {
    const n = rawNodes.get(id)!;
    nodes.push({ id: String(id), lat: n.lat, lon: n.lon });
  }

  const graph = { nodes, edges };

  const fs = await import("node:fs");
  const { fileURLToPath } = await import("node:url");
  const outPath = fileURLToPath(OUTPUT_PATH as unknown as string);
  fs.writeFileSync(outPath, JSON.stringify(graph));

  console.log(`Written ${nodes.length} nodes, ${edges.length} edges to ${outPath}`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
