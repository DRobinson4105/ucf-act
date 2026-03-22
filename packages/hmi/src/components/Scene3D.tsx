import React, { Suspense, useRef } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, Environment, Grid, ContactShadows } from '@react-three/drei'
import { GolfCart } from './GolfCart'
import * as THREE from 'three'

// ─── Constants ────────────────────────────────────────────────────────────────
const MAX_SPEED_MPH = 8
// How many world-X units the dashes scroll per second at max speed.
const SCROLL_SPEED_AT_MAX = 7   // units/sec at 8 mph
const DASH_LENGTH = 2.2         // world X (road forward direction)
const DASH_WIDTH  = 0.13        // world Z (road width direction)
const GAP         = 3.3         // space between dashes
const PERIOD      = DASH_LENGTH + GAP   // 5.5 — seamless tile length
const NUM_DASHES  = 22          // covers ~121 world-X units of road
const DASH_START_X = 0          // first dash at cart position
const ROAD_HEIGHT = 0.02        // height of road markings above surface

// ─── Animated center dashes ──────────────────────────────────────────────────
function CenterDashes({ speed }: { speed: number }) {
  const groupRef = useRef<THREE.Group>(null)
  const offsetRef = useRef(0)

  useFrame((_, delta) => {
    // Scroll speed proportional to actual speed (0 → 8 mph maps to 0 → SCROLL_SPEED_AT_MAX)
    const scrollSpeed = (Math.min(speed, MAX_SPEED_MPH) / MAX_SPEED_MPH) * SCROLL_SPEED_AT_MAX
    offsetRef.current = (offsetRef.current + delta * scrollSpeed) % PERIOD
    if (groupRef.current) {
      // Shift group in -X direction: dashes flow from high X (horizon) toward camera
      groupRef.current.position.x = -offsetRef.current
    }
  })

  return (
    <group ref={groupRef}>
      {Array.from({ length: NUM_DASHES }, (_, i) => (
        <mesh
          key={i}
          rotation={[-Math.PI / 2, 0, 0]}
          position={[DASH_START_X + i * PERIOD, ROAD_HEIGHT, 0]}
        >
          {/* args=[length, width] — with this rotation: length→world X, width→world Z */}
          <planeGeometry args={[DASH_LENGTH, DASH_WIDTH]} />
          <meshBasicMaterial color="#f5c842" toneMapped={false} />
        </mesh>
      ))}
    </group>
  )
}

// ─── Scene ────────────────────────────────────────────────────────────────────
interface Scene3DProps {
  speed: number
}

export const Scene3D: React.FC<Scene3DProps> = ({ speed }) => {
  return (
    <div className="w-full h-full bg-transparent">
      <Canvas
        camera={{ position: [-5, 3, 0], fov: 55 }}
        shadows
        gl={{ antialias: true, alpha: true }}
      >
        <ambientLight intensity={0.5} />
        <directionalLight position={[5, 10, 5]} intensity={1} castShadow />

        <Suspense fallback={null}>
          <group rotation={[0, Math.PI, 0]}>
            <GolfCart />
          </group>

          {/* Moving reference grid */}
          <Grid
            position={[0, -0.01, 0]}
            args={[60, 40]}
            cellSize={2}
            cellThickness={0.5}
            cellColor="#222"
            sectionSize={10}
            sectionThickness={1}
            sectionColor="#333"
            fadeDistance={35}
            fadeStrength={1}
            infiniteGrid
          />

          {/* Road surface — extends along world X (cart's forward direction) */}
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[20, 0, 0]}>
            <planeGeometry args={[100, 6]} />
            <meshStandardMaterial
              color="#222"
              roughness={0.9}
              polygonOffset
              polygonOffsetFactor={1}
              polygonOffsetUnits={1}
            />
          </mesh>

          {/* Left edge line (negative Z) */}
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[20, ROAD_HEIGHT, -2.5]}>
            <planeGeometry args={[100, 0.2]} />
            <meshBasicMaterial color="#3b82f6" toneMapped={false} />
          </mesh>

          {/* Right edge line (positive Z) */}
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[20, ROAD_HEIGHT, 2.5]}>
            <planeGeometry args={[100, 0.2]} />
            <meshBasicMaterial color="#3b82f6" toneMapped={false} />
          </mesh>

          {/* Center dashes — speed-driven scrolling */}
          <CenterDashes speed={speed} />

          <ContactShadows resolution={1024} scale={30} blur={2} opacity={0.4} far={10} color="#000000" />
          <Environment preset="night" />
        </Suspense>

        <OrbitControls
          target={[3, 0, 0]}
          enablePan={false}
          enableZoom={false}
          minPolarAngle={Math.PI / 6}
          maxPolarAngle={Math.PI / 2.2}
        />
      </Canvas>
    </div>
  )
}
