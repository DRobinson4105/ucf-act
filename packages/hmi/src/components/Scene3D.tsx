import React, { Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Environment, Grid, ContactShadows } from '@react-three/drei'
import { GolfCart } from './GolfCart'
import { DetectedObject } from './DetectedObject'
import { Zone } from '../types'

interface Scene3DProps {
  zones: Zone[]
}

export const Scene3D: React.FC<Scene3DProps> = ({ zones }) => {
  return (
    <div className="w-full h-full bg-transparent">
      <Canvas
        camera={{ position: [-12, 4, 0], fov: 50 }} // Camera at -X (Behind), +Y (Up), 0 Z (Center)
        shadows
        gl={{ antialias: true, alpha: true }}
      >
        <ambientLight intensity={0.5} />
        <directionalLight
          position={[10, 1, 5]}
          intensity={1}
          castShadow
        />

        <Suspense fallback={null}>
          <group rotation={[0, -Math.PI/2, 0]}> {/* Rotate car 180deg to face +Z (aligned with new road) */}
             <GolfCart />
          </group>

          {zones.map(zone => (
            zone.status === 'detected' && zone.objectType ? (
               <DetectedObject
                  key={zone.id}
                  type={zone.objectType}
                  position={[zone.position[0], 0, zone.position[2]]} // Correct mapping: X=Forward, Z=Side (matches Camera/Car)
                  rotation={[0, zone.rotation[1] + Math.PI/2, 0]} // Rotate objects to match
               />
            ) : null
          ))}

          {/* "Moving" Reference Line */}
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

          {/* Main Road Surface */}
          <mesh rotation={[-Math.PI / 2, 0, Math.PI / 2]} position={[0, 0.005, 0]}>
             <planeGeometry args={[100, 6]} /> {/* Slightly wider road */}
             <meshStandardMaterial color="#222" roughness={0.9} />
          </mesh>

          {/* Left Lane Line (Blue) */}
           <mesh rotation={[-Math.PI / 2, 0, Math.PI / 2]} position={[-1.75, 0.02, 0]}>
             <planeGeometry args={[100, 0.2]} />
             <meshBasicMaterial color="#3b82f6" toneMapped={false} /> {/* Tesla Blue */}
          </mesh>

          {/* Right Lane Line (Blue) */}
           <mesh rotation={[-Math.PI / 2, 0, Math.PI / 2]} position={[1.75, 0.02, 0]}>
             <planeGeometry args={[100, 0.2]} />
             <meshBasicMaterial color="#3b82f6" toneMapped={false} /> {/* Tesla Blue */}
          </mesh>

          <ContactShadows resolution={1024} scale={30} blur={2} opacity={0.4} far={10} color="#000000" />

          <Environment preset="night" />
        </Suspense>

        <OrbitControls
           enablePan={false}
           enableZoom={false}
           minPolarAngle={Math.PI / 3}
           maxPolarAngle={Math.PI / 2.2}
           // Restrict azimuth to keep view "behind" car mostly
           minAzimuthAngle={-Math.PI / 4}
           maxAzimuthAngle={Math.PI / 4}
        />
      </Canvas>
    </div>
  )
}
