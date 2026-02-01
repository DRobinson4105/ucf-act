import React from 'react'
import { Box } from '@react-three/drei'
import { Zone } from '../types'
import * as THREE from 'three'

interface ProximityZoneProps {
  zone: Zone
}

const ObjectMarker: React.FC<{ type: 'person' | 'bike' | 'car' }> = ({ type }) => {
  const getColor = () => {
    switch (type) {
      case 'person': return '#ef4444'
      case 'bike': return '#f59e0b'
      case 'car': return '#8b5cf6'
      default: return '#ef4444'
    }
  }

  const getGeometry = () => {
    switch (type) {
      case 'person':
        return <boxGeometry args={[0.2, 0.4, 0.2]} />
      case 'bike':
        return <boxGeometry args={[0.3, 0.2, 0.1]} />
      case 'car':
        return <boxGeometry args={[0.4, 0.15, 0.2]} />
      default:
        return <boxGeometry args={[0.2, 0.2, 0.2]} />
    }
  }

  return (
    <mesh position={[0, 0.2, 0]}>
      {getGeometry()}
      <meshStandardMaterial
        color={getColor()}
        emissive={getColor()}
        emissiveIntensity={0.3}
      />
    </mesh>
  )
}

export const ProximityZone: React.FC<ProximityZoneProps> = ({ zone }) => {
  const getZoneColor = () => {
    return zone.status === 'clear' ? '#3b82f6' : '#ef4444'
  }

  return (
    <group position={zone.position} rotation={zone.rotation}>
      {/* Zone visualization */}
      <Box args={zone.size}>
        <meshStandardMaterial
          color={getZoneColor()}
          transparent
          opacity={zone.status === 'clear' ? 0.1 : 0.3}
          wireframe={zone.status === 'clear'}
          emissive={getZoneColor()}
          emissiveIntensity={zone.status === 'detected' ? 0.2 : 0}
        />
      </Box>

      {/* Zone border */}
      <Box args={zone.size}>
        <meshBasicMaterial
          color={getZoneColor()}
          wireframe
          transparent
          opacity={0.6}
        />
      </Box>

      {/* Object marker if detected */}
      {zone.status === 'detected' && zone.objectType && (
        <ObjectMarker type={zone.objectType} />
      )}

      {/* Zone label */}
      <mesh position={[0, zone.size[1] / 2 + 0.3, 0]}>
        <planeGeometry args={[0.3, 0.2]} />
        <meshBasicMaterial
          color={zone.status === 'clear' ? '#1a1a1a' : '#ef4444'}
          transparent
          opacity={0.8}
        />
      </mesh>
    </group>
  )
}
