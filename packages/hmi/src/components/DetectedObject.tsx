import React, { useMemo, useEffect } from 'react'
import { useGLTF, useAnimations } from '@react-three/drei'
import { Zone } from '../types'
import personModel from '../models/person_walking.glb'
import bikeModel from '../models/bike.glb'
import carModel from '../models/car_low_poly.glb'

// Preload models
useGLTF.preload(personModel)
useGLTF.preload(bikeModel)
useGLTF.preload(carModel)

interface DetectedObjectProps {
  type: 'person' | 'bike' | 'car'
  position: [number, number, number]
  rotation: [number, number, number]
}

export const DetectedObject: React.FC<DetectedObjectProps> = ({ type, position, rotation }) => {
  const modelPath = useMemo(() => {
    switch (type) {
      case 'person': return personModel
      case 'bike': return bikeModel
      case 'car': return carModel
      default: return personModel
    }
  }, [type])

  const { scene } = useGLTF(modelPath)
  // Clone scene for multiple instances
  const clonedScene = useMemo(() => scene.clone(), [scene])

  // Scale adjustments for typical low poly models to match our scene scale
  // Cart is ~0.02 scale for meters. These should likely be similar.
  const scale: [number, number, number] = useMemo(() => {
    switch (type) {
      case 'person': return [0.015, 0.015, 0.015]
      case 'bike': return [0.004, 0.004, 0.004]
      case 'car': return [0.5, 0.5, 0.5]
      default: return [0.015, 0.015, 0.015]
    }
  }, [type])

  return (
    <primitive
      object={clonedScene}
      position={position}
      rotation={rotation}
      scale={scale}
    />
  )
}
