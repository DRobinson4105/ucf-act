import React from 'react'
import { useGLTF } from '@react-three/drei'
import golfCartModel from '../models/golf_cart.glb'

export const GolfCart: React.FC = () => {
  const { scene } = useGLTF(golfCartModel)

  return (
    <primitive
      object={scene}
      position={[0, 0, 0]}
      scale={0.02}
    />
  )
}

useGLTF.preload(golfCartModel)
