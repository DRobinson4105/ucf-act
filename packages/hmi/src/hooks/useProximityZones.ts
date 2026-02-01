import { useState, useEffect, useCallback } from 'react'
import { Zone } from '../types'

const createInitialZones = (): Zone[] => [
  // Front zones (Far Forward)
  { id: 1, position: [20, 0, 0], rotation: [0, 0, 0], size: [2, 1, 1.5], status: 'clear' },
  { id: 2, position: [15, 0, 4], rotation: [0, 0, 0], size: [1.5, 1, 1], status: 'clear' },
  { id: 3, position: [15, 0, -4], rotation: [0, 0, 0], size: [1.5, 1, 1], status: 'clear' },

  // Side zones (Wide Left/Right)
  { id: 4, position: [0, 0, 8], rotation: [0, 3.14, 0], size: [4, 1, 1.5], status: 'clear' },
  { id: 5, position: [0, 0, -8], rotation: [0, 0, 0], size: [4, 1, 1.5], status: 'clear' },

  // Rear zones (Far Back)
  { id: 6, position: [-20, 0, 0], rotation: [0, 3.14, 0], size: [2, 1, 1.5], status: 'clear' },
  { id: 7, position: [-15, 0, 4], rotation: [0, 3.14, 0], size: [1.5, 1, 1], status: 'clear' },
  { id: 8, position: [-15, 0, -4], rotation: [0, 3.14, 0], size: [1.5, 1, 1], status: 'clear' }
]

const objectTypes: ('person' | 'bike' | 'car')[] = ['person', 'bike', 'car']

export const useProximityZones = () => {
  const [zones, setZones] = useState<Zone[]>(createInitialZones())
  const [activeDetections, setActiveDetections] = useState<Set<number>>(new Set())

  const triggerDetection = useCallback((zoneId: number) => {
    setZones(prev => prev.map(zone =>
      zone.id === zoneId
        ? {
            ...zone,
            status: 'detected',
            objectType: objectTypes[Math.floor(Math.random() * objectTypes.length)]
          }
        : zone
    ))
    setActiveDetections(prev => new Set([...prev, zoneId]))

    // Clear detection after 3-5 seconds
    const clearTimeout = setTimeout(() => {
      setZones(prev => prev.map(zone =>
        zone.id === zoneId
          ? { ...zone, status: 'clear', objectType: undefined }
          : zone
      ))
      setActiveDetections(prev => {
        const newSet = new Set(prev)
        newSet.delete(zoneId)
        return newSet
      })
    }, 3000 + Math.random() * 2000)

    return () => clearTimeout(clearTimeout)
  }, [])

  useEffect(() => {
    const interval = setInterval(() => {
      // Randomly trigger detections
      if (Math.random() > 0.7) {
        const availableZones = zones.filter(z => z.status === 'clear')
        if (availableZones.length > 0) {
          const randomZone = availableZones[Math.floor(Math.random() * availableZones.length)]
          triggerDetection(randomZone.id)
        }
      }
    }, 1500)

    return () => clearInterval(interval)
  }, [zones, triggerDetection])

  return { zones, activeDetections, triggerDetection }
}
