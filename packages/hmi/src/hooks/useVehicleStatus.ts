import { useState, useEffect } from 'react'
import { VehicleStatus, GPSPosition } from '../types'
import { findPath } from '../utils/pathfinding'
import { UCF_CENTER } from '../constants/campus-locations'

export const useVehicleStatus = () => {
  const [status, setStatus] = useState<VehicleStatus>({
    battery: 85,
    speed: 12,
    connectivity: 'excellent',
    isConnected: true
  })

  const [currentTime, setCurrentTime] = useState(new Date())

  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentTime(new Date())

      setStatus(prev => ({
        ...prev,
        battery: Math.max(0, prev.battery - 0.01),
        speed: Math.max(0, Math.min(25, prev.speed + (Math.random() - 0.5) * 2)),
        connectivity: Math.random() > 0.95 ?
          (['excellent', 'good', 'poor'] as const)[Math.floor(Math.random() * 3)] :
          prev.connectivity,
        isConnected: Math.random() > 0.05
      }))
    }, 1000)

    return () => clearInterval(interval)
  }, [])

  return { status, currentTime }
}

export const useGPSPosition = () => {
  const [position, setPosition] = useState<GPSPosition>({
    lat: UCF_CENTER.latitude,
    lng: UCF_CENTER.longitude,
    timestamp: Date.now()
  })

  const [path, setPath] = useState<[number, number][]>([])
  const [pathIndex, setPathIndex] = useState(0)

  useEffect(() => {
    const generatePath = async () => {
        // Path from Student Union (Center) to Engineering II area
        const start = { latitude: UCF_CENTER.latitude, longitude: UCF_CENTER.longitude }
        const goal = { latitude: 28.6016, longitude: -81.1987 }

        try {
          const pathNodes = await findPath(start, goal)
          if (pathNodes.length > 0) {
              const coords = pathNodes.map(n => [n.latitude, n.longitude] as [number, number])
              setPath(coords)
          }
        } catch (e) {
          console.error("Failed to generate path", e)
        }
    }

    generatePath()
  }, [])

  useEffect(() => {
    if (path.length === 0) return

    const interval = setInterval(() => {
      setPathIndex(prev => {
         // Loop the path back and forth or just restart?
         // Restarting from 0 for continuous demo loop
         const next = (prev + 1) % path.length
         setPosition({
            lat: path[next][0],
            lng: path[next][1],
            timestamp: Date.now()
         })
         return next
      })
    }, 500) // Update every 500ms for smoother movement

    return () => clearInterval(interval)
  }, [path])

  return { position, path }
}
