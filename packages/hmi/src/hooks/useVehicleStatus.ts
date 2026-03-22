import { useState, useEffect } from 'react'
import { useQuery } from 'convex/react'
import { api } from '@convex-api'
import { Id } from '@convex-types'
import { VehicleStatus, GPSPosition } from '../types'
import { UCF_CENTER } from '../constants/campus-locations'

const CART_ID = import.meta.env.VITE_CART_ID as string | undefined

export interface Waypoint {
  latitude: number
  longitude: number
}

export const useCartData = () => {
  const cart = useQuery(
    api.carts.getById,
    CART_ID ? { cartId: CART_ID as Id<'carts'> } : 'skip',
  )

  const [currentTime, setCurrentTime] = useState(new Date())

  useEffect(() => {
    const interval = setInterval(() => setCurrentTime(new Date()), 1000)
    return () => clearInterval(interval)
  }, [])

  const status: VehicleStatus = {
    battery: cart?.batteryLevel ?? 0,
    speed: cart?.speed ?? 0,
    connectivity: cart ? 'excellent' : 'none',
    isConnected: !!cart,
  }

  const position: GPSPosition = {
    lat: cart?.location.latitude ?? UCF_CENTER.latitude,
    lng: cart?.location.longitude ?? UCF_CENTER.longitude,
    timestamp: cart?.lastUpdated ?? Date.now(),
  }

  const waypoints: Waypoint[] = cart?.currentRoute ?? []
  const path: [number, number][] = waypoints.map(w => [w.latitude, w.longitude])

  return { status, position, path, waypoints, currentTime }
}

// Keep legacy exports so nothing else breaks if still imported
export const useVehicleStatus = () => {
  const { status, currentTime } = useCartData()
  return { status, currentTime }
}

export const useGPSPosition = () => {
  const { position, path } = useCartData()
  return { position, path }
}
