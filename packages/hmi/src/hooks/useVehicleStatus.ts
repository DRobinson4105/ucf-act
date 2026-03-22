import { useState, useEffect, useMemo } from 'react'
import { useQuery } from 'convex/react'
import { api } from '@convex-api'
import { Id } from '@convex-types'
import { VehicleStatus, GPSPosition } from '../types'

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

  const ride = useQuery(
    api.rides.getActiveRideForCart,
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

  // Memoize so the object reference only changes when the actual coordinates change,
  // not on every App re-render (e.g. the 1-second currentTime tick).
  const position = useMemo<GPSPosition | null>(() => {
    if (!cart?.location?.latitude) return null
    return {
      lat: cart.location.latitude,
      lng: cart.location.longitude,
      timestamp: cart.lastUpdated,
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cart?.location?.latitude, cart?.location?.longitude, cart?.lastUpdated])

  const waypoints: Waypoint[] = cart?.currentRoute ?? []

  // Stable array reference — only recreate when the route actually changes
  const path = useMemo<[number, number][]>(
    () => waypoints.map(w => [w.latitude, w.longitude]),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [cart?.currentRoute],
  )

  return { status, position, path, waypoints, currentTime, ride }
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
