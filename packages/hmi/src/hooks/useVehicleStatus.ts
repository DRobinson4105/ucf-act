import { useState, useEffect, useMemo, useRef } from 'react'
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

  // Hold last-known values so a brief undefined during Convex reactive
  // update cycles doesn't flash speed/heading/battery to 0.
  const lastStatusRef = useRef<VehicleStatus>({
    battery: 0,
    speed: 0,
    heading: 0,
    connectivity: 'none',
    isConnected: false,
  })

  const status = useMemo<VehicleStatus>(() => {
    if (cart === undefined) return lastStatusRef.current
    const next: VehicleStatus = {
      battery: cart?.batteryLevel ?? lastStatusRef.current.battery,
      speed: cart?.speed ?? lastStatusRef.current.speed,
      heading: cart?.location?.heading ?? lastStatusRef.current.heading,
      connectivity: cart ? 'excellent' : 'none',
      isConnected: !!cart,
    }
    lastStatusRef.current = next
    return next
  }, [cart])

  // Hold last-known position — never flick to null mid-session.
  const lastPositionRef = useRef<GPSPosition | null>(null)

  const position = useMemo<GPSPosition | null>(() => {
    if (cart === undefined) return lastPositionRef.current
    if (!cart?.location?.latitude) return lastPositionRef.current
    const next: GPSPosition = {
      lat: cart.location.latitude,
      lng: cart.location.longitude,
      timestamp: cart.lastUpdated,
    }
    lastPositionRef.current = next
    return next
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
