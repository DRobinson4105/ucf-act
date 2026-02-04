import React, { useEffect, useRef } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import { GPSPosition, MapRoute } from '../types'

interface MapViewProps {
  position: GPSPosition
  path: [number, number][]
}

export const MapView: React.FC<MapViewProps> = ({ position, path }) => {
  const mapRef = useRef<L.Map | null>(null)
  const markerRef = useRef<L.Marker | null>(null)
  const pathRef = useRef<L.Polyline | null>(null)

  const mapContainerRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    if (!mapRef.current && mapContainerRef.current) {
      // Initialize map
      mapRef.current = L.map(mapContainerRef.current, {
        center: [position.lat, position.lng],
        zoom: 17,
        zoomControl: false, // Tesla UI usually hides these or customizes them
        attributionControl: false,
        scrollWheelZoom: true, // Allow interaction
        doubleClickZoom: true,
        dragging: true,
        touchZoom: true
      })

      // Add dark tile layer
      L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
        attribution: '',
        maxZoom: 19
      }).addTo(mapRef.current)

      // Add current position marker
      const customIcon = L.divIcon({
        className: 'custom-marker',
        html: '<div style="background: #3b82f6; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
        iconSize: [16, 16],
        iconAnchor: [8, 8]
      })

      markerRef.current = L.marker([position.lat, position.lng], { icon: customIcon })
        .addTo(mapRef.current)

      // Add path polyline
      pathRef.current = L.polyline(path, {
        color: '#3b82f6', // Tesla Blue
        weight: 5,
        opacity: 0.8,
        lineCap: 'round',
        lineJoin: 'round'
      }).addTo(mapRef.current)

      // Force a resize to ensuring tiles load correctly - important for layout changes
      setTimeout(() => {
          mapRef.current?.invalidateSize()
      }, 100)
    }

    // Cleanup only on unmount
    return () => {
      if (mapRef.current) {
        mapRef.current.remove()
        mapRef.current = null
        markerRef.current = null
        pathRef.current = null
      }
    }
  }, []) // Empty dependency array to run only once

  // Handle path updates
  useEffect(() => {
    if (pathRef.current && path.length > 0) {
      pathRef.current.setLatLngs(path)
      // Optional: Fit map to route when loaded
      // mapRef.current?.fitBounds(pathRef.current.getBounds(), { padding: [20, 20] })
    }
  }, [path])

  // Separate effect for updating the view
  useEffect(() => {
     if (!mapRef.current || !markerRef.current) return

     // Update marker
     markerRef.current.setLatLng([position.lat, position.lng])

     // Update map center smoothly
     mapRef.current.panTo([position.lat, position.lng], { animate: true, duration: 1 })

  }, [position]) // Only run when position changes

  return (
    <div className="w-full h-full bg-gray-900">
      <div ref={mapContainerRef} className="w-full h-full" style={{ background: '#111' }} />
    </div>
  )
}
