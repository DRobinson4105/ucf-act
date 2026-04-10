import React, { useEffect, useRef } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import { GPSPosition } from '../types'
import { UCF_CENTER } from '../constants/campus-locations'

interface MapViewProps {
  position: GPSPosition | null
  path: [number, number][]
  heading?: number
}

// Teal accent matching the mobile app
const ACCENT = '#2DD4BF'

// Build the icon HTML once — the arrow rotation is handled via CSS class
function buildMarkerIcon(): L.DivIcon {
  return L.divIcon({
    className: 'custom-marker',
    html: `<div style="
      width: 28px;
      height: 28px;
      border-radius: 50%;
      background: ${ACCENT};
      border: 3px solid white;
      box-shadow: 0 2px 8px rgba(0,0,0,0.4);
      display: flex;
      align-items: center;
      justify-content: center;
    ">
      <div class="heading-arrow" style="
        width: 0;
        height: 0;
        border-left: 5px solid transparent;
        border-right: 5px solid transparent;
        border-bottom: 8px solid white;
        margin-top: -1px;
        transition: transform 0.4s ease;
      "></div>
    </div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  })
}

export const MapView: React.FC<MapViewProps> = ({ position, path, heading = 0 }) => {
  const mapRef = useRef<L.Map | null>(null)
  const markerRef = useRef<L.Marker | null>(null)
  const pathRef = useRef<L.Polyline | null>(null)
  const mapContainerRef = useRef<HTMLDivElement>(null)
  const lastPanRef = useRef<{ lat: number; lng: number } | null>(null)

  // Initialize map once — centered at UCF until real cart GPS arrives
  useEffect(() => {
    if (!mapRef.current && mapContainerRef.current) {
      mapRef.current = L.map(mapContainerRef.current, {
        center: [UCF_CENTER.latitude, UCF_CENTER.longitude],
        zoom: 17,
        zoomControl: false,
        attributionControl: false,
        scrollWheelZoom: true,
        doubleClickZoom: true,
        dragging: true,
        touchZoom: true
      })

      L.tileLayer('https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png', {
        attribution: '',
        maxZoom: 19
      }).addTo(mapRef.current)

      pathRef.current = L.polyline(path, {
        color: ACCENT,
        weight: 4,
        opacity: 0.8,
        lineCap: 'round',
        lineJoin: 'round'
      }).addTo(mapRef.current)

      setTimeout(() => { mapRef.current?.invalidateSize() }, 100)
    }

    return () => {
      if (mapRef.current) {
        mapRef.current.remove()
        mapRef.current = null
        markerRef.current = null
        pathRef.current = null
      }
    }
  }, [])

  // Handle path updates
  useEffect(() => {
    if (!pathRef.current) return
    pathRef.current.setLatLngs(path.length > 0 ? path : [])
  }, [path])

  // Follow cart GPS — marker created once, then moved
  useEffect(() => {
    if (!mapRef.current || !position) return

    // Skip if coordinates haven't actually changed
    const last = lastPanRef.current
    if (last && last.lat === position.lat && last.lng === position.lng) return
    lastPanRef.current = { lat: position.lat, lng: position.lng }

    if (!markerRef.current) {
      // First real GPS fix — create marker once and set view
      markerRef.current = L.marker([position.lat, position.lng], { icon: buildMarkerIcon() })
        .addTo(mapRef.current)
      mapRef.current.setView([position.lat, position.lng], 17, { animate: false })
    } else {
      markerRef.current.setLatLng([position.lat, position.lng])
      mapRef.current.panTo([position.lat, position.lng], { animate: true, duration: 1 })
    }
  }, [position])

  // Rotate the heading arrow via CSS transform — no icon recreation
  useEffect(() => {
    if (!markerRef.current) return
    const el = markerRef.current.getElement()
    if (!el) return
    const arrow = el.querySelector('.heading-arrow') as HTMLElement | null
    if (arrow) {
      arrow.style.transform = `rotate(${heading}deg)`
    }
  }, [heading])

  return (
    <div className="w-full h-full" style={{ background: '#e8e5df' }}>
      <div ref={mapContainerRef} className="w-full h-full" style={{ background: '#e8e5df' }} />
    </div>
  )
}
