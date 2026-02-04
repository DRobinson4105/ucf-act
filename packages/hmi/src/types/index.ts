export interface Zone {
  id: number
  position: [number, number, number]
  rotation: [number, number, number]
  size: [number, number, number]
  status: 'clear' | 'detected'
  objectType?: 'person' | 'bike' | 'car'
}

export interface VehicleStatus {
  battery: number
  speed: number
  connectivity: 'excellent' | 'good' | 'poor' | 'none'
  isConnected: boolean
}

export interface GPSPosition {
  lat: number
  lng: number
  timestamp: number
}

export interface MapRoute {
  id: string
  name: string
  path: [number, number][]
  color: string
}

export type ControlPanelTab = 'climate' | 'settings' | 'music' | 'none'
