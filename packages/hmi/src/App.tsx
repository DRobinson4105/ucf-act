import { useMemo } from 'react'
import { Scene3D } from './components/Scene3D'
import { MapView } from './components/MapView'
import { RouteInstructions } from './components/RouteInstructions'
import { useCartData, Waypoint } from './hooks/useVehicleStatus'
import {
  Battery, Wifi, WifiOff, Music, SkipBack, SkipForward, Play,
  Navigation, Clock, Compass,
} from 'lucide-react'

// ─── Ride phase helpers ────────────────────────────────────────────────────────

type RideStatus = 'assigned' | 'arriving' | 'in_progress' | null

function phaseLabel(rideStatus: RideStatus): string {
  switch (rideStatus) {
    case 'assigned':    return 'Cart on the way'
    case 'arriving':    return 'Cart has arrived!'
    case 'in_progress': return 'On your way'
    default:            return 'Idle'
  }
}

function phaseSub(rideStatus: RideStatus): string {
  switch (rideStatus) {
    case 'assigned':    return 'Heading to pickup location'
    case 'arriving':    return 'Waiting for passenger to board'
    case 'in_progress': return 'En route to destination'
    default:            return 'Waiting for ride assignment'
  }
}

function phaseDotColor(rideStatus: RideStatus): string {
  switch (rideStatus) {
    case 'assigned':    return 'bg-primary'
    case 'arriving':    return 'bg-success'
    case 'in_progress': return 'bg-primary'
    default:            return 'bg-text-secondary'
  }
}

// ─── Speed deceleration near route endpoints ─────────────────────────────────

const DECEL_ZONE_FT = 150 // start decelerating within 150 ft of destination

function haversineFt(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const R = 20_902_231 // Earth radius in feet
  const phi1 = (lat1 * Math.PI) / 180
  const phi2 = (lat2 * Math.PI) / 180
  const dPhi = ((lat2 - lat1) * Math.PI) / 180
  const dLam = ((lon2 - lon1) * Math.PI) / 180
  const a =
    Math.sin(dPhi / 2) ** 2 +
    Math.cos(phi1) * Math.cos(phi2) * Math.sin(dLam / 2) ** 2
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
}

function computeDisplaySpeed(
  rawSpeed: number,
  position: { lat: number; lng: number } | null,
  waypoints: Waypoint[],
): number {
  if (!position || waypoints.length < 2) return rawSpeed

  const lastWp = waypoints[waypoints.length - 1]
  const distToEnd = haversineFt(position.lat, position.lng, lastWp.latitude, lastWp.longitude)

  if (distToEnd < DECEL_ZONE_FT) {
    // Smooth ease-out curve: speed * (dist / zone)^0.5
    const factor = Math.pow(Math.max(0, distToEnd / DECEL_ZONE_FT), 0.5)
    return rawSpeed * factor
  }

  return rawSpeed
}

const COMPASS_DIRS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'] as const

function compassDir(heading: number): string {
  return COMPASS_DIRS[Math.round(heading / 45) % 8]
}

// ─── App ──────────────────────────────────────────────────────────────────────

function App() {
  const { status, position, path, waypoints, currentTime, ride } = useCartData()

  const rideStatus: RideStatus = (ride?.status as RideStatus) ?? null
  const originName      = ride?.origin?.name      ?? 'Pickup'
  const destinationName = ride?.destination?.name ?? 'Destination'

  // Decelerate smoothly as cart approaches the route endpoint (pickup or dropoff)
  const displaySpeed = useMemo(
    () => computeDisplaySpeed(status.speed, position, waypoints),
    [status.speed, position, waypoints],
  )

  // Show the "arriving" banner instead of turn-by-turn when cart is waiting
  const showArrivalBanner = rideStatus === 'arriving'

  return (
    <div className="h-full w-full bg-background text-text-primary overflow-hidden flex flex-row font-sans">

      {/* ── LEFT PANEL: 3D visualization ─────────────────────────────────── */}
      <div className="w-1/2 h-full relative border-r border-border bg-gradient-to-b from-surface to-background">

        {/* Turn-by-turn banner OR arrival banner */}
        {showArrivalBanner ? (
          <div className="absolute top-0 left-0 right-0 z-20 mx-4 mt-4">
            <div className="bg-success/20 backdrop-blur-md border border-success/40 rounded-2xl px-5 py-3 flex items-center gap-4 shadow-2xl">
              <Clock size={28} className="text-success shrink-0" />
              <div>
                <p className="text-text-primary font-semibold text-lg leading-tight">Cart has arrived!</p>
                <p className="text-success text-sm">Waiting at {originName}</p>
              </div>
            </div>
          </div>
        ) : (
          <RouteInstructions
            waypoints={waypoints}
            currentLat={position?.lat ?? 0}
            currentLng={position?.lng ?? 0}
          />
        )}

        {/* Speed + heading overlay */}
        <div className="absolute top-0 left-0 w-full p-4 z-10 flex justify-between items-start pt-20">
          <div className="flex flex-col">
            <span className="text-6xl font-medium tracking-tighter text-text-primary">
              {Math.round(displaySpeed)}
            </span>
            <span className="text-sm text-text-secondary font-medium uppercase tracking-widest pl-1">MPH</span>
          </div>

          <div className="flex items-center gap-4">
            {/* Heading */}
            <div className="flex items-center gap-2 bg-surfaceLight/80 backdrop-blur-sm rounded-xl px-3 py-2 border border-border">
              <Compass size={16} className="text-primary" />
              <span className="text-sm font-semibold text-text-primary">{Math.round(status.heading)}°</span>
              <span className="text-xs text-text-secondary">{compassDir(status.heading)}</span>
            </div>

            {/* Gear */}
            <div className="flex items-center gap-1 text-text-secondary">
              <span className="text-sm font-medium">PRND</span>
              <span className="text-primary font-bold ml-1">
                {rideStatus === null ? 'P' : 'D'}
              </span>
            </div>
          </div>
        </div>

        {/* 3D Scene */}
        <div className="w-full h-full">
          <Scene3D speed={displaySpeed} />
        </div>

        {/* Battery / connectivity */}
        <div className="absolute bottom-20 left-0 w-full px-6 py-4 flex justify-between items-end z-10">
          <div className="flex items-center gap-2 text-success">
            <Battery size={24} fill="currentColor" />
            <span className="text-xl font-medium">{Math.floor(status.battery)}%</span>
          </div>

          <div className="flex items-center gap-2 text-text-secondary">
            {status.isConnected
              ? <Wifi size={18} className="text-success" />
              : <WifiOff size={18} className="text-danger" />
            }
            <span className="text-xs">{status.connectivity}</span>
          </div>
        </div>
      </div>

      {/* ── RIGHT PANEL: Map ──────────────────────────────────────────────── */}
      <div className="w-1/2 h-full relative bg-background">

        {/* Map */}
        <div className="absolute inset-0 z-0">
          <MapView position={position} path={path} heading={status.heading} />
        </div>

        {/* Top bar: time */}
        <div className="absolute top-0 right-0 w-full p-4 z-10 flex justify-end items-center gap-4 bg-gradient-to-b from-background/70 to-transparent pointer-events-none">
          <div className="flex items-center gap-2 text-text-primary font-medium drop-shadow-md">
            <span>72°F</span>
            <span>{currentTime.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
            <Wifi size={16} />
          </div>
        </div>

        {/* Trip info card — matches mobile app ride card */}
        <div className="absolute top-12 left-6 w-80 bg-surface rounded-[20px] shadow-2xl border border-border p-[18px] flex flex-col gap-1">
          {/* Status row */}
          <div className="flex items-center gap-2 mb-1">
            <div className={`w-2 h-2 rounded-full shrink-0 ${phaseDotColor(rideStatus)} ${rideStatus ? 'animate-pulse' : ''}`} />
            <h3 className="font-bold text-text-primary text-[17px] leading-tight truncate">
              {status.isConnected ? phaseLabel(rideStatus) : 'Connecting...'}
            </h3>
          </div>
          <p className="text-[13px] text-text-secondary mb-3 pl-4">
            {status.isConnected ? phaseSub(rideStatus) : 'Establishing connection'}
          </p>

          {/* Route chip — origin → destination when ride is active */}
          {rideStatus !== null && (
            <div className="flex items-center gap-1.5 bg-surfaceLight rounded-xl px-3.5 py-2.5 mb-3 border border-border">
              <div className="w-[7px] h-[7px] rounded-full bg-text-secondary shrink-0" />
              <span className="text-[13px] text-text-secondary truncate flex-1">{originName}</span>
              <span className="text-[13px] text-text-secondary mx-0.5">→</span>
              <div className="w-[7px] h-[7px] rounded-sm bg-primary shrink-0" />
              <span className="text-[13px] font-semibold text-text-primary truncate flex-1">{destinationName}</span>
            </div>
          )}

          {/* Cart info row */}
          <div className="flex items-center gap-3">
            <div className="w-11 h-11 rounded-xl bg-surfaceLight border border-border flex items-center justify-center">
              <Navigation size={18} className="text-primary" />
            </div>
            <div className="flex-1 min-w-0">
              <p className="text-[15px] font-semibold text-text-primary">ACT-001</p>
              <p className="text-[13px] text-text-secondary">Autonomous Golf Cart</p>
            </div>
          </div>

          {/* Waypoint count when route is active */}
          {waypoints.length > 0 && (
            <p className="text-xs text-text-muted pl-14 -mt-1">
              {waypoints.length} waypoints
            </p>
          )}

          {/* No ride state */}
          {rideStatus === null && (
            <p className="text-xs text-text-muted pl-14 -mt-1">No active ride</p>
          )}
        </div>

        {/* Music dock */}
        <div className="absolute bottom-8 right-8 w-96 bg-surface/90 backdrop-blur-md rounded-xl shadow-2xl border border-border p-4 flex items-center gap-4">
          <div className="w-12 h-12 bg-surfaceLight rounded-lg flex items-center justify-center border border-border">
            <Music size={20} className="text-text-secondary" />
          </div>
          <div className="flex-1 overflow-hidden">
            <h4 className="font-medium text-text-primary truncate">Usseewa</h4>
            <p className="text-xs text-text-secondary truncate">Ado · Kyogen</p>
          </div>
          <div className="flex items-center gap-3">
            <SkipBack size={20} className="text-text-secondary" />
            <Play size={24} fill="currentColor" className="text-text-primary" />
            <SkipForward size={20} className="text-text-secondary" />
          </div>
        </div>
      </div>
    </div>
  )
}

export default App
