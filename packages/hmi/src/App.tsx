import { Scene3D } from './components/Scene3D'
import { MapView } from './components/MapView'
import { RouteInstructions } from './components/RouteInstructions'
import { SpotifyPlayer } from './components/SpotifyPlayer'
import { useCartData } from './hooks/useVehicleStatus'
import {
  Battery, Wifi, WifiOff,
  MapPin, Navigation, Clock, CheckCircle2,
} from 'lucide-react'

// ─── Ride phase helpers ────────────────────────────────────────────────────────

type RideStatus = 'assigned' | 'arriving' | 'in_progress' | null

function phaseLabel(rideStatus: RideStatus): string {
  switch (rideStatus) {
    case 'assigned':    return 'Heading to pickup'
    case 'arriving':    return 'Awaiting passenger'
    case 'in_progress': return 'En route'
    default:            return 'Idle'
  }
}

function phaseDotColor(rideStatus: RideStatus): string {
  switch (rideStatus) {
    case 'assigned':    return 'bg-yellow-400'
    case 'arriving':    return 'bg-green-400'
    case 'in_progress': return 'bg-blue-500'
    default:            return 'bg-gray-500'
  }
}

function PhaseIcon({ status }: { status: RideStatus }) {
  const cls = 'shrink-0'
  switch (status) {
    case 'assigned':    return <Navigation size={16} className={`${cls} text-yellow-400`} />
    case 'arriving':    return <Clock      size={16} className={`${cls} text-green-400`} />
    case 'in_progress': return <MapPin     size={16} className={`${cls} text-blue-400`} />
    default:            return <CheckCircle2 size={16} className={`${cls} text-gray-500`} />
  }
}

// ─── App ──────────────────────────────────────────────────────────────────────

function App() {
  const { status, position, path, waypoints, currentTime, ride } = useCartData()

  const rideStatus: RideStatus = (ride?.status as RideStatus) ?? null
  const originName      = ride?.origin?.name      ?? 'Pickup'
  const destinationName = ride?.destination?.name ?? 'Destination'

  // What to show in the route destination sub-label
  const routeTarget =
    rideStatus === 'in_progress' ? destinationName :
    rideStatus === 'assigned'    ? originName :
    null

  // Show the "arriving" banner instead of turn-by-turn when cart is waiting
  const showArrivalBanner = rideStatus === 'arriving'

  return (
    <div className="h-full w-full bg-black text-white overflow-hidden flex flex-row font-sans">

      {/* ── LEFT PANEL: 3D visualization ─────────────────────────────────── */}
      <div className="w-1/2 h-full relative border-r border-gray-800 bg-gradient-to-b from-[#1a1a1a] to-[#000000]">

        {/* Turn-by-turn banner OR arrival banner */}
        {showArrivalBanner ? (
          <div className="absolute top-0 left-0 right-0 z-20 mx-4 mt-4">
            <div className="bg-green-900/80 backdrop-blur-md border border-green-700 rounded-2xl px-5 py-3 flex items-center gap-4 shadow-2xl">
              <Clock size={28} className="text-green-400 shrink-0" />
              <div>
                <p className="text-white font-semibold text-lg leading-tight">Awaiting passenger</p>
                <p className="text-green-300 text-sm">At {originName}</p>
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

        {/* Speed + gear overlay */}
        <div className="absolute top-0 left-0 w-full p-4 z-10 flex justify-between items-start pt-20">
          <div className="flex flex-col">
            <span className="text-6xl font-medium tracking-tighter">{Math.round(status.speed)}</span>
            <span className="text-sm text-gray-400 font-medium uppercase tracking-widest pl-1">MPH</span>
          </div>

          <div className="flex items-center gap-3">
            <div className="flex items-center gap-1 text-gray-400">
              <span className="text-sm font-medium">PRND</span>
              <span className="text-blue-500 font-bold ml-1">
                {rideStatus === null ? 'P' : 'D'}
              </span>
            </div>
          </div>
        </div>

        {/* 3D Scene */}
        <div className="w-full h-full">
          <Scene3D speed={status.speed} />
        </div>

        {/* Battery / connectivity */}
        <div className="absolute bottom-20 left-0 w-full px-6 py-4 flex justify-between items-end z-10">
          <div className="flex items-center gap-2 text-green-400">
            <Battery size={24} fill="currentColor" />
            <span className="text-xl font-medium">{Math.floor(status.battery)}%</span>
          </div>

          <div className="flex items-center gap-2 text-gray-400">
            {status.isConnected
              ? <Wifi size={18} className="text-green-400" />
              : <WifiOff size={18} className="text-red-400" />
            }
            <span className="text-xs">{status.connectivity}</span>
          </div>
        </div>
      </div>

      {/* ── RIGHT PANEL: Map ──────────────────────────────────────────────── */}
      <div className="w-1/2 h-full relative bg-gray-900">

        {/* Map */}
        <div className="absolute inset-0 z-0">
          <MapView position={position} path={path} />
        </div>

        {/* Top bar: time */}
        <div className="absolute top-0 right-0 w-full p-4 z-10 flex justify-end items-center gap-4 bg-gradient-to-b from-black/50 to-transparent pointer-events-none">
          <div className="flex items-center gap-2 text-white font-medium drop-shadow-md">
            <span>72°F</span>
            <span>{currentTime.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
            <Wifi size={16} />
          </div>
        </div>

        {/* Trip info card */}
        <div className="absolute top-12 left-6 w-80 bg-[#1a1a1a] rounded-xl shadow-2xl border border-gray-800 p-4 flex flex-col gap-2">
          {/* Status row */}
          <div className="flex items-center gap-3">
            <div className={`w-2.5 h-2.5 rounded-full shrink-0 ${phaseDotColor(rideStatus)}`} />
            <div className="flex items-center gap-2 min-w-0">
              <PhaseIcon status={rideStatus} />
              <h3 className="font-semibold text-white text-sm truncate">
                {status.isConnected ? phaseLabel(rideStatus) : 'Connecting…'}
              </h3>
            </div>
          </div>

          {/* Route detail — origin → destination when ride is active */}
          {rideStatus !== null && (
            <div className="flex flex-col gap-1 pl-5 border-l border-gray-700 ml-1">
              {rideStatus === 'in_progress' ? (
                <>
                  <div className="flex items-center gap-2 text-xs text-gray-400">
                    <div className="w-1.5 h-1.5 rounded-full bg-green-400" />
                    <span className="truncate">{originName}</span>
                  </div>
                  <div className="flex items-center gap-2 text-xs text-white font-medium">
                    <div className="w-1.5 h-1.5 rounded-sm bg-blue-400" />
                    <span className="truncate">{destinationName}</span>
                  </div>
                </>
              ) : (
                <div className="flex items-center gap-2 text-xs text-gray-300">
                  <MapPin size={11} className="text-yellow-400 shrink-0" />
                  <span className="truncate">{routeTarget}</span>
                </div>
              )}
            </div>
          )}

          {/* Waypoint count when route is active */}
          {waypoints.length > 0 && (
            <p className="text-xs text-gray-500 pl-5">
              {waypoints.length} waypoints
            </p>
          )}

          {/* No ride state */}
          {rideStatus === null && (
            <p className="text-xs text-gray-500 pl-5">No active ride</p>
          )}
        </div>

        <SpotifyPlayer />
      </div>
    </div>
  )
}

export default App
