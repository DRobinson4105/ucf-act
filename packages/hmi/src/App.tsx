import { Scene3D } from './components/Scene3D'
import { MapView } from './components/MapView'
import { RouteInstructions } from './components/RouteInstructions'
import { useCartData } from './hooks/useVehicleStatus'
import { Battery, Wifi, WifiOff, Music, SkipBack, SkipForward, Play } from 'lucide-react'

function App() {
  const { status, position, path, waypoints, currentTime } = useCartData()

  return (
    <div className="h-full w-full bg-black text-white overflow-hidden flex flex-row font-sans">
      {/* LEFT PANEL: Vehicle Visualization & Status */}
      <div className="w-1/2 h-full relative border-r border-gray-800 bg-gradient-to-b from-[#1a1a1a] to-[#000000]">

        {/* Route instruction banner — overlaid at top */}
        <RouteInstructions
          waypoints={waypoints}
          currentLat={position.lat}
          currentLng={position.lng}
        />

        {/* Speed + gear overlay (below instruction banner) */}
        <div className="absolute top-0 left-0 w-full p-4 z-10 flex justify-between items-start pt-20">
          <div className="flex flex-col">
            <span className="text-6xl font-medium tracking-tighter">{Math.round(status.speed)}</span>
            <span className="text-sm text-gray-400 font-medium uppercase tracking-widest pl-1">MPH</span>
          </div>

          <div className="flex items-center gap-3">
            <div className="flex items-center gap-1 text-gray-400">
              <span className="text-sm font-medium">PRND</span>
              <span className="text-blue-500 font-bold ml-1">D</span>
            </div>
          </div>
        </div>

        {/* 3D Scene */}
        <div className="w-full h-full">
          <Scene3D speed={status.speed} />
        </div>

        {/* Battery / connectivity */}
        <div className="absolute bottom-20 left-0 w-full px-6 py-4 flex justify-between items-end z-10">
          <div className="flex flex-col gap-1">
            <div className="flex items-center gap-2 text-green-400">
              <Battery size={24} fill="currentColor" />
              <span className="text-xl font-medium">{Math.floor(status.battery)}%</span>
            </div>
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

      {/* RIGHT PANEL: Navigation Map */}
      <div className="w-1/2 h-full relative bg-gray-900">

        {/* Map */}
        <div className="absolute inset-0 z-0">
          <MapView position={position} path={path} />
        </div>

        {/* Top bar: time + connectivity */}
        <div className="absolute top-0 right-0 w-full p-4 z-10 flex justify-end items-center gap-4 bg-gradient-to-b from-black/50 to-transparent pointer-events-none">
          <div className="flex items-center gap-2 text-white font-medium drop-shadow-md">
            <span>72°F</span>
            <span>{currentTime.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
            <Wifi size={16} />
          </div>
        </div>

        {/* Trip info card */}
        <div className="absolute top-12 left-6 w-80 bg-[#1a1a1a] rounded-xl shadow-2xl border border-gray-800 p-4 flex flex-col gap-3">
          <div className="flex items-start gap-3">
            <div className="bg-blue-600 rounded-full p-1 mt-1">
              <div className="w-2 h-2 bg-white rounded-full"></div>
            </div>
            <div>
              <h3 className="font-semibold text-white">
                {status.isConnected ? 'ACT-001 · Live' : 'Connecting…'}
              </h3>
              <p className="text-sm text-gray-400">
                {waypoints.length > 0
                  ? `Route active · ${waypoints.length} waypoints`
                  : 'No active route'}
              </p>
            </div>
          </div>
        </div>

        {/* Music dock */}
        <div className="absolute bottom-8 right-8 w-96 bg-[#1a1a1a]/90 backdrop-blur-md rounded-xl shadow-2xl border border-gray-800 p-4 flex items-center gap-4">
          <div className="w-12 h-12 bg-gray-700 rounded-lg flex items-center justify-center">
            <Music size={20} className="text-gray-400" />
          </div>
          <div className="flex-1 overflow-hidden">
            <h4 className="font-medium text-white truncate">Usseewa</h4>
            <p className="text-xs text-gray-400 truncate">Ado · Kyogen</p>
          </div>
          <div className="flex items-center gap-3">
            <SkipBack size={20} className="text-gray-400" />
            <Play size={24} fill="currentColor" className="text-white" />
            <SkipForward size={20} className="text-gray-400" />
          </div>
        </div>
      </div>
    </div>
  )
}

export default App
