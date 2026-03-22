import React, { useMemo } from 'react'
import { ArrowUp, ArrowUpLeft, ArrowUpRight, CornerUpLeft, CornerUpRight } from 'lucide-react'
import { Waypoint } from '../hooks/useVehicleStatus'

interface RouteInstructionsProps {
  waypoints: Waypoint[]
  currentLat: number
  currentLng: number
}

// Haversine distance in feet between two WGS84 points
function distanceFt(lat1: number, lon1: number, lat2: number, lon2: number): number {
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

// Bearing from point 1 → point 2 in degrees (0 = North, CW)
function bearing(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const phi1 = (lat1 * Math.PI) / 180
  const phi2 = (lat2 * Math.PI) / 180
  const dLam = ((lon2 - lon1) * Math.PI) / 180
  const y = Math.sin(dLam) * Math.cos(phi2)
  const x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLam)
  return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360
}

// Signed delta between two bearings: positive = right turn, negative = left turn
function bearingDelta(b1: number, b2: number): number {
  let d = ((b2 - b1 + 540) % 360) - 180
  return d
}

type TurnType = 'straight' | 'slight-left' | 'slight-right' | 'turn-left' | 'turn-right' | 'sharp-left' | 'sharp-right'

interface Instruction {
  type: TurnType
  distanceFt: number
}

function computeInstruction(waypoints: Waypoint[], currentLat: number, currentLng: number): Instruction | null {
  if (waypoints.length < 2) return null

  // Find closest waypoint to current position
  let closestIdx = 0
  let minDist = Infinity
  for (let i = 0; i < waypoints.length; i++) {
    const d = distanceFt(currentLat, currentLng, waypoints[i].latitude, waypoints[i].longitude)
    if (d < minDist) {
      minDist = d
      closestIdx = i
    }
  }

  // Walk forward from closest point, accumulating distance until next significant turn
  let cumDist = 0
  let prevBearing: number | null = null

  for (let i = closestIdx; i < waypoints.length - 1; i++) {
    const a = waypoints[i]
    const b = waypoints[i + 1]
    const segDist = distanceFt(a.latitude, a.longitude, b.latitude, b.longitude)
    cumDist += segDist

    const segBearing = bearing(a.latitude, a.longitude, b.latitude, b.longitude)

    if (prevBearing !== null) {
      const delta = bearingDelta(prevBearing, segBearing)
      const absDelta = Math.abs(delta)

      let type: TurnType | null = null
      if (absDelta >= 100) {
        type = delta > 0 ? 'sharp-right' : 'sharp-left'
      } else if (absDelta >= 45) {
        type = delta > 0 ? 'turn-right' : 'turn-left'
      } else if (absDelta >= 20) {
        type = delta > 0 ? 'slight-right' : 'slight-left'
      }

      if (type) {
        return { type, distanceFt: cumDist }
      }
    }

    prevBearing = segBearing
  }

  // No turn found — continue straight
  const remaining = waypoints.slice(closestIdx).reduce((acc, wp, i, arr) => {
    if (i === 0) return acc
    return acc + distanceFt(arr[i - 1].latitude, arr[i - 1].longitude, wp.latitude, wp.longitude)
  }, 0)

  return { type: 'straight', distanceFt: remaining }
}

function formatDistance(ft: number): string {
  if (ft < 500) return `${Math.round(ft / 10) * 10} ft`
  return `${(ft / 5280).toFixed(1)} mi`
}

function InstructionIcon({ type }: { type: TurnType }) {
  const cls = 'w-8 h-8'
  switch (type) {
    case 'turn-left':
    case 'sharp-left':
      return <CornerUpLeft className={cls} />
    case 'turn-right':
    case 'sharp-right':
      return <CornerUpRight className={cls} />
    case 'slight-left':
      return <ArrowUpLeft className={cls} />
    case 'slight-right':
      return <ArrowUpRight className={cls} />
    default:
      return <ArrowUp className={cls} />
  }
}

function instructionLabel(type: TurnType): string {
  switch (type) {
    case 'sharp-left':  return 'Sharp left'
    case 'turn-left':   return 'Turn left'
    case 'slight-left': return 'Slight left'
    case 'sharp-right': return 'Sharp right'
    case 'turn-right':  return 'Turn right'
    case 'slight-right':return 'Slight right'
    default:            return 'Continue straight'
  }
}

export const RouteInstructions: React.FC<RouteInstructionsProps> = ({
  waypoints,
  currentLat,
  currentLng,
}) => {
  const instruction = useMemo(
    () => computeInstruction(waypoints, currentLat, currentLng),
    [waypoints, currentLat, currentLng],
  )

  if (!instruction || waypoints.length === 0) return null

  return (
    <div className="absolute top-0 left-0 right-0 z-20 mx-4 mt-4">
      <div className="bg-[#1a1a1a]/90 backdrop-blur-md border border-gray-700 rounded-2xl px-5 py-3 flex items-center gap-4 shadow-2xl">
        <div className="text-blue-400 flex-shrink-0">
          <InstructionIcon type={instruction.type} />
        </div>
        <div className="flex flex-col min-w-0">
          <span className="text-white font-semibold text-lg leading-tight">
            {instructionLabel(instruction.type)}
          </span>
          <span className="text-gray-400 text-sm">
            in {formatDistance(instruction.distanceFt)}
          </span>
        </div>
      </div>
    </div>
  )
}
