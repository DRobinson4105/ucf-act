import React from 'react'
import {
  Battery,
  Wifi,
  WifiOff,
  Clock,
  Gauge,
  Signal,
  SignalLow,
  SignalMedium
} from 'lucide-react'
import { VehicleStatus } from '../types'

interface StatusBarProps {
  status: VehicleStatus
  currentTime: Date
}

export const StatusBar: React.FC<StatusBarProps> = ({ status, currentTime }) => {
  const getConnectivityIcon = () => {
    if (!status.isConnected) return <WifiOff className="w-4 h-4 text-danger" />

    switch (status.connectivity) {
      case 'excellent':
        return <Signal className="w-4 h-4 text-success" />
      case 'good':
        return <SignalMedium className="w-4 h-4 text-warning" />
      case 'poor':
        return <SignalLow className="w-4 h-4 text-danger" />
      default:
        return <WifiOff className="w-4 h-4 text-danger" />
    }
  }

  const getBatteryColor = () => {
    if (status.battery > 50) return 'text-success'
    if (status.battery > 20) return 'text-warning'
    return 'text-danger'
  }

  const formatTime = (date: Date) => {
    return date.toLocaleTimeString('en-US', {
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    })
  }

  return (
    <div className="bento-card p-4">
      <div className="flex items-center justify-between">
        {/* Battery Status */}
        <div className="flex items-center space-x-2">
          <Battery className={`w-5 h-5 ${getBatteryColor()}`} />
          <span className={`text-lg font-semibold ${getBatteryColor()}`}>
            {status.battery.toFixed(0)}%
          </span>
        </div>

        {/* Connectivity */}
        <div className="flex items-center space-x-2">
          {getConnectivityIcon()}
          <span className="text-sm text-text-secondary capitalize">
            {status.isConnected ? status.connectivity : 'Disconnected'}
          </span>
        </div>

        {/* Current Time */}
        <div className="flex items-center space-x-2">
          <Clock className="w-4 h-4 text-text-secondary" />
          <span className="text-lg font-mono font-semibold">
            {formatTime(currentTime)}
          </span>
        </div>

        {/* Speed */}
        <div className="flex items-center space-x-2">
          <Gauge className="w-5 h-5 text-primary" />
          <span className="text-lg font-semibold">
            {status.speed.toFixed(0)}
          </span>
          <span className="text-sm text-text-muted">MPH</span>
        </div>
      </div>
    </div>
  )
}
