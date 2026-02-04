import React from 'react'
import {
  Thermometer,
  Settings,
  Music,
  Volume2,
  Sun,
  Snowflake,
  Wind,
  Sliders,
  Play,
  Pause,
  SkipBack,
  SkipForward
} from 'lucide-react'
import { ControlPanelTab } from '../types'

interface ControlDockProps {
  activeTab: ControlPanelTab
  onTabChange: (tab: ControlPanelTab) => void
}

const ControlButton: React.FC<{
  icon: React.ReactNode
  label: string
  isActive: boolean
  onClick: () => void
}> = ({ icon, label, isActive, onClick }) => {
  return (
    <button
      onClick={onClick}
      className={`
        flex flex-col items-center justify-center space-y-2 p-4 rounded-xl transition-all duration-200
        ${isActive
          ? 'bg-primary text-white shadow-lg'
          : 'bg-surfaceLight text-text-secondary hover:bg-surface hover:text-text-primary'
        }
      `}
    >
      {icon}
      <span className="text-xs font-medium">{label}</span>
    </button>
  )
}

const ClimatePanel: React.FC = () => {
  return (
    <div className="p-6 space-y-4">
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Thermometer className="w-5 h-5 text-primary" />
          <span className="text-lg font-semibold">72°F</span>
        </div>
        <div className="flex items-center space-x-2">
          <Wind className="w-4 h-4 text-text-secondary" />
          <span className="text-sm text-text-secondary">Auto</span>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-3">
        <button className="p-3 rounded-lg bg-surfaceLight hover:bg-surface transition-colors">
          <Sun className="w-5 h-5 mx-auto mb-1 text-warning" />
          <span className="text-xs">Heat</span>
        </button>
        <button className="p-3 rounded-lg bg-surfaceLight hover:bg-surface transition-colors">
          <Snowflake className="w-5 h-5 mx-auto mb-1 text-primary" />
          <span className="text-xs">Cool</span>
        </button>
        <button className="p-3 rounded-lg bg-surfaceLight hover:bg-surface transition-colors">
          <Wind className="w-5 h-5 mx-auto mb-1 text-text-secondary" />
          <span className="text-xs">Fan</span>
        </button>
      </div>
    </div>
  )
}

const SettingsPanel: React.FC = () => {
  return (
    <div className="p-6 space-y-4">
      <h3 className="text-lg font-semibold mb-4">Vehicle Settings</h3>

      <div className="space-y-3">
        <div className="flex items-center justify-between p-3 rounded-lg bg-surfaceLight">
          <span className="text-sm">Autonomous Mode</span>
          <div className="w-12 h-6 bg-primary rounded-full relative">
            <div className="w-5 h-5 bg-white rounded-full absolute right-0.5 top-0.5 transition-transform" />
          </div>
        </div>

        <div className="flex items-center justify-between p-3 rounded-lg bg-surfaceLight">
          <span className="text-sm">Collision Avoidance</span>
          <div className="w-12 h-6 bg-primary rounded-full relative">
            <div className="w-5 h-5 bg-white rounded-full absolute right-0.5 top-0.5 transition-transform" />
          </div>
        </div>

        <div className="flex items-center justify-between p-3 rounded-lg bg-surfaceLight">
          <span className="text-sm">Emergency Stop</span>
          <button className="px-4 py-2 bg-danger text-white rounded-lg text-sm font-medium">
            STOP
          </button>
        </div>
      </div>
    </div>
  )
}

const MusicPanel: React.FC = () => {
  return (
    <div className="p-6 space-y-4">
      <div className="text-center">
        <h3 className="text-lg font-semibold mb-1">Now Playing</h3>
        <p className="text-sm text-text-secondary mb-1">Ambient Drive</p>
        <p className="text-xs text-text-muted">Autonomous Focus</p>
      </div>

      <div className="flex items-center justify-center space-x-4">
        <button className="p-2 rounded-full bg-surfaceLight hover:bg-surface transition-colors">
          <SkipBack className="w-4 h-4" />
        </button>
        <button className="p-3 rounded-full bg-primary text-white">
          <Play className="w-5 h-5" />
        </button>
        <button className="p-2 rounded-full bg-surfaceLight hover:bg-surface transition-colors">
          <SkipForward className="w-4 h-4" />
        </button>
      </div>

      <div className="flex items-center space-x-3">
        <Volume2 className="w-4 h-4 text-text-secondary" />
        <div className="flex-1 h-2 bg-surfaceLight rounded-full overflow-hidden">
          <div className="w-3/4 h-full bg-primary rounded-full" />
        </div>
        <span className="text-xs text-text-muted">75%</span>
      </div>
    </div>
  )
}

export const ControlDock: React.FC<ControlDockProps> = ({ activeTab, onTabChange }) => {
  const renderActivePanel = () => {
    switch (activeTab) {
      case 'climate':
        return <ClimatePanel />
      case 'settings':
        return <SettingsPanel />
      case 'music':
        return <MusicPanel />
      default:
        return null
    }
  }

  return (
    <div className="bento-card overflow-hidden">
      {/* Control Buttons */}
      <div className="p-4 border-b border-border">
        <div className="grid grid-cols-3 gap-4">
          <ControlButton
            icon={<Thermometer className="w-6 h-6" />}
            label="Climate"
            isActive={activeTab === 'climate'}
            onClick={() => onTabChange(activeTab === 'climate' ? 'none' : 'climate')}
          />
          <ControlButton
            icon={<Settings className="w-6 h-6" />}
            label="Settings"
            isActive={activeTab === 'settings'}
            onClick={() => onTabChange(activeTab === 'settings' ? 'none' : 'settings')}
          />
          <ControlButton
            icon={<Music className="w-6 h-6" />}
            label="Music"
            isActive={activeTab === 'music'}
            onClick={() => onTabChange(activeTab === 'music' ? 'none' : 'music')}
          />
        </div>
      </div>

      {/* Active Panel */}
      {activeTab !== 'none' && (
        <div className="border-b border-border">
          {renderActivePanel()}
        </div>
      )}
    </div>
  )
}
