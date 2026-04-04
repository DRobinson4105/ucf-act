import { useState, useEffect, useRef, useCallback } from 'react'
import {
  Music2, SkipBack, SkipForward, Play, Pause,
  Volume2, VolumeX, Shuffle, Repeat, Repeat1,
  LogIn, LogOut,
} from 'lucide-react'
import { useSpotify, RepeatState } from '../hooks/useSpotify'

// ─── Helpers ──────────────────────────────────────────────────────────────────

function formatMs(ms: number): string {
  const s = Math.max(0, Math.floor(ms / 1_000))
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`
}

function RepeatIcon({ state, size = 16 }: { state: RepeatState; size?: number }) {
  return state === 'track'
    ? <Repeat1 size={size} />
    : <Repeat  size={size} />
}

// ─── Component ────────────────────────────────────────────────────────────────

export function SpotifyPlayer() {
  const spotify = useSpotify()
  const { isAuthenticated, isLoading, player, login, logout } = spotify

  // Smooth progress interpolation between API polls
  const syncRef = useRef<{ progressMs: number; syncedAt: number; isPlaying: boolean }>({
    progressMs: 0, syncedAt: Date.now(), isPlaying: false,
  })
  const [displayProgress, setDisplayProgress] = useState(0)

  // Sync ref + state whenever the API gives us a new position or track
  useEffect(() => {
    if (!player) return
    syncRef.current = {
      progressMs: player.progressMs,
      syncedAt:   Date.now(),
      isPlaying:  player.isPlaying,
    }
    setDisplayProgress(player.progressMs)
  }, [player?.progressMs, player?.isPlaying, player?.track?.id]) // eslint-disable-line react-hooks/exhaustive-deps

  // Tick every 100 ms while playing to keep the bar smooth
  useEffect(() => {
    const timer = setInterval(() => {
      const { progressMs, syncedAt, isPlaying } = syncRef.current
      if (!isPlaying) return
      setDisplayProgress(progressMs + (Date.now() - syncedAt))
    }, 100)
    return () => clearInterval(timer)
  }, [])

  // Seek bar — only fire API call on pointer-up to avoid request flood
  const [isDragging,  setIsDragging]  = useState(false)
  const [dragValue,   setDragValue]   = useState(0)

  const handleSeekStart = useCallback(() => {
    setIsDragging(true)
    setDragValue(displayProgress)
  }, [displayProgress])

  const handleSeekEnd = useCallback((e: React.PointerEvent<HTMLInputElement>) => {
    const val = Number((e.target as HTMLInputElement).value)
    setIsDragging(false)
    void spotify.seek(val)
  }, [spotify])

  // Volume — local state + debounced API call
  const [localVolume, setLocalVolume] = useState(player?.volumePercent ?? 50)
  const volDebounceRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  useEffect(() => {
    setLocalVolume(player?.volumePercent ?? 50)
  }, [player?.volumePercent])

  const handleVolumeChange = useCallback((value: number) => {
    setLocalVolume(value)
    if (volDebounceRef.current) clearTimeout(volDebounceRef.current)
    volDebounceRef.current = setTimeout(() => void spotify.setVolume(value), 250)
  }, [spotify])

  // ── Shell (shared outer card) ────────────────────────────────────────────

  const card = (children: React.ReactNode) => (
    <div className="absolute bottom-8 right-8 w-96 bg-[#121212]/95 backdrop-blur-md rounded-xl shadow-2xl border border-white/10 p-4 flex flex-col gap-3">
      {children}
    </div>
  )

  // ── Unauthenticated ──────────────────────────────────────────────────────

  if (!isAuthenticated) {
    return card(
      <div className="flex items-center gap-3">
        <div className="w-11 h-11 bg-[#1DB954]/15 rounded-lg flex items-center justify-center shrink-0">
          <Music2 size={20} className="text-[#1DB954]" />
        </div>
        <div className="flex-1 min-w-0">
          <p className="text-white font-semibold text-sm">Spotify</p>
          <p className="text-gray-400 text-xs">Connect to control playback</p>
        </div>
        <button
          onClick={() => void login()}
          disabled={isLoading}
          className="flex items-center gap-1.5 bg-[#1DB954] hover:bg-[#1ed760] active:bg-[#1aa34a] text-black font-bold text-xs px-3 py-2 rounded-lg transition-colors shrink-0 disabled:opacity-50"
        >
          <LogIn size={13} />
          Connect
        </button>
      </div>
    )
  }

  // ── Authenticated but nothing playing ────────────────────────────────────

  if (!player?.track) {
    return card(
      <div className="flex items-center gap-3">
        <div className="w-11 h-11 bg-gray-800 rounded-lg flex items-center justify-center shrink-0">
          <Music2 size={20} className="text-gray-500" />
        </div>
        <div className="flex-1 min-w-0">
          <p className="text-gray-300 text-sm font-medium">No active playback</p>
          <p className="text-gray-500 text-xs">Open Spotify on a device to start</p>
        </div>
        <button
          onClick={logout}
          title="Disconnect"
          className="text-gray-600 hover:text-gray-400 transition-colors shrink-0"
        >
          <LogOut size={15} />
        </button>
      </div>
    )
  }

  // ── Full player ──────────────────────────────────────────────────────────

  const { track, isPlaying, shuffleState, repeatState } = player
  const duration        = track.durationMs
  const progressClamped = Math.min(isDragging ? dragValue : displayProgress, duration)

  return card(
    <>
      {/* ── Track info ── */}
      <div className="flex items-center gap-3">
        {track.albumArt ? (
          <img
            src={track.albumArt}
            alt={track.albumName}
            className="w-11 h-11 rounded-lg object-cover shrink-0 shadow"
          />
        ) : (
          <div className="w-11 h-11 bg-gray-800 rounded-lg flex items-center justify-center shrink-0">
            <Music2 size={18} className="text-gray-500" />
          </div>
        )}
        <div className="flex-1 min-w-0">
          <p className="text-white font-semibold text-sm leading-tight truncate">{track.name}</p>
          <p className="text-gray-400 text-xs leading-tight truncate mt-0.5">
            {track.artists} · {track.albumName}
          </p>
        </div>
        <button
          onClick={logout}
          title="Disconnect Spotify"
          className="text-gray-600 hover:text-gray-400 transition-colors shrink-0"
        >
          <LogOut size={14} />
        </button>
      </div>

      {/* ── Progress bar ── */}
      <div className="flex items-center gap-2">
        <span className="text-gray-500 text-[10px] w-8 text-right tabular-nums shrink-0">
          {formatMs(progressClamped)}
        </span>
        <input
          type="range"
          min={0}
          max={duration}
          value={progressClamped}
          onPointerDown={handleSeekStart}
          onChange={e => setDragValue(Number(e.target.value))}
          onPointerUp={handleSeekEnd}
          className="flex-1 h-1 rounded-full cursor-pointer accent-[#1DB954]"
          style={{ background: `linear-gradient(to right, #1DB954 ${(progressClamped / duration) * 100}%, #3f3f3f ${(progressClamped / duration) * 100}%)` }}
        />
        <span className="text-gray-500 text-[10px] w-8 tabular-nums shrink-0">
          {formatMs(duration)}
        </span>
      </div>

      {/* ── Playback controls ── */}
      <div className="flex items-center justify-between px-2">
        {/* Shuffle */}
        <button
          onClick={() => void spotify.toggleShuffle()}
          title="Shuffle"
          className={`transition-colors ${shuffleState ? 'text-[#1DB954]' : 'text-gray-500 hover:text-gray-300'}`}
        >
          <Shuffle size={15} />
        </button>

        {/* Previous */}
        <button
          onClick={() => void spotify.previous()}
          className="text-gray-300 hover:text-white transition-colors"
        >
          <SkipBack size={20} fill="currentColor" />
        </button>

        {/* Play / Pause */}
        <button
          onClick={() => void (isPlaying ? spotify.pause() : spotify.play())}
          className="w-10 h-10 bg-white hover:bg-gray-100 active:bg-gray-200 rounded-full flex items-center justify-center text-black transition-colors shadow"
        >
          {isPlaying
            ? <Pause size={17} fill="currentColor" />
            : <Play  size={17} fill="currentColor" className="ml-0.5" />
          }
        </button>

        {/* Next */}
        <button
          onClick={() => void spotify.next()}
          className="text-gray-300 hover:text-white transition-colors"
        >
          <SkipForward size={20} fill="currentColor" />
        </button>

        {/* Repeat */}
        <button
          onClick={() => void spotify.cycleRepeat()}
          title={`Repeat: ${repeatState}`}
          className={`transition-colors ${repeatState !== 'off' ? 'text-[#1DB954]' : 'text-gray-500 hover:text-gray-300'}`}
        >
          <RepeatIcon state={repeatState} size={15} />
        </button>
      </div>

      {/* ── Volume ── */}
      <div className="flex items-center gap-2 px-1">
        <button
          onClick={() => handleVolumeChange(localVolume === 0 ? 50 : 0)}
          className="text-gray-500 hover:text-gray-300 transition-colors shrink-0"
        >
          {localVolume === 0
            ? <VolumeX size={14} />
            : <Volume2 size={14} />
          }
        </button>
        <input
          type="range"
          min={0}
          max={100}
          value={localVolume}
          onChange={e => handleVolumeChange(Number(e.target.value))}
          className="flex-1 h-1 rounded-full cursor-pointer accent-[#1DB954]"
          style={{ background: `linear-gradient(to right, #1DB954 ${localVolume}%, #3f3f3f ${localVolume}%)` }}
        />
        <span className="text-gray-600 text-[10px] w-6 text-right tabular-nums shrink-0">
          {localVolume}
        </span>
      </div>
    </>
  )
}
