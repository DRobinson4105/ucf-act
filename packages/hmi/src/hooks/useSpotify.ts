import { useState, useEffect, useCallback, useRef } from 'react'

// ─── Types ────────────────────────────────────────────────────────────────────

export type RepeatState = 'off' | 'track' | 'context'

export interface SpotifyTrack {
  id: string
  name: string
  artists: string
  albumName: string
  albumArt: string | null
  durationMs: number
}

export interface SpotifyPlayerState {
  isPlaying: boolean
  track: SpotifyTrack | null
  progressMs: number
  volumePercent: number
  shuffleState: boolean
  repeatState: RepeatState
}

export interface UseSpotifyReturn {
  isAuthenticated: boolean
  isLoading: boolean
  error: string | null
  player: SpotifyPlayerState | null
  login: () => Promise<void>
  logout: () => void
  play: () => Promise<void>
  pause: () => Promise<void>
  next: () => Promise<void>
  previous: () => Promise<void>
  seek: (positionMs: number) => Promise<void>
  setVolume: (volumePercent: number) => Promise<void>
  toggleShuffle: () => Promise<void>
  cycleRepeat: () => Promise<void>
}

// ─── Constants ────────────────────────────────────────────────────────────────

const CLIENT_ID    = import.meta.env.VITE_SPOTIFY_CLIENT_ID  as string | undefined
const REDIRECT_URI = import.meta.env.VITE_SPOTIFY_REDIRECT_URI as string | undefined

const SCOPES = [
  'user-read-playback-state',
  'user-modify-playback-state',
  'user-read-currently-playing',
].join(' ')

const LS_ACCESS_TOKEN  = 'spotify_access_token'
const LS_REFRESH_TOKEN = 'spotify_refresh_token'
const LS_TOKEN_EXPIRY  = 'spotify_token_expiry'
const LS_CODE_VERIFIER = 'spotify_code_verifier'

const API_BASE      = 'https://api.spotify.com/v1'
const POLL_INTERVAL = 2_000 // ms

const REPEAT_CYCLE: RepeatState[] = ['off', 'context', 'track']

// ─── PKCE helpers ─────────────────────────────────────────────────────────────

function generateCodeVerifier(length = 128): string {
  const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-._~'
  const arr   = new Uint8Array(length)
  crypto.getRandomValues(arr)
  return Array.from(arr).map(b => chars[b % chars.length]).join('')
}

async function generateCodeChallenge(verifier: string): Promise<string> {
  const data   = new TextEncoder().encode(verifier)
  const digest = await crypto.subtle.digest('SHA-256', data)
  return btoa(String.fromCharCode(...new Uint8Array(digest)))
    .replace(/\+/g, '-').replace(/\//g, '_').replace(/=/g, '')
}

// ─── Token helpers ────────────────────────────────────────────────────────────

function getStoredTokens() {
  return {
    accessToken:  localStorage.getItem(LS_ACCESS_TOKEN),
    refreshToken: localStorage.getItem(LS_REFRESH_TOKEN),
    expiry:       Number(localStorage.getItem(LS_TOKEN_EXPIRY) ?? 0),
  }
}

function storeTokens(accessToken: string, refreshToken: string, expiresIn: number) {
  localStorage.setItem(LS_ACCESS_TOKEN,  accessToken)
  localStorage.setItem(LS_REFRESH_TOKEN, refreshToken)
  localStorage.setItem(LS_TOKEN_EXPIRY,  String(Date.now() + expiresIn * 1_000))
}

function clearTokens() {
  localStorage.removeItem(LS_ACCESS_TOKEN)
  localStorage.removeItem(LS_REFRESH_TOKEN)
  localStorage.removeItem(LS_TOKEN_EXPIRY)
  localStorage.removeItem(LS_CODE_VERIFIER)
}

// ─── Hook ─────────────────────────────────────────────────────────────────────

export function useSpotify(): UseSpotifyReturn {
  const [isAuthenticated, setIsAuthenticated] = useState(false)
  const [isLoading,       setIsLoading]       = useState(true)
  const [error,           setError]           = useState<string | null>(null)
  const [player,          setPlayer]          = useState<SpotifyPlayerState | null>(null)

  const pollTimerRef = useRef<ReturnType<typeof setInterval> | null>(null)

  // ── Token refresh ──────────────────────────────────────────────────────────

  const refreshAccessToken = useCallback(async (): Promise<string | null> => {
    const { refreshToken } = getStoredTokens()
    if (!refreshToken || !CLIENT_ID) return null

    const res = await fetch('https://accounts.spotify.com/api/token', {
      method:  'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type:    'refresh_token',
        refresh_token: refreshToken,
        client_id:     CLIENT_ID,
      }),
    })

    if (!res.ok) {
      clearTokens()
      setIsAuthenticated(false)
      return null
    }

    const data = await res.json() as {
      access_token: string
      refresh_token?: string
      expires_in: number
    }
    storeTokens(data.access_token, data.refresh_token ?? refreshToken, data.expires_in)
    return data.access_token
  }, [])

  // ── Authenticated fetch ────────────────────────────────────────────────────

  const apiFetch = useCallback(async (
    path: string,
    options: RequestInit = {},
  ): Promise<Response> => {
    const { accessToken, expiry } = getStoredTokens()

    let token = accessToken
    if (!token || Date.now() > expiry - 60_000) {
      token = await refreshAccessToken()
      if (!token) throw new Error('Not authenticated')
    }

    const doFetch = (t: string) => fetch(`${API_BASE}${path}`, {
      ...options,
      headers: {
        Authorization:  `Bearer ${t}`,
        'Content-Type': 'application/json',
        ...options.headers,
      },
    })

    const res = await doFetch(token)
    if (res.status !== 401) return res

    const newToken = await refreshAccessToken()
    if (!newToken) throw new Error('Token refresh failed')
    return doFetch(newToken)
  }, [refreshAccessToken])

  // ── Playback state polling ─────────────────────────────────────────────────

  const fetchPlaybackState = useCallback(async () => {
    try {
      const res = await apiFetch('/me/player')
      if (res.status === 204 || res.status === 404) { setPlayer(null); return }
      if (!res.ok) return

      const data = await res.json() as {
        is_playing:    boolean
        progress_ms:   number
        shuffle_state: boolean
        repeat_state:  RepeatState
        item: {
          id:          string
          name:        string
          duration_ms: number
          artists:     Array<{ name: string }>
          album: {
            name:   string
            images: Array<{ url: string }>
          }
        } | null
        device?: { volume_percent: number }
      }

      if (!data?.item) { setPlayer(null); return }

      setPlayer({
        isPlaying:     data.is_playing,
        progressMs:    data.progress_ms ?? 0,
        volumePercent: data.device?.volume_percent ?? 100,
        shuffleState:  data.shuffle_state,
        repeatState:   data.repeat_state,
        track: {
          id:         data.item.id,
          name:       data.item.name,
          artists:    data.item.artists.map(a => a.name).join(', '),
          albumName:  data.item.album.name,
          albumArt:   data.item.album.images?.[0]?.url ?? null,
          durationMs: data.item.duration_ms,
        },
      })
    } catch {
      // Silently ignore transient network errors — state refreshes on next poll
    }
  }, [apiFetch])

  const startPolling = useCallback(() => {
    if (pollTimerRef.current) clearInterval(pollTimerRef.current)
    void fetchPlaybackState()
    pollTimerRef.current = setInterval(() => void fetchPlaybackState(), POLL_INTERVAL)
  }, [fetchPlaybackState])

  const stopPolling = useCallback(() => {
    if (pollTimerRef.current) {
      clearInterval(pollTimerRef.current)
      pollTimerRef.current = null
    }
  }, [])

  // ── Initialization ─────────────────────────────────────────────────────────

  useEffect(() => {
    let cancelled = false

    const init = async () => {
      const params  = new URLSearchParams(window.location.search)
      const code    = params.get('code')
      const authErr = params.get('error')

      if (authErr) {
        window.history.replaceState({}, '', window.location.pathname)
        if (!cancelled) { setError(`Spotify auth error: ${authErr}`); setIsLoading(false) }
        return
      }

      if (code) {
        window.history.replaceState({}, '', window.location.pathname)
        const verifier = localStorage.getItem(LS_CODE_VERIFIER)

        if (!verifier) {
          if (!cancelled) { setError('Missing PKCE code verifier'); setIsLoading(false) }
          return
        }

        const res = await fetch('https://accounts.spotify.com/api/token', {
          method:  'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: new URLSearchParams({
            grant_type:    'authorization_code',
            code,
            redirect_uri:  REDIRECT_URI ?? '',
            client_id:     CLIENT_ID     ?? '',
            code_verifier: verifier,
          }),
        })

        if (cancelled) return

        if (!res.ok) {
          setError('Failed to exchange authorization code')
          setIsLoading(false)
          return
        }

        const data = await res.json() as {
          access_token:  string
          refresh_token: string
          expires_in:    number
        }
        storeTokens(data.access_token, data.refresh_token, data.expires_in)
        localStorage.removeItem(LS_CODE_VERIFIER)

        if (!cancelled) {
          setIsAuthenticated(true)
          setIsLoading(false)
          startPolling()
        }
        return
      }

      // No OAuth callback — check stored tokens
      const { accessToken, expiry } = getStoredTokens()

      if (accessToken && Date.now() < expiry) {
        if (!cancelled) { setIsAuthenticated(true); setIsLoading(false); startPolling() }
        return
      }

      if (accessToken) {
        const newToken = await refreshAccessToken()
        if (!cancelled) {
          if (newToken) { setIsAuthenticated(true); startPolling() }
          setIsLoading(false)
        }
        return
      }

      if (!cancelled) setIsLoading(false)
    }

    void init()
    return () => { cancelled = true; stopPolling() }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  // ── Auth actions ───────────────────────────────────────────────────────────

  const login = useCallback(async () => {
    if (!CLIENT_ID || !REDIRECT_URI) {
      setError('VITE_SPOTIFY_CLIENT_ID and VITE_SPOTIFY_REDIRECT_URI must be set')
      return
    }

    const verifier  = generateCodeVerifier()
    const challenge = await generateCodeChallenge(verifier)
    localStorage.setItem(LS_CODE_VERIFIER, verifier)

    const params = new URLSearchParams({
      response_type:         'code',
      client_id:             CLIENT_ID,
      scope:                 SCOPES,
      redirect_uri:          REDIRECT_URI,
      code_challenge_method: 'S256',
      code_challenge:        challenge,
    })

    window.location.href = `https://accounts.spotify.com/authorize?${params}`
  }, [])

  const logout = useCallback(() => {
    clearTokens()
    setIsAuthenticated(false)
    setPlayer(null)
    stopPolling()
  }, [stopPolling])

  // ── Playback controls ──────────────────────────────────────────────────────

  const play = useCallback(async () => {
    await apiFetch('/me/player/play',  { method: 'PUT' })
    setPlayer(p => p ? { ...p, isPlaying: true  } : p)
  }, [apiFetch])

  const pause = useCallback(async () => {
    await apiFetch('/me/player/pause', { method: 'PUT' })
    setPlayer(p => p ? { ...p, isPlaying: false } : p)
  }, [apiFetch])

  const next = useCallback(async () => {
    await apiFetch('/me/player/next',     { method: 'POST' })
    setTimeout(() => void fetchPlaybackState(), 400)
  }, [apiFetch, fetchPlaybackState])

  const previous = useCallback(async () => {
    await apiFetch('/me/player/previous', { method: 'POST' })
    setTimeout(() => void fetchPlaybackState(), 400)
  }, [apiFetch, fetchPlaybackState])

  const seek = useCallback(async (positionMs: number) => {
    await apiFetch(`/me/player/seek?position_ms=${Math.round(positionMs)}`, { method: 'PUT' })
    setPlayer(p => p ? { ...p, progressMs: positionMs } : p)
  }, [apiFetch])

  const setVolume = useCallback(async (volumePercent: number) => {
    await apiFetch(`/me/player/volume?volume_percent=${Math.round(volumePercent)}`, { method: 'PUT' })
    setPlayer(p => p ? { ...p, volumePercent } : p)
  }, [apiFetch])

  const toggleShuffle = useCallback(async () => {
    const next = !player?.shuffleState
    await apiFetch(`/me/player/shuffle?state=${next}`, { method: 'PUT' })
    setPlayer(p => p ? { ...p, shuffleState: next } : p)
  }, [apiFetch, player?.shuffleState])

  const cycleRepeat = useCallback(async () => {
    const current   = player?.repeatState ?? 'off'
    const nextState = REPEAT_CYCLE[(REPEAT_CYCLE.indexOf(current) + 1) % REPEAT_CYCLE.length]
    await apiFetch(`/me/player/repeat?state=${nextState}`, { method: 'PUT' })
    setPlayer(p => p ? { ...p, repeatState: nextState } : p)
  }, [apiFetch, player?.repeatState])

  return {
    isAuthenticated, isLoading, error, player,
    login, logout,
    play, pause, next, previous, seek, setVolume, toggleShuffle, cycleRepeat,
  }
}
