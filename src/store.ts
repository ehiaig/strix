import { useSyncExternalStore } from 'react'
import type { Director } from './owl/director'

type Listener = () => void

export function createStore<T>(initial: T) {
  let state = initial
  const listeners = new Set<Listener>()
  return {
    get: () => state,
    set: (patch: Partial<T>) => {
      state = { ...state, ...patch }
      listeners.forEach((l) => l())
    },
    subscribe: (l: Listener) => {
      listeners.add(l)
      return () => listeners.delete(l)
    },
  }
}

export interface UIState {
  director: Director | null
  hovered: string | null
  pinned: string | null
  loaded: number
  resetView: number
}

export const ui = createStore<UIState>({ director: null, hovered: null, pinned: null, loaded: 0, resetView: 0 })

export function useUI<T>(sel: (s: UIState) => T): T {
  return useSyncExternalStore(ui.subscribe, () => sel(ui.get()))
}

const emptySub = () => () => {}
const nullSnap = () => null

export function useDirectorSnapshot() {
  const d = useUI((s) => s.director)
  return useSyncExternalStore(d ? d.subscribe : emptySub, d ? d.getSnapshot : nullSnap)
}
