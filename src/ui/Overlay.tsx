import { useEffect } from 'react'
import { useProgress } from '@react-three/drei'
import { ui, useUI, useDirectorSnapshot } from '../store'
import { infoFor } from '../owl/parts'
import { FLIGHT } from '../owl/flight'
import { resetCamera } from '../scene/CameraRig'
import type { Command, OwlState } from '../owl/director'

const LABEL: Record<OwlState, string> = {
  ASSEMBLED: 'Assembled',
  DETACHING: 'Detaching',
  DETACHED: 'Exploded view',
  ASSEMBLING: 'Assembling',
  TAKING_OFF: 'Take-off',
  FLYING: 'Flight',
  LANDING: 'Landing',
}

function useCommand() {
  const d = useUI((s) => s.director)
  return (c: Command) => d?.command(c)
}

function Button({ cmd, label, disabled, active, queued, onClick, hint }: {
  cmd: Command; label: string; disabled: boolean; active: boolean; queued: boolean; onClick: () => void; hint: string
}) {
  return (
    <button
      className={`ctl ${active ? 'is-active' : ''} ${queued ? 'is-queued' : ''}`}
      disabled={disabled}
      onClick={onClick}
      aria-label={label}
      title={`${label} (${hint})`}
      data-cmd={cmd}
    >
      <span className="ctl-dot" />
      <span className="ctl-label">{label}</span>
    </button>
  )
}

export function Controls() {
  const snap = useDirectorSnapshot()
  const run = useCommand()
  const st = snap?.state ?? 'ASSEMBLED'
  const flying = st === 'TAKING_OFF' || st === 'FLYING' || st === 'LANDING'

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement) return
      const k = e.key.toLowerCase()
      if (k === 'd') run('DETACH')
      else if (k === 'a') run('ASSEMBLE')
      else if (k === 'f') run('FLY')
      else if (k === 'r') resetCamera()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  })

  return (
    <div className="dock">
      <div className="status mono">
        <span className={`led ${flying ? 'led-fly' : st === 'DETACHED' || st === 'DETACHING' || st === 'ASSEMBLING' ? 'led-work' : ''}`} />
        {LABEL[st]}
        {snap?.queued ? <span className="dim"> · then {snap.queued.toLowerCase()}</span> : null}
        {flying ? (
          <span className="dim">
            {' '}· {snap?.speed.toFixed(1)} m/s · playback {FLIGHT.timeScale}×
          </span>
        ) : null}
      </div>
      <div className="ctls" role="group" aria-label="Robot controls">
        <Button cmd="DETACH" label="Detach" hint="D" disabled={st === 'DETACHED' || st === 'DETACHING'}
          active={st === 'DETACHED' || st === 'DETACHING'} queued={snap?.queued === 'DETACH'} onClick={() => run('DETACH')} />
        <Button cmd="ASSEMBLE" label="Assemble" hint="A" disabled={st === 'ASSEMBLED' || st === 'ASSEMBLING' || flying}
          active={st === 'ASSEMBLING'} queued={false} onClick={() => run('ASSEMBLE')} />
        <Button cmd="FLY" label="Fly" hint="F" disabled={flying} active={flying} queued={snap?.queued === 'FLY'} onClick={() => run('FLY')} />
      </div>
    </div>
  )
}

export function InfoCard() {
  const hovered = useUI((s) => s.hovered)
  const pinned = useUI((s) => s.pinned)
  const snap = useDirectorSnapshot()
  const name = hovered ?? pinned
  const info = name ? infoFor(name) : null
  const show = !!info && (snap?.state === 'ASSEMBLED' || snap?.state === 'DETACHED')
  return (
    <div className={`info ${show ? 'is-on' : ''}`} aria-live="polite">
      {info ? (
        <>
          <div className="info-group mono">{info.group}</div>
          <div className="info-title">{info.title}</div>
          <div className="info-fn">{info.fn}</div>
          {pinned && !hovered ? (
            <button className="info-close mono" onClick={() => ui.set({ pinned: null })}>close</button>
          ) : null}
        </>
      ) : null}
    </div>
  )
}

export function Header() {
  const snap = useDirectorSnapshot()
  const hint = snap?.state === 'DETACHED' ? 'Hover or tap a component for its function' : 'Drag to orbit · scroll to zoom · right-drag to pan'
  return (
    <>
      <header className="brand">
        <div className="brand-mark">STRIX<span className="brand-sep">·</span>01</div>
        <div className="brand-sub">Biomimetic flapping-wing robot · concept visualisation</div>
      </header>
      <div className="topright">
        <button className="iconbtn" onClick={resetCamera} title="Reset view (R)" aria-label="Reset view">
          <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" strokeWidth="1.6">
            <circle cx="12" cy="12" r="7" />
            <path d="M12 2v4M12 18v4M2 12h4M18 12h4" />
          </svg>
        </button>
      </div>
      <div className="hint mono">{hint}</div>
    </>
  )
}

export function Loader() {
  const { progress, active } = useProgress()
  const ready = useUI((s) => !!s.director)
  return (
    <div className={`loader ${ready && !active ? 'is-done' : ''}`}>
      <div className="loader-inner">
        <div className="brand-mark">STRIX<span className="brand-sep">·</span>01</div>
        <div className="loader-bar"><div style={{ width: `${Math.round(progress)}%` }} /></div>
        <div className="mono dim">loading airframe · {Math.round(progress)}%</div>
      </div>
    </div>
  )
}
