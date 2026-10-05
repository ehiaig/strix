import * as THREE from 'three'
import { useEffect, useRef } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import { ui, useUI } from '../store'

const D = THREE.MathUtils.degToRad
const easeInOut = (t: number) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2)

/**
 * Orbit camera with three behaviours driven by the Director:
 *  - frame : animated re-target + dolly (keeps the user's viewing direction)
 *  - track : follows the flying robot; user orbit offsets are preserved
 *  - orbit : free
 */
export function CameraRig() {
  const controls = useRef<OrbitControlsImpl>(null!)
  const { camera, size } = useThree()
  const director = useUI((s) => s.director)
  const resetView = useUI((s) => s.resetView)
  const anim = useRef<{ seq: number; t: number; dur: number; fromT: THREE.Vector3; toT: THREE.Vector3; fromD: number; toD: number; dir: THREE.Vector3; fromDir?: THREE.Vector3 } | null>(null)
  const seen = useRef(-1)
  const trackTarget = useRef(new THREE.Vector3())
  const trackDist = useRef(2.2)

  // initial pose: front three-quarter, slightly above the eye line
  useEffect(() => {
    if (!director || !controls.current) return
    const t = director.defaultTarget()
    const dir = new THREE.Vector3(Math.sin(D(28)) * Math.cos(D(8)), Math.sin(D(8)), Math.cos(D(28)) * Math.cos(D(8)))
    const dist = fitDistance(1.3, size)
    camera.position.copy(t).addScaledVector(dir, dist)
    controls.current.target.copy(t)
    controls.current.update()
    seen.current = director.camera.seq
  }, [director])

  useEffect(() => {
    if (!director || !resetView) return
    const t = director.state === 'DETACHED' ? null : director.defaultTarget()
    const dir = new THREE.Vector3(Math.sin(D(28)) * Math.cos(D(8)), Math.sin(D(8)), Math.cos(D(28)) * Math.cos(D(8)))
    const req = director.camera
    const target = t ?? req.target
    const dist = t ? fitDistance(1.3, size) : fitDistance(req.distance, size)
    anim.current = { seq: -2, t: 0, dur: 1.2, fromT: controls.current.target.clone(), toT: target.clone(), fromD: camera.position.distanceTo(controls.current.target), toD: dist, dir }
  }, [resetView])

  useFrame((_, dtRaw) => {
    if (!director || !controls.current) return
    const dt = director.paused ? 0 : Math.min(dtRaw, 1 / 20) * director.timeScale
    const c = controls.current
    const req = director.camera

    if (req.seq !== seen.current) {
      seen.current = req.seq
      if (req.mode === 'track') {
        trackTarget.current.copy(c.target)
        trackDist.current = camera.position.distanceTo(c.target)
        anim.current = null
      } else {
        const off = camera.position.clone().sub(c.target)
        const dir = off.normalize()
        // keep the viewer's azimuth but bring elevation into a flattering band
        const el = Math.asin(THREE.MathUtils.clamp(dir.y, -1, 1))
        const elC = THREE.MathUtils.clamp(el, D(-20), D(38))
        const az = Math.atan2(dir.x, dir.z)
        dir.set(Math.sin(az) * Math.cos(elC), Math.sin(elC), Math.cos(az) * Math.cos(elC))
        anim.current = {
          seq: req.seq, t: 0, dur: Math.max(0.01, req.duration), fromT: c.target.clone(), toT: req.target.clone(),
          fromD: camera.position.distanceTo(c.target), toD: fitDistance(req.distance, size), dir,
        }
      }
    }

    if (dt === 0) {
      // paused (automation / capture): never fight an externally placed camera
      trackTarget.current.copy(c.target)
      c.update()
      return
    }
    if (req.mode === 'track' && !anim.current) {
      const owl = director.owlRoot.position
      const prev = c.target.clone()
      // follow the bird tightly, so a fast cruise or a sharp turn can't lag it to the frame edge
      trackTarget.current.lerp(owl, 1 - Math.exp(-dt * 9))
      c.target.copy(trackTarget.current)
      camera.position.add(_v.copy(c.target).sub(prev))
      // ease the distance toward the chase distance (sized to hold the full wingspan)
      const off = _off.copy(camera.position).sub(c.target)
      const want = fitDistance(req.distance, size)
      const nd = THREE.MathUtils.damp(off.length(), want, 2.5, dt)
      off.normalize()
      camera.position.copy(c.target).addScaledVector(off, nd)
      // hard guarantee the bird stays on screen: if it still projects past a safe margin
      // (it can on the first frames of a turn, before the follow catches up), recentre on it now
      _p.copy(owl).project(camera)
      if (Math.max(Math.abs(_p.x), Math.abs(_p.y)) > 0.82) {
        const keep = camera.position.distanceTo(owl)
        c.target.copy(owl)
        trackTarget.current.copy(owl)
        camera.position.copy(owl).addScaledVector(off, keep)
      }
    } else if (anim.current) {
      const a = anim.current
      // direction is interpolated from where the animation started, so the end view does not depend on frame rate
      if (!a.fromDir) a.fromDir = camera.position.clone().sub(c.target).normalize()
      a.t += dt
      const k = easeInOut(Math.min(1, a.t / a.dur))
      const tgt = a.fromT.clone().lerp(a.toT, k)
      const dir = slerpDir(a.fromDir, a.dir, k)
      const dist = THREE.MathUtils.lerp(a.fromD, a.toD, k)
      c.target.copy(tgt)
      camera.position.copy(tgt).addScaledVector(dir, dist)
      if (a.t >= a.dur) anim.current = null
    }
    // keep the camera above the floor
    if (camera.position.y < 0.12) camera.position.y = 0.12
    c.update()
  })

  return (
    <OrbitControls
      ref={controls as any}
      makeDefault
      enableDamping
      dampingFactor={0.08}
      rotateSpeed={0.7}
      zoomSpeed={0.8}
      panSpeed={0.6}
      minDistance={0.42}
      maxDistance={9}
      minPolarAngle={0.08}
      maxPolarAngle={Math.PI * 0.86}
      enablePan
      screenSpacePanning
      onStart={() => {
        anim.current = null
      }}
    />
  )
}

const _v = new THREE.Vector3()
const _off = new THREE.Vector3()
const _p = new THREE.Vector3()
const _qd = new THREE.Quaternion()
const _qi = new THREE.Quaternion()
function slerpDir(from: THREE.Vector3, to: THREE.Vector3, k: number) {
  _qd.setFromUnitVectors(from, to)
  _qi.identity().slerp(_qd, k)
  return from.clone().applyQuaternion(_qi).normalize()
}

/** Adapt framing distance to narrow (portrait) viewports. */
export function fitDistance(d: number, size: { width: number; height: number }) {
  const aspect = size.width / Math.max(1, size.height)
  return aspect < 1 ? d * (1 + (1 - aspect) * 1.1) : d
}

export function resetCamera() {
  ui.set({ resetView: ui.get().resetView + 1 })
}
