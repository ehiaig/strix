import * as THREE from 'three'
import gsap from 'gsap'
import { OwlRig, makePose, clonePose, lerpPose, type Pose, type Side } from './rig'
import { AssemblyController } from './assembly'
import { FlightSim, FLIGHT, demoRoute, returnRoute } from './flight'
import report from '../../blender/build_report.json'

const D = THREE.MathUtils.degToRad

// GSAP is ticked manually from the render loop so every animation shares one clock
// (deterministic, pausable, and steppable for automated screenshots/tests).
gsap.ticker.remove(gsap.updateRoot)

export type OwlState = 'ASSEMBLED' | 'DETACHING' | 'DETACHED' | 'ASSEMBLING' | 'TAKING_OFF' | 'FLYING' | 'LANDING'
export type Command = 'DETACH' | 'ASSEMBLE' | 'FLY'

export interface CameraRequest {
  mode: 'orbit' | 'frame' | 'track'
  target: THREE.Vector3
  distance: number
  seq: number
  duration: number
}

export const PERCHED_PITCH = D(-62)

export function perchedPose(): Pose {
  const w = { stroke: D(-90), sweep: D(96), pitch: D(0), handTwist: D(0), fold: 1.0, alula: 0, featherOpen: 0, fan: 0, bend: 0 }
  return makePose({
    wing: { L: { ...w }, R: { ...w } },
    neckPitch: -PERCHED_PITCH,
    tailPitch: D(14),
    tailSpread: -0.6, // closed fan while perched
    hip: { L: -PERCHED_PITCH - D(10), R: -PERCHED_PITCH - D(10) },
    ankle: { L: D(10), R: D(10) },
    toe: { L: 1, R: 1 },
    iris: 0.35,
  })
}

export const NEUTRAL = makePose({ iris: 0.3 })

/** Flight centre of mass in owl space, from the Blender build's mass budget (Blender -> glTF axes). */
const _cgb = (report as any).cg_blender as number[]
export const CG_OWL = new THREE.Vector3(_cgb[0], _cgb[2], -_cgb[1])

export interface Snapshot {
  state: OwlState
  queued: Command | null
  flightPhase: string
  speed: number
  progress: number
  explode: number
}

const _v = new THREE.Vector3()
const _v2 = new THREE.Vector3()
const _g = new THREE.Vector3()
const _q = new THREE.Quaternion()
const _m = new THREE.Matrix4()
const X = new THREE.Vector3(1, 0, 0)

export class Director {
  state: OwlState = 'ASSEMBLED'
  queued: Command | null = null
  clock = 0
  paused = false
  timeScale = 1

  rig: OwlRig
  assembly: AssemblyController
  owlRoot: THREE.Object3D
  owl: THREE.Object3D
  grips: THREE.Object3D[]
  perchBar: THREE.Vector3
  displayPos = new THREE.Vector3(0, 1.8, 0.12)

  blend = { w: 0 } // 0 = perched showcase, 1 = neutral assembly pose on the fixture
  private tl: gsap.core.Timeline | null = null
  pose: Pose
  perched: Pose
  perchRootPos = new THREE.Vector3()
  perchRootQuat = new THREE.Quaternion()
  sim: FlightSim | null = null
  private rerouted = false

  // idle behaviour
  look = new THREE.Vector3(0, 1.1, 2)
  pointerLook = new THREE.Vector3(0, 1.1, 2)
  private neck = { p: 0, y: 0 }
  private eye = { y: 0, p: 0, ty: 0, tp: 0, next: 1 }
  private blinkT = { next: 2.5, t: -1 }
  camera: CameraRequest
  private listeners = new Set<() => void>()
  private snap: Snapshot
  headLocal = new THREE.Vector3(0, 0.05, 0.15)

  constructor(owlRoot: THREE.Object3D, owl: THREE.Object3D, rig: OwlRig, assembly: AssemblyController, perchBar: THREE.Vector3) {
    this.owlRoot = owlRoot
    this.owl = owl
    this.rig = rig
    this.assembly = assembly
    this.perchBar = perchBar
    this.grips = ['G_grip_L', 'G_grip_R'].map((n) => owl.getObjectByName(n)!).filter(Boolean)
    this.perched = perchedPose()
    this.balancePerch()
    this.pose = clonePose(this.perched)
    this.rig.apply(this.pose)
    this.pivotRoot(PERCHED_PITCH, this.perchRootPos, this.perchRootQuat)
    owlRoot.position.copy(this.perchRootPos)
    owlRoot.quaternion.copy(this.perchRootQuat)
    this.camera = { mode: 'frame', target: this.defaultTarget(), distance: 1.3, seq: 0, duration: 0 }
    this.snap = this.makeSnap()
  }

  // ------------------------------------------------------------------ store
  subscribe = (fn: () => void) => {
    this.listeners.add(fn)
    return () => this.listeners.delete(fn)
  }
  getSnapshot = () => this.snap
  private makeSnap(): Snapshot {
    return {
      state: this.state, queued: this.queued, flightPhase: this.sim?.frame.phase ?? '',
      speed: this.sim?.frame.speed ?? 0, progress: this.sim?.frame.progress ?? 0, explode: this.assembly.progress,
    }
  }
  private emit() {
    this.snap = this.makeSnap()
    for (const l of this.listeners) l()
  }
  private setState(s: OwlState) {
    if (this.state !== s) {
      this.state = s
      this.emit()
    }
  }

  defaultTarget() {
    return this.perchRootPos.clone().add(new THREE.Vector3(0, -0.035, 0.0))
  }

  private requestCamera(mode: CameraRequest['mode'], target: THREE.Vector3, distance: number, duration: number) {
    this.camera = { mode, target: target.clone(), distance, seq: this.camera.seq + 1, duration }
  }

  // ------------------------------------------------------------------ kinematic helpers
  /** Grip point (mean of both feet) in owl-root local coordinates for the current rig pose. */
  gripLocal(out: THREE.Vector3) {
    out.set(0, 0, 0)
    for (const g of this.grips) {
      _m.identity()
      for (let n: THREE.Object3D | null = g; n && n !== this.owlRoot; n = n.parent) {
        n.updateMatrix()
        _m.premultiply(n.matrix)
      }
      out.add(_v2.setFromMatrixPosition(_m))
    }
    return out.multiplyScalar(1 / Math.max(1, this.grips.length))
  }

  /** Root transform that keeps the feet locked on the perch bar for a body pitch. */
  pivotRoot(pitch: number, outPos: THREE.Vector3, outQuat: THREE.Quaternion) {
    outQuat.setFromAxisAngle(X, pitch)
    const g = this.gripLocal(_g).applyQuaternion(outQuat)
    outPos.copy(this.perchBar).sub(g)
  }

  /**
   * Choose the hip angle so the (flight) centre of mass sits directly above the perch bar:
   * the perched robot balances on its feet instead of being held up by the stand.
   */
  private balancePerch() {
    const cg = CG_OWL
    const pos = new THREE.Vector3(), quat = new THREE.Quaternion()
    const f = (h: number) => {
      this.perched.hip.L = this.perched.hip.R = -PERCHED_PITCH + h
      this.rig.apply(this.perched)
      this.pivotRoot(PERCHED_PITCH, pos, quat)
      return cg.clone().applyQuaternion(quat).add(pos).z - this.perchBar.z
    }
    let lo = D(-45), hi = D(35)
    let flo = f(lo), fhi = f(hi)
    if (flo * fhi > 0) {
      // fall back to the best sampled value
      let best = 0, bv = Infinity
      for (let h = lo; h <= hi; h += D(1)) { const v = Math.abs(f(h)); if (v < bv) { bv = v; best = h } }
      f(best)
      this.hipTrim = best
      return
    }
    for (let i = 0; i < 40; i++) {
      const m = 0.5 * (lo + hi), fm = f(m)
      if (fm * flo <= 0) { hi = m; fhi = fm } else { lo = m; flo = fm }
    }
    this.hipTrim = 0.5 * (lo + hi)
    f(this.hipTrim)
  }
  hipTrim = 0

  /** Neck angles that point the head at a world target (pitch then yaw). */
  private aimHead(target: THREE.Vector3, rootQuat: THREE.Quaternion, rootPos: THREE.Vector3, limitYaw = 1.7) {
    const head = _v.copy(this.headLocal).applyQuaternion(rootQuat).add(rootPos)
    const d = _v2.copy(target).sub(head).normalize().applyQuaternion(_q.copy(rootQuat).invert())
    // pitch puts the target in the head's yaw plane (|pitch| < 90°, so a target behind the robot
    // turns the head sideways at a natural pitch instead of craning it over backwards)
    const pT = Math.atan(-d.y / (Math.abs(d.z) < 1e-6 ? (d.z < 0 ? -1e-6 : 1e-6) : d.z))
    // for targets out to the side the pitch is ill-conditioned: hold the head level instead
    const up = _v.set(0, 1, 0).applyQuaternion(_q.copy(rootQuat).invert())
    const pLevel = Math.atan2(up.z, up.y)
    const w = THREE.MathUtils.smoothstep(Math.abs(d.x), 0.45, 0.95)
    const np = THREE.MathUtils.clamp(THREE.MathUtils.lerp(pT, pLevel, w), -D(40), D(100))
    const fz = -d.y * Math.sin(np) + d.z * Math.cos(np)
    const ny = THREE.MathUtils.clamp(Math.atan2(d.x, fz), -limitYaw, limitYaw)
    return { p: np, y: ny }
  }

  // ------------------------------------------------------------------ commands
  command(cmd: Command) {
    const s = this.state
    if (cmd === 'DETACH') {
      if (s === 'ASSEMBLED' || s === 'ASSEMBLING') this.startDetach()
      else if (s === 'TAKING_OFF' || s === 'FLYING' || s === 'LANDING') {
        this.queued = 'DETACH'
        this.abortFlight()
      }
    } else if (cmd === 'ASSEMBLE') {
      if (s === 'DETACHED' || s === 'DETACHING') {
        if (this.queued === 'DETACH') this.queued = null
        this.startAssemble()
      } else if (this.queued === 'DETACH') this.queued = null
    } else if (cmd === 'FLY') {
      if (s === 'ASSEMBLED') this.startFlight()
      else if (s === 'DETACHED' || s === 'DETACHING') {
        this.queued = 'FLY'
        this.startAssemble()
      } else if (s === 'ASSEMBLING') this.queued = 'FLY'
    }
    this.emit()
  }

  private killTl() {
    if (this.tl) {
      this.tl.kill()
      this.tl = null
    }
    this.assembly.kill()
  }

  private explodedFrame() {
    const box = this.assembly.explodedBox
    const c = box.getCenter(new THREE.Vector3()).add(this.displayPos)
    const size = box.getSize(new THREE.Vector3())
    const r = 0.5 * Math.hypot(size.x, size.y, size.z)
    return { c, dist: Math.max(1.6, r * 2.05) }
  }

  private startDetach() {
    this.killTl()
    this.queued = this.queued === 'DETACH' ? null : this.queued
    this.setState('DETACHING')
    const f = this.explodedFrame()
    this.requestCamera('frame', f.c, f.dist, 2.4)
    const tl = gsap.timeline()
    const w = this.blend.w
    if (w < 1) tl.to(this.blend, { w: 1, duration: 1.1 * (1 - w) + 0.05, ease: 'power2.inOut' })
    tl.call(() => {
      this.assembly.run(1, () => {
        this.setState('DETACHED')
      })
    })
    this.tl = tl
  }

  private startAssemble() {
    this.killTl()
    this.setState('ASSEMBLING')
    this.assembly.run(0, () => {
      const tl = gsap.timeline()
      tl.to(this.blend, { w: 0, duration: 1.15 * this.blend.w + 0.05, ease: 'power2.inOut' })
      tl.call(() => {
        this.tl = null
        this.setState('ASSEMBLED')
        this.requestCamera('frame', this.defaultTarget(), 1.3, 1.6)
        if (this.queued === 'FLY') {
          this.queued = null
          this.startFlight()
        } else this.emit()
      })
      this.tl = tl
      // pull the camera back toward the showcase framing while the owl settles
      this.requestCamera('frame', this.defaultTarget(), 1.3, 2.0)
    })
  }

  private startFlight() {
    this.killTl()
    this.blend.w = 0
    this.rerouted = false
    const sim = new FlightSim(demoRoute(this.perchRootPos), this.perched, PERCHED_PITCH, this.perchRootQuat)
    // copy current idle head/eye state so the prep starts continuously
    sim.frame.pose = clonePose(this.pose)
    this.sim = sim
    this.setState('TAKING_OFF')
    this.requestCamera('track', this.perchRootPos, 3.8, 2.5)
  }

  private abortFlight() {
    const sim = this.sim
    if (!sim) return
    if (!sim.launched) {
      ;(sim as any).launched = true
      ;(sim as any).touched = true
      ;(sim as any).touchTime = 0
      ;(sim as any).settleFrom = clonePose(sim.frame.pose)
      ;(sim as any).settlePitch = sim.frame.bodyPitch
      return
    }
    if (sim.touched || this.rerouted) return
    const pos = this.owlRoot.position.clone()
    const dir = new THREE.Vector3(0, 0, 1).applyQuaternion(this.owlRoot.quaternion)
    dir.y = 0
    dir.normalize()
    sim.reroute(returnRoute(pos, dir, this.perchRootPos))
    this.rerouted = true
    sim.rerouted = true
  }

  // ------------------------------------------------------------------ frame update
  update(dtReal: number, pointerLook?: THREE.Vector3) {
    if (this.paused) dtReal = 0
    const dt = Math.min(dtReal, 1 / 20) * this.timeScale
    this.clock += dt
    gsap.updateRoot(this.clock)
    if (pointerLook) this.pointerLook.copy(pointerLook)

    const st = this.state
    if (st === 'TAKING_OFF' || st === 'FLYING' || st === 'LANDING') {
      this.updateFlight(dt)
    } else {
      this.updateShowcase(dt)
    }
    this.assembly.update()
    if (this.snap.explode !== this.assembly.progress && (st === 'DETACHING' || st === 'ASSEMBLING')) this.emit()
  }

  private updateShowcase(dt: number) {
    const w = this.blend.w
    const e = w // eased by GSAP already
    // pose: perched (+idle) -> neutral
    const idle = clonePose(this.perched)
    this.idleLife(idle, dt, this.perchRootQuat, this.perchRootPos, 1 - e)
    lerpPose(idle, NEUTRAL, e, this.pose)
    // keep eyes alive on the fixture too
    this.pose.eyeYaw = idle.eyeYaw
    this.pose.eyePitch = idle.eyePitch
    this.pose.blink = idle.blink
    this.pose.iris = idle.iris
    this.rig.apply(this.pose)
    // root: feet-locked on perch -> display fixture
    if (e <= 0) {
      this.pivotRoot(PERCHED_PITCH, this.owlRoot.position, this.owlRoot.quaternion)
    } else {
      const lift = new THREE.Vector3().copy(this.perchRootPos)
      this.owlRoot.position.copy(lift).lerp(this.displayPos, e)
      this.owlRoot.quaternion.copy(this.perchRootQuat).slerp(new THREE.Quaternion(), e)
    }
  }

  /** Ambient behaviour on the perch: head tracking, saccades, blinks, iris, breathing. */
  private idleLife(p: Pose, dt: number, rootQuat: THREE.Quaternion, rootPos: THREE.Vector3, weight: number) {
    const t = this.clock
    // head tracks the viewer / pointer with owl-like holds
    this.look.lerp(this.pointerLook, 1 - Math.exp(-dt * 2.2))
    const aim = this.aimHead(this.look, rootQuat, rootPos)
    this.neck.p = THREE.MathUtils.damp(this.neck.p, aim.p, 4.0, dt)
    this.neck.y = THREE.MathUtils.damp(this.neck.y, aim.y, 3.2, dt)
    p.neckPitch = THREE.MathUtils.lerp(p.neckPitch, this.neck.p, weight)
    p.neckYaw = THREE.MathUtils.lerp(p.neckYaw, this.neck.y, weight)
    // saccades
    if (t > this.eye.next) {
      this.eye.ty = D((Math.random() - 0.5) * 7)
      this.eye.tp = D((Math.random() - 0.5) * 5)
      this.eye.next = t + 0.9 + Math.random() * 2.4
    }
    this.eye.y = THREE.MathUtils.damp(this.eye.y, this.eye.ty, 22, dt)
    this.eye.p = THREE.MathUtils.damp(this.eye.p, this.eye.tp, 22, dt)
    p.eyeYaw = this.eye.y
    p.eyePitch = this.eye.p
    // blinks
    if (this.blinkT.t < 0 && t > this.blinkT.next) this.blinkT.t = 0
    if (this.blinkT.t >= 0) {
      this.blinkT.t += dt
      const b = this.blinkT.t / 0.22
      p.blink = b < 1 ? Math.sin(Math.PI * b) : 0
      if (b >= 1) {
        this.blinkT.t = -1
        this.blinkT.next = t + 3.5 + Math.random() * 4
      }
    }
    // iris breathing
    p.iris = 0.35 + 0.08 * Math.sin(t * 0.7) + 0.04 * Math.sin(t * 1.9)
    // subtle tail and wing micro-trim
    p.tailTilt += D(1.2) * Math.sin(t * 0.6) * weight
    for (const s of ['L', 'R'] as Side[]) {
      p.wing[s].stroke += D(0.6) * Math.sin(t * 1.3 + (s === 'L' ? 0 : 0.4)) * weight
    }
  }

  private updateFlight(dt: number) {
    const sim = this.sim!
    const dts = dt * FLIGHT.timeScale
    const wasLaunched = sim.launched
    const fr = sim.step(dts)
    if (!wasLaunched && sim.launched && !sim.touched) {
      // build the route from the actual launch position so the transition is seamless
      this.pivotRoot(sim.frame.bodyPitch, _v, _q)
      sim.plan = demoRoute(_v.clone(), this.perchRootPos)
    }
    this.pose = fr.pose
    // head control
    if (fr.feetPivot >= 1) {
      // on the perch: stabilise and look forward/down the launch line
      const q = _q.setFromAxisAngle(X, fr.bodyPitch)
      this.rig.apply(this.pose)
      this.pivotRoot(fr.bodyPitch, this.owlRoot.position, this.owlRoot.quaternion)
      const lookAt = fr.phase === 'settle' ? this.pointerLook : new THREE.Vector3(0, this.perchRootPos.y + 0.1, 4)
      const aim = this.aimHead(lookAt, q, this.owlRoot.position)
      this.neck.p = THREE.MathUtils.damp(this.neck.p, aim.p, 6, dt)
      this.neck.y = THREE.MathUtils.damp(this.neck.y, aim.y, 5, dt)
      this.pose.neckPitch = this.neck.p
      this.pose.neckYaw = this.neck.y
      this.rig.apply(this.pose)
    } else {
      const aim = this.aimHead(fr.lookWorld, fr.quat, fr.pos, 1.3)
      this.neck.p = THREE.MathUtils.damp(this.neck.p, aim.p, 8, dt)
      this.neck.y = THREE.MathUtils.damp(this.neck.y, aim.y, 6, dt)
      this.pose.neckPitch = this.neck.p
      this.pose.neckYaw = this.neck.y
      this.rig.apply(this.pose)
      // blend into the feet-locked transform during the last half metre
      const L = sim.plan.length
      const k = THREE.MathUtils.smoothstep(sim.s, L - 0.55, L)
      this.owlRoot.quaternion.copy(fr.quat)
      if (k > 0) {
        this.pivotRoot(fr.bodyPitch, _v, _q)
        this.owlRoot.position.copy(fr.pos).lerp(_v, k)
        this.owlRoot.quaternion.slerp(_q, k)
      } else {
        this.owlRoot.position.copy(fr.pos)
      }
    }
    // state mapping
    const ph = fr.phase
    const next: OwlState = ph === 'prep' || ph === 'launch' ? 'TAKING_OFF' : ph === 'cruise' || ph === 'glide' ? 'FLYING' : ph === 'done' ? 'ASSEMBLED' : 'LANDING'
    if (ph === 'done') {
      this.sim = null
      this.pose = clonePose(this.perched)
      this.pose.neckPitch = this.neck.p
      this.pose.neckYaw = this.neck.y
      this.setState('ASSEMBLED')
      this.requestCamera('frame', this.defaultTarget(), 1.3, 2.0)
      if (this.queued === 'DETACH') {
        this.queued = null
        this.startDetach()
      }
      this.emit()
      return
    }
    if (next !== this.state) this.setState(next)
    else if (Math.random() < 0.1) this.emit()
  }
}
