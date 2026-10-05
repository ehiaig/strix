import * as THREE from 'three'
import { makePose, clonePose, lerpPose, type Pose, type Side } from './rig'

const D = THREE.MathUtils.degToRad
const clamp = THREE.MathUtils.clamp
const smooth = (a: number, b: number, x: number) => {
  const t = clamp((x - a) / (b - a), 0, 1)
  return t * t * (3 - 2 * t)
}

/**
 * Flight demonstration parameters. All rates are REAL-TIME values for the assumed
 * 1.1 kg / 1.22 m-span airframe; `timeScale` slows playback for presentation only.
 */
export const FLIGHT = {
  timeScale: 0.65,
  cruiseSpeed: 10.5, // m/s (C_L ≈ 0.84 on total planform at 2.15 kg)
  glideSpeed: 10.7,
  launchSpeed: 2.0, // from the leg push
  turnRadius: 18, // m (≈32° bank at 10.5 m/s, load factor ≈1.18: steeper turns exceed the lift margin)
  maxBank: D(34),
  freq: { takeoff: 4.3, cruise: 3.8, landing: 4.2 }, // Hz (Pennycuick-type estimate ≈3.8 Hz for 2.15 kg, 1.44 m span, 0.37 m²)
  amp: { takeoff: D(52), cruise: D(36), landing: D(46) }, // stroke half-amplitude
  strokeBias: D(5), // mean dihedral during flapping
  downFraction: 0.56, // fraction of the cycle spent in the downstroke
  twistDown: D(8), // arm pronation at mid-downstroke
  twistUp: D(6), // arm supination at mid-upstroke
  handTwistGain: 2.0, // hand twist slaved to arm pitch by the 2:1 wrist tendon pulleys (≈16° at the hand)
  upstrokeFlex: 0.32, // passive, load-actuated elbow+wrist flex in the upstroke (reduces upstroke span)
  featherOpenUp: D(13), // primary slot opening at mid-upstroke
  sweepAmp: D(0), // sweep is held during flapping (servo used for stow/landing trim only)
  tipBend: D(7), // aeroelastic primary tip bend under downstroke load
  bobAmp: 0.008, // body heave (m) from the wingbeat
  prepTime: 1.1, // seconds (sim) of pre-launch preparation on the perch
  settleTime: 0.9, // seconds (sim) after touchdown
  aoaCruise: D(5),
}

export interface FlightFrame {
  pos: THREE.Vector3
  quat: THREE.Quaternion
  pose: Pose
  phase: 'prep' | 'launch' | 'cruise' | 'glide' | 'approach' | 'settle' | 'done'
  progress: number
  speed: number
  feetPivot: number // 0..1: how much the root is governed by the feet-on-perch constraint
  bodyPitch: number // for feet pivot (rad, Rx)
  lookWorld: THREE.Vector3
}

/** Wingbeat generator: maps phase to stroke and coupled articulation. */
export function wingbeat(u: number, A: number, opts: { bias: number; twist: number; flex: number; open: number; sweep: number; df: number }) {
  const { df } = opts
  let c: number // +1 top .. -1 bottom
  let down: number // 0..1 bump during downstroke
  let up: number // 0..1 bump during upstroke
  if (u < df) {
    const t = u / df
    c = Math.cos(Math.PI * t)
    down = Math.sin(Math.PI * t)
    up = 0
  } else {
    const t = (u - df) / (1 - df)
    c = -Math.cos(Math.PI * t)
    down = 0
    up = Math.sin(Math.PI * t)
  }
  return {
    c,
    down,
    stroke: opts.bias + A * c,
    pitch: opts.twist * (down - 0.66 * up),
    fold: opts.flex * Math.pow(up, 1.2),
    open: opts.open * up - D(12) * down, // downstroke: aeroelastic nose-down rotation of the asymmetric-vaned primaries (to ≈12° at P10)
    sweep: -opts.sweep * down + opts.sweep * 0.8 * up,
  }
}

export class FlightPlan {
  curve: THREE.CatmullRomCurve3
  length: number
  glide: [number, number]
  approachStart: number
  constructor(points: THREE.Vector3[], glideFrac: [number, number]) {
    this.curve = new THREE.CatmullRomCurve3(points, false, 'centripetal', 0.5)
    this.curve.arcLengthDivisions = 2000
    this.length = this.curve.getLength()
    this.glide = [glideFrac[0] * this.length, glideFrac[1] * this.length]
    this.approachStart = this.length - 4.5
  }
  at(s: number, out: THREE.Vector3) {
    return this.curve.getPointAt(clamp(s / this.length, 0, 1), out)
  }
  tangent(s: number, out: THREE.Vector3) {
    return this.curve.getTangentAt(clamp(s / this.length, 0, 0.9999), out)
  }
}

function findS(curve: THREE.CatmullRomCurve3, pred: (p: THREE.Vector3) => boolean, from = 0) {
  const n = 1500
  const L = curve.getLength()
  const p = new THREE.Vector3()
  for (let i = 0; i <= n; i++) {
    const u = i / n
    curve.getPointAt(u, p)
    if (u * L >= from && pred(p)) return u * L
  }
  return L
}

/**
 * Racetrack demo route (world metres), sized for a credible bank angle at cruise speed:
 * climb-out toward the viewer, a left turn of radius R, a long straight, a second turn,
 * then a glide and a flapping approach to the perch from behind.
 */
export function demoRoute(start: THREE.Vector3, end: THREE.Vector3 = start) {
  const R = FLIGHT.turnRadius, L1 = 12, L2 = 14
  const y0 = end.y
  const P = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z)
  // launch: the robot trades ~0.55 m of height for speed while the high-amplitude strokes build
  // airspeed past the stall speed, then climbs out
  const y = start.y
  const pts = [start.clone(), P(0, y - 0.10, start.z + 1.2), P(0, y - 0.38, 3.4), P(0, y - 0.55, 5.8), P(0, y - 0.32, 8.6), P(0.0, 1.85, L1)]
  for (const a of [30, 60, 90, 120, 150]) {
    const t = THREE.MathUtils.degToRad(a)
    pts.push(P(R - R * Math.cos(t), 2.25 + 0.3 * Math.sin(t / 2), L1 + R * Math.sin(t)))
  }
  pts.push(P(2 * R, 2.55, L1), P(2 * R, 2.6, -3), P(2 * R, 2.55, -L2))
  for (const a of [30, 60, 90, 120, 150]) {
    const t = THREE.MathUtils.degToRad(a)
    pts.push(P(R + R * Math.cos(t), 2.5 - 0.08 * (a / 150), -L2 - R * Math.sin(t)))
  }
  pts.push(P(0, 2.42, -L2), P(0, 2.3, -10.5), P(0, 2.0, -7.0), P(0, 1.6, -3.6), P(0, y0 + 0.26, -1.3), P(0, y0 + 0.06, end.z - 0.34), end.clone())
  const plan = new FlightPlan(pts, [0, 0])
  const g0 = findS(plan.curve, (p) => p.x < 0.5 && p.z > -L2 + 0.8 && p.z < 0, plan.length * 0.7)
  const g1 = findS(plan.curve, (p) => p.x < 0.3 && p.z > -5.6, g0)
  plan.glide = [g0, g1]
  plan.approachStart = plan.length - 5.0
  return plan
}

/** Builds a "return to perch" route from the current state (used when DETACH interrupts flight). */
export function returnRoute(pos: THREE.Vector3, dir: THREE.Vector3, start: THREE.Vector3) {
  const R = FLIGHT.turnRadius
  const y0 = start.y
  const P = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z)
  const pts = [pos.clone(), pos.clone().addScaledVector(dir, 3.0).setY(Math.max(pos.y, 2.2))]
  // swing out to a point well behind the perch, then line up on the approach
  const side = pts[1].x >= R ? 1 : 0
  if (pts[1].z > -9) pts.push(P(side ? 2 * R : 0.6 * R, 2.4, Math.min(pts[1].z - 6, -6)))
  pts.push(P(R * 0.9, 2.4, -17), P(0.2, 2.35, -12), P(0, 2.2, -9), P(0, 1.9, -6.0), P(0, 1.55, -3.4), P(0, y0 + 0.26, -1.3), P(0, y0 + 0.06, start.z - 0.34), start.clone())
  const plan = new FlightPlan(pts, [0, 0])
  plan.glide = [plan.length, plan.length]
  plan.approachStart = plan.length - 4.5
  return plan
}

const _t = new THREE.Vector3()
const _t2 = new THREE.Vector3()
const _up = new THREE.Vector3(0, 1, 0)
const _x = new THREE.Vector3()
const _y = new THREE.Vector3()
const _z = new THREE.Vector3()
const _m = new THREE.Matrix4()
const _q = new THREE.Quaternion()
const _e = new THREE.Euler()

export class FlightSim {
  plan: FlightPlan
  time = 0
  s = 0
  v = 0
  wingPhase = 0
  launched = false
  rerouted = false
  touched = false
  touchTime = 0
  roll = 0
  pitch = 0
  amp = 0
  freq = 0
  perchedPose: Pose
  startQuat: THREE.Quaternion
  startPitch: number
  frame: FlightFrame
  done = false
  private glideW = 0

  constructor(plan: FlightPlan, perchedPose: Pose, startPitch: number, startQuat: THREE.Quaternion) {
    this.plan = plan
    this.perchedPose = clonePose(perchedPose)
    this.startPitch = startPitch
    this.startQuat = startQuat.clone()
    this.frame = {
      pos: plan.at(0, new THREE.Vector3()), quat: startQuat.clone(), pose: clonePose(perchedPose), phase: 'prep',
      progress: 0, speed: 0, feetPivot: 1, bodyPitch: startPitch, lookWorld: new THREE.Vector3(),
    }
  }

  /** Switch to a new route mid-flight (keeps phase/speed continuous). */
  reroute(plan: FlightPlan) {
    this.plan = plan
    this.s = 0
  }

  private speedAt(s: number) {
    const L = this.plan.length
    const F = FLIGHT
    let v = F.cruiseSpeed
    if (!this.rerouted) v = THREE.MathUtils.lerp(F.launchSpeed + 1.4, v, smooth(0.0, 8.0, s)) // launch acceleration
    if (s >= this.plan.glide[0] && s <= this.plan.glide[1] + 0.5) v = THREE.MathUtils.lerp(v, F.glideSpeed, this.glideW)
    // approach: decelerate to ~1 m/s at the perch
    const a0 = this.plan.approachStart
    if (s > a0) v = THREE.MathUtils.lerp(v, 1.0, Math.pow(smooth(a0, L, s), 0.9))
    return v
  }

  step(dtSim: number) {
    const F = FLIGHT
    const fr = this.frame
    this.time += dtSim
    const pose = fr.pose
    const prepT = F.prepTime
    const L = this.plan.length

    // ---------------------------------------------------------------- prep on perch
    if (!this.launched) {
      const t = this.time
      const k = smooth(0, prepT, t)
      fr.phase = 'prep'
      fr.feetPivot = 1
      fr.bodyPitch = THREE.MathUtils.lerp(this.startPitch, D(-38), smooth(0.25, prepT, t))
      // wings unfold to the operational configuration, raised for the first downstroke
      const ready = makePose({
        wing: {
          L: { stroke: D(58), fold: 0, sweep: D(-4), pitch: D(-4), alula: 1, fan: D(8) },
          R: { stroke: D(58), fold: 0, sweep: D(-4), pitch: D(-4), alula: 1, fan: D(8) },
        },
        tailSpread: 1, tailPitch: D(12), iris: 0.15,
        hip: { L: -fr.bodyPitch - D(14), R: -fr.bodyPitch - D(14) },
        ankle: { L: D(18), R: D(18) },
        toe: { L: 1, R: 1 },
      })
      const wk = smooth(0.1, prepT * 0.85, t)
      lerpPose(this.perchedPose, ready, wk, pose)
      // motors "wake": a small preparatory oscillation
      const twitch = Math.sin(t * Math.PI * 2 * 2.2) * D(5) * smooth(0.45, 0.8, t) * (1 - smooth(0.85, prepT, t))
      pose.wing.L.stroke += twitch
      pose.wing.R.stroke += twitch
      pose.hip.L = pose.hip.R = -fr.bodyPitch - D(14)
      this.headStabilise(pose, fr.bodyPitch, 0)
      fr.speed = 0
      fr.progress = 0
      if (t >= prepT) {
        this.launched = true
        this.time = 0
        this.wingPhase = 0
        this.v = F.launchSpeed
        this.amp = F.amp.takeoff
        this.freq = F.freq.takeoff
        this.pitch = fr.bodyPitch
      }
      return fr
    }

    // ---------------------------------------------------------------- airborne
    if (!this.touched) {
      // glide weight
      const inGlide = this.s > this.plan.glide[0] && this.s < this.plan.glide[1]
      this.glideW = THREE.MathUtils.damp(this.glideW, inGlide ? 1 : 0, 4.5, dtSim)
      const vt = this.speedAt(this.s)
      this.v = THREE.MathUtils.damp(this.v, vt, this.time < 0.4 ? 9 : this.s > this.plan.approachStart ? 7 : 3, dtSim)
      this.s = Math.min(L, this.s + this.v * dtSim)
      const sN = this.s / L
      const approachK = smooth(this.plan.approachStart, L, this.s)
      const launchK = this.rerouted ? 0 : 1 - smooth(0.0, 5.0, this.s)

      // kinematics on the path
      this.plan.at(this.s, fr.pos)
      this.plan.tangent(this.s, _t)
      const ds = 0.25
      this.plan.tangent(Math.min(L, this.s + ds), _t2)
      const turn = _t.clone().cross(_t2).dot(_up) / ds // signed curvature (+ = left turn)
      const targetRoll = clamp(Math.atan((this.v * this.v * turn) / 9.81), -F.maxBank, F.maxBank) * (1 - approachK * 0.8)
      this.roll = THREE.MathUtils.damp(this.roll, targetRoll, 2.2, dtSim)
      const climb = Math.asin(clamp(_t.y, -1, 1))
      // nose-up pitch (Rx negative): climb + angle of attack (higher when slow, flare at the end)
      const aoa = F.aoaCruise + D(10) * launchK + D(40) * Math.pow(approachK, 1.6) + D(4) * this.glideW
      const targetPitch = -(climb + aoa)
      this.pitch = THREE.MathUtils.damp(this.pitch, targetPitch, this.time < 0.5 ? 6 : 3, dtSim)

      // wingbeat schedule
      const tf = THREE.MathUtils.lerp(F.freq.cruise, F.freq.takeoff, launchK)
      const fTarget = THREE.MathUtils.lerp(tf, F.freq.landing, approachK)
      const ta = THREE.MathUtils.lerp(F.amp.cruise, F.amp.takeoff, launchK)
      const aTarget = THREE.MathUtils.lerp(THREE.MathUtils.lerp(ta, F.amp.landing, approachK), 0, this.glideW)
      this.freq = THREE.MathUtils.damp(this.freq, fTarget, 4, dtSim)
      this.amp = THREE.MathUtils.damp(this.amp, aTarget, 5, dtSim)
      this.wingPhase = (this.wingPhase + this.freq * dtSim) % 1

      const rollCmd = clamp((targetRoll - this.roll) * 1.5 + targetRoll * 0.4, -0.5, 0.5)
      const wb = (side: Side) => {
        const asym = side === 'L' ? 1 - rollCmd * 0.35 : 1 + rollCmd * 0.35
        return wingbeat(this.wingPhase, this.amp * asym, {
          bias: F.strokeBias * (1 - this.glideW) + D(4) * this.glideW + D(6) * approachK,
          twist: F.twistDown * (1 - this.glideW),
          flex: F.upstrokeFlex * (1 - this.glideW) * (1 - 0.5 * approachK),
          open: F.featherOpenUp * (1 - this.glideW),
          sweep: F.sweepAmp * (1 - this.glideW),
          df: F.downFraction,
        })
      }
      const wl = wb('L'), wr = wb('R')
      for (const [s, w] of [['L', wl], ['R', wr]] as [Side, ReturnType<typeof wingbeat>][]) {
        const wp = pose.wing[s]
        wp.stroke = w.stroke
        wp.pitch = w.pitch + D(2) * this.glideW
        wp.handTwist = w.pitch * (F.handTwistGain - 1) + D(-3) * this.glideW
        wp.fold = w.fold
        wp.sweep = w.sweep - D(10) * approachK + D(2) * this.glideW
        wp.featherOpen = w.open
        wp.fan = D(8) * this.glideW + D(6) * approachK + D(4) * launchK
        wp.alula = Math.max(launchK * 0.8, smooth(0.35, 1, approachK))
        wp.bend = F.tipBend * w.down * (this.amp / F.amp.cruise) + D(3) * this.glideW
      }
      // body heave and small pitch response to the wing forces
      const heave = -F.bobAmp * wl.c * (this.amp / F.amp.cruise)
      fr.pos.y += heave
      const pitchOsc = D(1.2) * Math.sin(this.wingPhase * Math.PI * 2) * (this.amp / F.amp.cruise)

      // orientation: forward = path tangent, banked and pitched
      const fwd = _z.set(_t.x, 0, _t.z).normalize()
      _q.setFromUnitVectors(new THREE.Vector3(0, 0, 1), fwd)
      _e.set(this.pitch + pitchOsc, 0, 0)
      const qp = new THREE.Quaternion().setFromEuler(_e)
      const qr = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -this.roll)
      fr.quat.copy(_q).multiply(qr).multiply(qp)
      fr.bodyPitch = this.pitch

      // tail: trims with pitch, tilts for yaw in turns, spreads when slow
      pose.tailPitch = D(-4) + D(18) * approachK + D(10) * launchK - D(3) * this.glideW
      pose.tailTilt = -this.roll * 0.35
      pose.tailSpread = Math.max(launchK, approachK, 0.55 + 0.25 * this.glideW)

      // legs: push then tuck; reach forward for the landing
      const tuck = (this.rerouted ? 1 : smooth(0.2, 0.9, this.time)) * (1 - smooth(0.35, 0.85, approachK))
      const hipTuck = D(98), hipReach = D(-8)
      for (const s of ['L', 'R'] as Side[]) {
        const hipLaunch = -this.pitch - D(14)
        pose.hip[s] = THREE.MathUtils.lerp(THREE.MathUtils.lerp(hipLaunch, hipTuck, tuck), hipReach - this.pitch * 0.6, smooth(0.45, 0.95, approachK))
        // tucked: tibia and tarsus trail straight back under the tail, toes clenched
        pose.ankle[s] = THREE.MathUtils.lerp(D(18), D(-6), tuck) * (1 - smooth(0.5, 0.95, approachK)) + D(10) * smooth(0.5, 0.95, approachK)
        pose.toe[s] = THREE.MathUtils.lerp(0.25, 1.0, tuck) * (1 - smooth(0.6, 0.95, approachK))
      }
      pose.iris = 0.25 + 0.2 * approachK
      pose.blink = 0

      // head: stabilised, looks ahead into the turn, then at the perch
      const look = this.plan.at(Math.min(L, this.s + 2.5), new THREE.Vector3())
      look.addScaledVector(_t2.set(_t.z, 0, -_t.x), -this.roll * 1.2)
      const perchLook = this.plan.at(L, new THREE.Vector3()).add(new THREE.Vector3(0, -0.15, 0))
      fr.lookWorld.copy(look).lerp(perchLook, smooth(0.0, 0.6, approachK))
      fr.feetPivot = 0
      fr.speed = this.v
      fr.progress = sN
      fr.phase = approachK > 0.01 ? 'approach' : this.glideW > 0.5 ? 'glide' : this.s < 5.0 && !this.rerouted ? 'launch' : 'cruise'
      if (this.s >= L - 1e-4) {
        this.touched = true
        this.touchTime = 0
        this.settleFrom = clonePose(pose)
        this.settlePitch = this.pitch
      }
      return fr
    }

    // ---------------------------------------------------------------- touchdown & settle
    this.touchTime += dtSim
    const k = smooth(0, F.settleTime, this.touchTime)
    fr.phase = 'settle'
    fr.feetPivot = 1
    fr.bodyPitch = THREE.MathUtils.lerp(this.settlePitch, this.startPitch, smooth(0, F.settleTime * 0.9, this.touchTime))
    // a couple of decaying braking beats, then fold
    this.wingPhase = (this.wingPhase + this.freq * dtSim * (1 - k)) % 1
    lerpPose(this.settleFrom!, this.perchedPose, k, pose)
    pose.hip.L = pose.hip.R = THREE.MathUtils.lerp(this.settleFrom!.hip.L, this.perchedPose.hip.L, k)
    pose.toe.L = pose.toe.R = smooth(0.0, 0.35, this.touchTime)
    fr.progress = 1
    fr.speed = 0
    if (this.touchTime >= F.settleTime) {
      fr.phase = 'done'
      this.done = true
    }
    return fr
  }

  private settleFrom: Pose | null = null
  private settlePitch = 0

  private headStabilise(pose: Pose, bodyPitch: number, yaw: number) {
    pose.neckPitch = -bodyPitch
    pose.neckYaw = yaw
  }
}
