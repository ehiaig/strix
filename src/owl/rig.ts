import * as THREE from 'three'

/**
 * Kinematic rig. Joint nodes (J_*) carry a canonical transform from the GLB; the rig
 * only ever writes   joint.quaternion = canonical * jointRotation   (and translation for
 * the sliding fold rod), so the assembled geometry is recovered exactly at a zero pose.
 *
 * Feather nodes (P_pri / P_sec / P_rect) are exploded by the AssemblyController, so
 * their articulation goes through an inserted child group "R_*" that is identity at rest.
 *
 * Axis conventions (glTF local frames):
 *   shoulder stroke : local Z (body forward)   +  = left wing up
 *   shoulder sweep  : local Y (wing normal)    +  = sweep back
 *   arm pitch/twist : local X (spar axis)      +  = pronate (leading edge down)
 *   wrist fold      : local Y                   +  = hand folds back
 *   feather twist   : local X (rachis)         +  = trailing vane lifts (slot opens)
 *   feather fan     : local Y                   +  = rotates toward trailing side
 * Right-side joints are built as mirror images (M R M), so a left-side rotation maps to
 * the right side by negating its Y and Z axis components.
 */

export type Side = 'L' | 'R'

export interface WingPose {
  stroke: number
  sweep: number
  pitch: number
  handTwist: number
  fold: number // 0..1 (1 = 110°)
  alula: number // 0..1
  featherOpen: number // rad, primaries (secondaries get a fraction)
  fan: number // extra primary fan spread (rad, + = spread wider)
  bend: number // aeroelastic primary tip bend (rad, + = tips up)
}

export interface Pose {
  wing: Record<Side, WingPose>
  neckPitch: number
  neckYaw: number
  eyeYaw: number
  eyePitch: number
  blink: number // 0 open .. 1 closed
  iris: number // 0 wide .. 1 stopped down
  tailPitch: number
  tailTilt: number
  tailSpread: number // 0 = modelled rest fan, 1 = fully spread, negative = folded narrower (perched)
  hip: Record<Side, number>
  ankle: Record<Side, number>
  toe: Record<Side, number> // curl 0..1
}

export const WRIST_MAX = THREE.MathUtils.degToRad(150)
export const ELBOW_MAX = THREE.MathUtils.degToRad(145)
export const SCAP_RETRACT = THREE.MathUtils.degToRad(72)
/** secondaries' angle to the forearm spar when fully folded (deg); ~180 = parallel, trailing back */
export let SEC_FOLD_TARGET = 160
export function setSecFoldTarget(v: number) { SEC_FOLD_TARGET = v }

export function wingPose(p: Partial<WingPose> = {}): WingPose {
  return { stroke: 0, sweep: 0, pitch: 0, handTwist: 0, fold: 0, alula: 0, featherOpen: 0, fan: 0, bend: 0, ...p }
}

export function makePose(p: Partial<Omit<Pose, 'wing'>> & { wing?: Partial<Record<Side, Partial<WingPose>>> } = {}): Pose {
  const { wing, ...rest } = p
  return {
    wing: { L: wingPose(wing?.L), R: wingPose(wing?.R) },
    neckPitch: 0, neckYaw: 0, eyeYaw: 0, eyePitch: 0, blink: 0, iris: 0.35,
    tailPitch: 0, tailTilt: 0, tailSpread: 0,
    hip: { L: 0, R: 0 }, ankle: { L: 0, R: 0 }, toe: { L: 0, R: 0 },
    ...(rest as any),
  }
}

export function clonePose(p: Pose): Pose {
  return {
    ...p,
    wing: { L: { ...p.wing.L }, R: { ...p.wing.R } },
    hip: { ...p.hip }, ankle: { ...p.ankle }, toe: { ...p.toe },
  }
}

const lerp = THREE.MathUtils.lerp

export function lerpPose(a: Pose, b: Pose, t: number, out?: Pose): Pose {
  const o = out ?? clonePose(a)
  for (const s of ['L', 'R'] as Side[]) {
    const wa = a.wing[s], wb = b.wing[s], wo = o.wing[s]
    for (const k of Object.keys(wa) as (keyof WingPose)[]) wo[k] = lerp(wa[k], wb[k], t)
    o.hip[s] = lerp(a.hip[s], b.hip[s], t)
    o.ankle[s] = lerp(a.ankle[s], b.ankle[s], t)
    o.toe[s] = lerp(a.toe[s], b.toe[s], t)
  }
  for (const k of ['neckPitch', 'neckYaw', 'eyeYaw', 'eyePitch', 'blink', 'iris', 'tailPitch', 'tailTilt', 'tailSpread'] as const) {
    o[k] = lerp(a[k], b[k], t)
  }
  return o
}

// --------------------------------------------------------------------------------------
const _q = new THREE.Quaternion()
const _q2 = new THREE.Quaternion()
const _v = new THREE.Vector3()
const X = new THREE.Vector3(1, 0, 0)
const Y = new THREE.Vector3(0, 1, 0)
const Z = new THREE.Vector3(0, 0, 1)

function axisFor(side: Side, axis: THREE.Vector3) {
  return side === 'L' ? axis : _v.set(axis.x, -axis.y, -axis.z)
}

interface Joint {
  node: THREE.Object3D
  q0: THREE.Quaternion
  p0: THREE.Vector3
}

interface FeatherRig {
  wrap: THREE.Object3D
  kind: 'pri' | 'sec' | 'ter' | 'rect'
  side: Side | 'C'
  index: number
  angle0: number // fan angle of the rachis in its parent assembly frame (rad)
}

// primary / secondary base angles (deg) from the Blender build, used for fold compression
const PRI_A = [86, 79, 71, 62, 52, 41, 31, 22, 14, 8]

export class OwlRig {
  joints = new Map<string, Joint>()
  feathers: FeatherRig[] = []
  iris: { node: THREE.Object3D; q0: THREE.Quaternion; side: Side }[] = []
  membranes: { wrap: THREE.Object3D; side: Side }[] = []
  root: THREE.Object3D

  constructor(root: THREE.Object3D) {
    this.root = root
    root.traverse((o) => {
      if (o.name.startsWith('J_')) {
        this.joints.set(o.name, { node: o, q0: o.quaternion.clone(), p0: o.position.clone() })
      }
    })
    // wrap feathers in an articulation group
    const featherNodes: THREE.Object3D[] = []
    root.traverse((o) => {
      if (/^P_(pri|sec|ter)_[LR]_\d\d$/.test(o.name) || /^P_rect_\d\d$/.test(o.name)) featherNodes.push(o)
    })
    for (const f of featherNodes) {
      const wrap = new THREE.Group()
      wrap.name = 'R_' + f.name.slice(2)
      const kids = [...f.children]
      for (const k of kids) wrap.add(k)
      f.add(wrap)
      // geometry may live on the node itself (single-primitive mesh): re-home it in the wrapper
      if ((f as THREE.Mesh).isMesh) {
        const m = f as THREE.Mesh
        const clone = new THREE.Mesh(m.geometry, m.material)
        clone.name = f.name + '_geo'
        wrap.add(clone)
        m.geometry = new THREE.BufferGeometry()
      }
      let kind: FeatherRig['kind'] = 'rect'
      let side: FeatherRig['side'] = 'C'
      let index = 0
      const mm = f.name.match(/^P_(pri|sec|ter)_([LR])_(\d\d)$/)
      if (mm) {
        kind = mm[1] as 'pri' | 'sec' | 'ter'
        side = mm[2] as Side
        index = parseInt(mm[3]) - 1
      } else {
        index = parseInt(f.name.slice(7)) - 1
      }
      // rachis direction in parent space -> fan angle
      _v.set(1, 0, 0).applyQuaternion(f.quaternion)
      const angle0 = kind === 'rect' ? Math.atan2(_v.x, -_v.z) : Math.atan2(-_v.z, _v.x)
      this.feathers.push({ wrap, kind, side, index, angle0 })
    }
    // root membranes pleat (shorten chordwise) as the wing folds
    for (const s of ['L', 'R'] as Side[]) {
      const m = root.getObjectByName(`P_membrane_${s}`)
      if (!m) continue
      const wrap = new THREE.Group()
      wrap.name = `R_membrane_${s}`
      for (const k of [...m.children]) wrap.add(k)
      if ((m as THREE.Mesh).isMesh) {
        const mm = m as THREE.Mesh
        wrap.add(new THREE.Mesh(mm.geometry, mm.material))
        mm.geometry = new THREE.BufferGeometry()
      }
      m.add(wrap)
      this.membranes.push({ wrap, side: s })
    }
    root.traverse((o) => {
      const m = o.name.match(/^K_iris_([LR])_\d$/)
      if (m) this.iris.push({ node: o, q0: o.quaternion.clone(), side: m[1] as Side })
    })
  }

  private rot(name: string, ...rots: [THREE.Vector3, number][]) {
    const j = this.joints.get(name)
    if (!j) return
    j.node.quaternion.copy(j.q0)
    for (const [ax, a] of rots) j.node.quaternion.multiply(_q.setFromAxisAngle(ax, a))
  }

  private rotSide(name: string, side: Side, ...rots: [THREE.Vector3, number][]) {
    const j = this.joints.get(name)
    if (!j) return
    j.node.quaternion.copy(j.q0)
    for (const [ax, a] of rots) {
      const axis = axisFor(side, ax).clone()
      j.node.quaternion.multiply(_q.setFromAxisAngle(axis, a))
    }
  }

  apply(p: Pose) {
    for (const s of ['L', 'R'] as Side[]) {
      const w = p.wing[s]
      this.rotSide(`J_shoulder_${s}`, s, [Z, w.stroke])
      this.rotSide(`J_sweep_${s}`, s, [Y, w.sweep])
      this.rotSide(`J_armPitch_${s}`, s, [X, w.pitch])
      // drawing-parallels linkage: one fold actuator flexes the elbow forward and the wrist back
      const fold = w.fold * WRIST_MAX
      this.rotSide(`J_elbow_${s}`, s, [Y, -w.fold * ELBOW_MAX])
      this.rotSide(`J_wrist_${s}`, s, [Y, fold])
      this.rotSide(`J_handTwist_${s}`, s, [X, w.handTwist])
      this.rotSide(`J_alula_${s}`, s, [Y, -0.55 * w.alula])
      const rod = this.joints.get(`J_foldRod_${s}`)
      if (rod) rod.node.position.copy(rod.p0).add(_v.set(0.0085 * Math.sin(fold), 0, 0).applyQuaternion(rod.q0))
      // scapular stow cover: drops over the wing root only once the wing is folded past ~60 %
      const cover = THREE.MathUtils.smoothstep(w.fold, 0.55, 0.95)
      this.rotSide(`J_scapular_${s}`, s, [Z, -SCAP_RETRACT * (1 - cover)])
      this.rotSide(`J_hip_${s}`, s, [X, p.hip[s]])
      this.rotSide(`J_ankle_${s}`, s, [X, p.ankle[s]])
      // toes: each segment curls (hallux and outer toe a little less at the base)
      for (let t = 1; t <= 4; t++) {
        for (let k = 0; k < 3; k++) {
          const base = k === 0 ? 0.55 : 0.85
          this.rotSide(`J_toe_${s}_${t}_${k}`, s, [Z, -p.toe[s] * base])
        }
      }
      this.rot(`J_eye_${s}`, [Y, p.eyeYaw], [X, p.eyePitch])
      this.rotSide(`J_lidU_${s}`, s, [X, 1.36 * p.blink])
      this.rotSide(`J_lidL_${s}`, s, [X, -1.36 * p.blink])
    }
    this.rot('J_neckPitch', [X, p.neckPitch])
    this.rot('J_neckYaw', [Y, p.neckYaw])
    this.rot('J_tailPitch', [X, p.tailPitch])
    this.rot('J_tailTilt', [Z, p.tailTilt])

    for (const ir of this.iris) {
      ir.node.quaternion.copy(ir.q0).multiply(_q.setFromAxisAngle(axisFor(ir.side, Y).clone(), -0.32 * p.iris))
    }

    for (const m of this.membranes) {
      const f = p.wing[m.side].fold
      m.wrap.scale.set(1, 1 - 0.25 * f, 1 - 0.8 * THREE.MathUtils.smoothstep(f, 0, 0.8))
    }
    for (const f of this.feathers) {
      let fan = 0
      let twist = 0
      let bend = 0
      if (f.kind === 'rect') {
        const a = f.angle0
        fan = -(a * 0.9 * p.tailSpread)
        twist = 0
        f.wrap.quaternion.setFromAxisAngle(Y, fan)
        continue
      }
      const w = p.wing[f.side as Side]
      if (f.kind === 'pri') {
        const k = f.index
        // folding stacks every primary onto the outermost one (parallel to the hand spar)
        const c = THREE.MathUtils.smoothstep(w.fold, 0.0, 0.85)
        const compress = -THREE.MathUtils.degToRad(PRI_A[k] - 9 + 0.6 * k) * c
        const spread = w.fan * (k - 4.5) / 4.5
        fan = compress - spread
        twist = w.featherOpen * (0.35 + 0.65 * (k / 9)) * (1 - 0.8 * w.fold)
        bend = w.bend * (0.3 + 0.7 * (k / 9)) * (1 - w.fold)
      } else if (f.kind === 'ter') {
        // tertials swing in against the humeral spar as the wing folds
        fan = -THREE.MathUtils.degToRad(70 + 6 * f.index) * THREE.MathUtils.smoothstep(w.fold, 0.0, 0.9)
        twist = 0
      } else {
        const j = f.index // 0 = outer
        // secondaries swing toward the folded hand so the closed wing reads as one stacked panel
        const c = THREE.MathUtils.smoothstep(w.fold, 0.0, 0.9)
        const base = 82 + 1.4 * j
        const target = SEC_FOLD_TARGET + 1.0 * j
        fan = THREE.MathUtils.degToRad(target - base) * c
        twist = w.featherOpen * 0.18 * (1 - j / 11) * (1 - w.fold)
      }
      const ax = axisFor(f.side as Side, Y).clone()
      f.wrap.quaternion.setFromAxisAngle(ax, fan).multiply(_q2.setFromAxisAngle(X, twist))
      if (bend !== 0) f.wrap.quaternion.multiply(_q2.setFromAxisAngle(axisFor(f.side as Side, Z).clone(), bend))
    }
  }
}
