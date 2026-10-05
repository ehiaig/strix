import * as THREE from 'three'
import gsap from 'gsap'
import { EXPLODE, LAYOUT, type ExplodeSpec } from './parts'

/**
 * Detach / assemble controller.
 *
 * Every explodable node (A_* assemblies and P_* parts) keeps its canonical local
 * transform from the GLB. Its exploded state is described by a progress value p ∈ [0,1]:
 *
 *   position(p)   = pos0 + release·e1(p) + offset·e2(p)
 *   quaternion(p) = q0 · slerp(I, qExplode, e2(p))
 *
 * where offsets are authored in owl space and converted once (in the neutral pose) into
 * the node's parent frame. At p = 0 the canonical transform is copied back verbatim, so
 * any number of DETACH/ASSEMBLE cycles cannot drift.
 */

const ease = (t: number) => (t <= 0 ? 0 : t >= 1 ? 1 : t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2)
const clamp01 = (t: number) => Math.min(1, Math.max(0, t))
// phases inside one part's progress
const SCREW = [0.0, 0.16]
const RELEASE = [0.1, 0.34]
const MAIN = [0.24, 1.0]
const phase = (p: number, r: number[]) => ease(clamp01((p - r[0]) / (r[1] - r[0])))

export interface ExplodePart {
  name: string
  node: THREE.Object3D
  spec: ExplodeSpec
  pos0: THREE.Vector3
  q0: THREE.Quaternion
  offset: THREE.Vector3
  release: THREE.Vector3
  qx: THREE.Quaternion
  prog: { v: number }
  fasteners?: FastenerSet
}

export interface FastenerSet {
  mesh: THREE.InstancedMesh
  base: THREE.Matrix4[]
}

const _m = new THREE.Matrix4()
const _m2 = new THREE.Matrix4()
const _qa = new THREE.Quaternion()
const _qb = new THREE.Quaternion()
const _q = new THREE.Quaternion()
const _qi = new THREE.Quaternion()
const _v = new THREE.Vector3()
const _s = new THREE.Vector3()
const Yax = new THREE.Vector3(0, 1, 0)

export class AssemblyController {
  parts: ExplodePart[] = []
  byName = new Map<string, ExplodePart>()
  tl: gsap.core.Timeline | null = null
  explodedBox = new THREE.Box3()
  leaders: { part: ExplodePart; p0: THREE.Vector3 }[] = []
  private dirty = true

  /** absolute exploded offsets (owl space) produced by the stack layout */
  layoutOffsets = new Map<string, THREE.Vector3>()

  constructor(owl: THREE.Object3D, fastenerSets: Map<string, FastenerSet>) {
    owl.updateMatrixWorld(true)
    const owlInv = new THREE.Matrix4().copy(owl.matrixWorld).invert()
    this.computeLayout(owl, owlInv)
    const absD = new Map<THREE.Object3D, THREE.Vector3>()

    const visit = (o: THREE.Object3D, ancD: THREE.Vector3) => {
      let myD = ancD
      const spec = EXPLODE[o.name]
      if (spec && (o.name.startsWith('A_') || o.name.startsWith('P_'))) {
        const d = spec.d ? new THREE.Vector3(...spec.d) : ancD.clone().add(new THREE.Vector3(...(spec.rel ?? [0, 0, 0])))
        const relOwl = d.clone().sub(ancD)
        // parent rotation in owl space
        _m.multiplyMatrices(owlInv, o.parent!.matrixWorld)
        _m.decompose(_v, _q, _s)
        const qParInv = _q.clone().invert()
        // parent frames may carry a uniform scale (body scale / wing counter-scale): divide it out
        const pScale = _s.x
        const offset = relOwl.applyQuaternion(qParInv).divideScalar(pScale)
        if (spec.dl) offset.add(new THREE.Vector3(...spec.dl).applyQuaternion(o.quaternion))
        const release = new THREE.Vector3(...(spec.release ?? [0, 0, 0])).applyQuaternion(qParInv).divideScalar(pScale)
        const r = spec.rot ?? [0, 0, 0]
        const qx = new THREE.Quaternion().setFromEuler(new THREE.Euler(...(r.map(THREE.MathUtils.degToRad) as [number, number, number])))
        const part: ExplodePart = {
          name: o.name, node: o, spec, pos0: o.position.clone(), q0: o.quaternion.clone(), offset, release, qx,
          prog: { v: 0 }, fasteners: fastenerSets.get(o.name),
        }
        this.parts.push(part)
        this.byName.set(o.name, part)
        myD = d
      }
      absD.set(o, myD)
      for (const c of o.children) visit(c, myD)
    }
    visit(owl, new THREE.Vector3())

    // measure the fully exploded envelope (owl space) for camera framing
    for (const p of this.parts) p.prog.v = 1
    this.dirty = true
    this.update()
    owl.updateMatrixWorld(true)
    this.explodedBox.makeEmpty()
    const tmp = new THREE.Box3()
    owl.traverse((o) => {
      const m = o as THREE.Mesh
      if (m.isMesh && m.geometry && m.visible) {
        if (!m.geometry.boundingBox) m.geometry.computeBoundingBox()
        if (!m.geometry.boundingBox) return
        tmp.copy(m.geometry.boundingBox).applyMatrix4(_m2.multiplyMatrices(owlInv, m.matrixWorld))
        if (Number.isFinite(tmp.min.x)) this.explodedBox.union(tmp)
      }
    })
    for (const p of this.parts) p.prog.v = 0
    this.dirty = true
    this.update()
    owl.updateMatrixWorld(true)
    for (const p of this.parts) {
      if (!p.spec.lead) continue
      const w = p.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(owlInv)
      this.leaders.push({ part: p, p0: w })
    }
  }

  /**
   * Turn the stack description (parts.ts LAYOUT) into absolute offsets: every tier is pushed along
   * its stack axis until it clears the previous tier (or the stack base) by exactly the stack gap.
   * Boxes are measured on the real geometry in the neutral pose, so the gaps stay uniform whatever
   * the part sizes are.
   */
  private computeLayout(owl: THREE.Object3D, owlInv: THREE.Matrix4) {
    const byName = new Map<string, THREE.Object3D>()
    owl.traverse((o) => { if (o.name) byName.set(o.name, o) })
    // parts with a purposeful exploded rotation are measured in that orientation
    const saved: [THREE.Object3D, THREE.Quaternion][] = []
    for (const [n, sp] of Object.entries(EXPLODE)) {
      const o = byName.get(n)
      if (o && sp.rot) {
        saved.push([o, o.quaternion.clone()])
        o.quaternion.multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(...(sp.rot.map(THREE.MathUtils.degToRad) as [number, number, number]))))
      }
    }
    owl.updateMatrixWorld(true)
    const tmp = new THREE.Box3(), mm = new THREE.Matrix4()
    const boxOf = (name: string, exclude: Set<string>) => {
      const b = new THREE.Box3()
      const root = byName.get(name)
      if (!root) return b
      const walk = (o: THREE.Object3D) => {
        if (o !== root && exclude.has(o.name)) return
        const m = o as THREE.Mesh
        if (m.isMesh && m.geometry && !(m as any).isInstancedMesh) {
          if (!m.geometry.boundingBox) m.geometry.computeBoundingBox()
          tmp.copy(m.geometry.boundingBox!).applyMatrix4(mm.multiplyMatrices(owlInv, m.matrixWorld))
          if (Number.isFinite(tmp.min.x)) b.union(tmp)
        }
        for (const c of o.children) walk(c)
      }
      walk(root)
      return b
    }
    const off = this.layoutOffsets
    const offOf = (name: string) => {
      for (let o: THREE.Object3D | null = byName.get(name) ?? null; o && o !== owl; o = o.parent) {
        const v = off.get(o.name)
        if (v) return v
      }
      return new THREE.Vector3()
    }
    const members = new Set<string>()
    for (const st of LAYOUT) for (const t of st.tiers) for (const n of t) members.add(n)
    const ax = ['x', 'y', 'z'] as const
    for (const st of LAYOUT) {
      const a = ax[st.axis]
      const same = new Set<string>()
      for (const t of st.tiers) for (const n of t) same.add(n)
      const bOff = offOf(st.base)
      const bb = boxOf(st.base, members).translate(bOff)
      let cursor = st.sign > 0 ? bb.max[a] : bb.min[a]
      for (const tier of st.tiers) {
        const tb = new THREE.Box3()
        for (const n of tier) tb.union(boxOf(n, same))
        if (tb.isEmpty()) continue
        tb.translate(bOff)
        const near = st.sign > 0 ? tb.min[a] : tb.max[a]
        let sc = cursor + st.sign * st.gap - near
        if (sc * st.sign < 0.006) sc = 0.006 * st.sign // always move at least a little outward
        const v = new THREE.Vector3()
        v[a] = sc
        for (const n of tier) off.set(n, bOff.clone().add(v))
        cursor = (st.sign > 0 ? tb.max[a] : tb.min[a]) + sc
      }
    }
    for (const [o, q] of saved) o.quaternion.copy(q)
    owl.updateMatrixWorld(true)
    // write the result into the specs: layout members get absolute offsets, every other
    // explodable assembly simply rides with its ancestor (feathers keep their small local slides)
    for (const [n, sp] of Object.entries(EXPLODE)) {
      const v = off.get(n)
      if (v) {
        sp.d = [v.x, v.y, v.z]
        sp.rel = undefined
        sp.dl = undefined
      } else if (!sp.dl) {
        sp.d = undefined
        sp.rel = [0, 0, 0]
      }
    }
  }

  /** mean progress (0 = assembled, 1 = exploded) */
  get progress() {
    let s = 0
    for (const p of this.parts) s += p.prog.v
    return this.parts.length ? s / this.parts.length : 0
  }

  get busy() {
    return !!this.tl && this.tl.isActive()
  }

  update() {
    for (const p of this.parts) {
      const v = p.prog.v
      const n = p.node
      if (v <= 0) {
        n.position.copy(p.pos0)
        n.quaternion.copy(p.q0)
      } else {
        const e1 = phase(v, RELEASE)
        const e2 = phase(v, MAIN)
        n.position.copy(p.pos0).addScaledVector(p.release, e1).addScaledVector(p.offset, e2)
        _q.copy(_qi.identity()).slerp(p.qx, e2)
        n.quaternion.copy(p.q0).multiply(_q)
      }
      if (p.fasteners) this.updateFasteners(p.fasteners, v)
    }
  }

  private updateFasteners(f: FastenerSet, v: number) {
    const s = phase(v, SCREW)
    const last = (f as any)._last as number | undefined
    if (last === s) return
    ;(f as any)._last = s
    for (let i = 0; i < f.base.length; i++) {
      if (s <= 0) {
        f.mesh.setMatrixAt(i, f.base[i])
      } else {
        _m.makeRotationAxis(Yax, -s * Math.PI * 4)
        _m.setPosition(0, 0.0055 * s, 0)
        _m2.multiplyMatrices(f.base[i], _m)
        f.mesh.setMatrixAt(i, _m2)
      }
    }
    f.mesh.instanceMatrix.needsUpdate = true
  }

  kill() {
    if (this.tl) {
      this.tl.kill()
      this.tl = null
    }
  }

  /** Animate toward exploded (target=1) or assembled (target=0), continuing from the current state. */
  run(target: 0 | 1, onDone: () => void, speed = 1) {
    this.kill()
    const remaining = target === 1 ? 1 - this.progress : this.progress
    const k = Math.max(0.15, remaining)
    const tl = gsap.timeline({ paused: false })
    for (const p of this.parts) {
      const dist = Math.abs(target - p.prog.v)
      if (dist < 1e-6) continue
      const delay = (target === 1 ? p.spec.detach : p.spec.assemble) * k / speed
      const dur = Math.max(0.12, (p.spec.dur ?? 1.15) * Math.max(0.3, dist)) / speed
      tl.to(p.prog, { v: target, duration: dur, ease: 'none' }, delay)
    }
    tl.eventCallback('onComplete', () => {
      for (const p of this.parts) p.prog.v = target
      this.update()
      this.tl = null
      onDone()
    })
    if (tl.getChildren().length === 0) {
      tl.kill()
      onDone()
      return
    }
    this.tl = tl
  }

  /** max deviation from canonical transforms (metres / radians) — used by tests */
  deviation() {
    let dp = 0, dq = 0
    for (const p of this.parts) {
      dp = Math.max(dp, p.node.position.distanceTo(p.pos0))
      // normalise first: GLB rotations are float32, and angleTo() of a slightly non-unit quaternion with itself is not 0
      dq = Math.max(dq, _qa.copy(p.node.quaternion).normalize().angleTo(_qb.copy(p.q0).normalize()))
    }
    return { dp, dq }
  }
}
