import * as THREE from 'three'
import { useEffect } from 'react'
import { useThree } from '@react-three/fiber'
import { useUI } from '../store'
import { lerpPose, makePose, clonePose, setSecFoldTarget } from '../owl/rig'

/**
 * Automation hooks used by the screenshot critic loop and the functional tests.
 * window.__owl exposes the Director plus helpers to step time deterministically
 * and to place the camera. Nothing here is needed for normal use.
 */
export function DebugBridge() {
  const director = useUI((s) => s.director)
  const { camera, controls, gl, scene, invalidate } = useThree() as any
  useEffect(() => {
    if (!director) return
    const api = {
      director,
      THREE,
      makePose,
      lerpPose,
      clonePose,
      setSecFoldTarget,
      gl,
      scene,
      camera,
      /** advance simulated time without rendering (seconds of real time) */
      advance(seconds: number, step = 1 / 60) {
        const wasPaused = director.paused
        director.paused = false
        let t = 0
        while (t < seconds - 1e-9) {
          const h = Math.min(step, seconds - t)
          director.update(h)
          t += h
        }
        director.paused = wasPaused
      },
      pause(p = true) {
        director.paused = p
      },
      setView(target: [number, number, number], azDeg: number, elDeg: number, dist: number) {
        const t = new THREE.Vector3(...target)
        const az = THREE.MathUtils.degToRad(azDeg), el = THREE.MathUtils.degToRad(elDeg)
        camera.position.set(t.x + dist * Math.sin(az) * Math.cos(el), t.y + dist * Math.sin(el), t.z + dist * Math.cos(az) * Math.cos(el))
        if (controls) {
          controls.target.copy(t)
          controls.update()
        } else camera.lookAt(t)
      },
      /** view relative to the robot's current root position */
      viewOwl(azDeg: number, elDeg: number, dist: number, dy = 0) {
        const p = director.owlRoot.position
        this.setView([p.x, p.y + dy, p.z], azDeg, elDeg, dist)
      },
      /** render one frame (demand mode) and resolve after it is presented */
      frame() {
        return new Promise<void>((res) => {
          invalidate()
          requestAnimationFrame(() => requestAnimationFrame(() => res()))
        })
      },
      /** solve shoulder stroke/sweep/pitch so the arm frame matches world-space targets (perched body) */
      solveWing(ta: number[], tn: number[], fold: number, apply = true) {
        const d = director
        const arm = d.owl.getObjectByName('A_arm_L')!
        const q = new THREE.Quaternion()
        const qb = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(-62))
        const qbi = qb.clone().invert()
        const A = new THREE.Vector3(...ta).normalize().applyQuaternion(qbi)
        const N = new THREE.Vector3(...tn).applyQuaternion(qbi)
        N.sub(A.clone().multiplyScalar(N.dot(A))).normalize()
        const pose = clonePose(d.perched)
        const err = (x: number[]) => {
          pose.wing.L.stroke = x[0]; pose.wing.L.sweep = x[1]; pose.wing.L.pitch = x[2]
          d.rig.apply(pose)
          const m = new THREE.Matrix4()
          for (let n: THREE.Object3D | null = arm; n && n !== d.owl; n = n.parent) { n.updateMatrix(); m.premultiply(n.matrix) }
          m.decompose(new THREE.Vector3(), q, new THREE.Vector3())
          const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(q), ny = new THREE.Vector3(0, 1, 0).applyQuaternion(q)
          return (1 - ax.dot(A)) * 4 + (1 - ny.dot(N))
        }
        let x = [-1.0, 0.3, -0.3], e = err(x), step = 0.4
        for (let it = 0; it < 4000 && step > 1e-5; it++) {
          let imp = false
          for (let k = 0; k < 3; k++) for (const sg of [1, -1]) {
            const y = x.slice(); y[k] += sg * step
            const ey = err(y)
            if (ey < e) { x = y; e = ey; imp = true }
          }
          if (!imp) step *= 0.5
        }
        if (apply) {
          for (const s of ['L', 'R'] as const) {
            d.perched.wing[s].stroke = x[0]; d.perched.wing[s].sweep = x[1]; d.perched.wing[s].pitch = x[2]; d.perched.wing[s].fold = fold
          }
          d.pivotRoot(THREE.MathUtils.degToRad(-62), d.perchRootPos, d.perchRootQuat)
          this.advance(0.05)
        }
        return { deg: x.map((v) => +THREE.MathUtils.radToDeg(v).toFixed(1)), err: e }
      },
      /** override the perched wing stow (degrees / fold 0..1) and re-seat the robot on the perch */
      setPerch(w: Record<string, number>, bodyPitch?: number) {
        const d = director
        for (const s of ['L', 'R'] as const) {
          const W = d.perched.wing[s] as any
          for (const [k, v] of Object.entries(w)) W[k] = k === 'fold' || k === 'featherOpen' || k === 'fan' ? v : THREE.MathUtils.degToRad(v)
        }
        ;(d as any).balancePerch()
        d.pivotRoot(THREE.MathUtils.degToRad(bodyPitch ?? -62), d.perchRootPos, d.perchRootQuat)
        d.pose = clonePose(d.perched)
        d.rig.apply(d.pose)
        d.owlRoot.position.copy(d.perchRootPos)
        d.owlRoot.quaternion.copy(d.perchRootQuat)
        this.advance(0.05)
        return 'ok'
      },
      /** run the flight sim until a named moment, then frame the robot relative to its heading */
      flyTo(target: string, el = 12) {
        const d = director
        if (!d.sim) d.command('FLY')
        for (let i = 0; i < 60 * 80; i++) {
          this.advance(1 / 60)
          const sim = d.sim
          if (!sim) return 'nosim'
          const fr = sim.frame
          const ph = sim.wingPhase
          const ok: Record<string, boolean> = {
            prep: fr.phase === 'prep' && sim.time > 0.9,
            takeoff: fr.phase === 'launch' && sim.s > 1.2 && Math.abs(ph - 0.3) < 0.03,
            down: fr.phase === 'cruise' && sim.s > 22 && Math.abs(sim.roll) < 0.1 && Math.abs(ph - 0.28) < 0.03,
            up: fr.phase === 'cruise' && sim.s > 22 && Math.abs(sim.roll) < 0.1 && Math.abs(ph - 0.86) < 0.03,
            bank: fr.phase === 'cruise' && Math.abs(sim.roll) > 0.45 && Math.abs(ph - 0.3) < 0.03,
            glide: fr.phase === 'glide' && (sim as any).glideW > 0.9,
            land: fr.phase === 'approach' && sim.s > sim.plan.length - 1.0 && Math.abs(ph - 0.3) < 0.05,
          }
          if (ok[target]) {
            const p = d.owlRoot.position
            const f = new THREE.Vector3(0, 0, 1).applyQuaternion(d.owlRoot.quaternion)
            f.y = 0
            f.normalize()
            const yaw = THREE.MathUtils.radToDeg(Math.atan2(f.x, f.z))
            const az = ({ prep: 28, takeoff: 35, down: 55, up: 55, bank: 55, glide: 20, land: 25 } as Record<string, number>)[target]
            const dist = ({ prep: 1.3, takeoff: 1.7, down: 1.9, up: 1.9, bank: 2.0, glide: 2.0, land: 1.6 } as Record<string, number>)[target]
            this.setView([p.x, p.y, p.z], yaw + az, el, dist)
            return `${target} ${fr.phase} s=${sim.s.toFixed(1)} v=${sim.v.toFixed(1)} roll=${THREE.MathUtils.radToDeg(sim.roll).toFixed(0)} clock=${d.clock.toFixed(1)}`
          }
        }
        return 'notfound ' + target
      },
      info() {
        const r = gl.info.render
        return { calls: r.calls, triangles: r.triangles, geometries: gl.info.memory.geometries, textures: gl.info.memory.textures }
      },
    }
    ;(window as any).__owl = api
  }, [director, controls])
  return null
}
