import * as THREE from 'three'
import { useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { Html } from '@react-three/drei'
import type { Director } from './director'

/**
 * Exploded-view callouts: a few hairline labels naming the main sub-assemblies, so the
 * engineering view reads at a glance without hovering. Only the left wing is labelled.
 */
type Side = 'top' | 'bottom' | 'left' | 'right'
const CALLOUTS: { node: string; text: string; side: Side; m: number }[] = [
  { node: 'A_headUnit', text: 'Sensor head · optics · facial discs', side: 'left', m: 0.03 },
  { node: 'P_shell_rear', text: 'Dorsal shell · scapular covers', side: 'top', m: 0.03 },
  { node: 'P_battery', text: 'Battery 6S · power · flight controller', side: 'right', m: 0.05 },
  { node: 'P_shell_front', text: 'Breast · belly · flank shells', side: 'left', m: 0.04 },
  { node: 'P_capstan_L', text: 'Wing drive: motor · belt · capstan', side: 'bottom', m: 0.07 },
  { node: 'P_shoulderHub_L', text: 'Stroke axle · drum · spring', side: 'top', m: 0.05 },
  { node: 'A_arm_L', text: 'Humeral section', side: 'top', m: 0.04 },
  { node: 'A_forearm_L', text: 'Forearm · secondaries', side: 'top', m: 0.04 },
  { node: 'A_hand_L', text: 'Hand · primaries', side: 'top', m: 0.04 },
  { node: 'A_tail', text: 'Tail fan · pitch, tilt and spread drives', side: 'top', m: 0.03 },
  { node: 'A_leg_L', text: 'Leg · catapult hip · grasping foot', side: 'right', m: 0.04 },
]

export function Labels({ director }: { director: Director }) {
  const items = useMemo(() => {
    const box = new THREE.Box3()
    const tmp = new THREE.Box3()
    return CALLOUTS.map((c) => {
      const node = director.owl.getObjectByName(c.node)
      return { ...c, node, box, tmp }
    }).filter((c) => !!c.node)
  }, [director])
  const refs = useRef<(HTMLDivElement | null)[]>([])
  const group = useRef<THREE.Group>(null)
  const anchors = useMemo(() => items.map(() => new THREE.Object3D()), [items])
  const v = useMemo(() => new THREE.Vector3(), [])

  useFrame(() => {
    const prog = director.assembly.progress
    const op = THREE.MathUtils.smoothstep(prog, 0.8, 0.98)
    items.forEach((it, i) => {
      const el = refs.current[i]
      if (el) el.style.opacity = String(op)
      if (op <= 0) return
      // anchor above/below the centre of the part's current (exploded) bounds
      it.box.makeEmpty()
      it.node!.updateWorldMatrix(true, true)
      // the node's own geometry, excluding nested sub-assemblies (A_*) that carry their own callouts
      const root = it.node!
      root.traverse((o) => {
        const m = o as THREE.Mesh
        if (!m.isMesh || !m.geometry) return
        if (m !== root) for (let p: THREE.Object3D | null = m.parent; p && p !== root; p = p.parent) if (p.name.startsWith('A_')) return
        if (!m.geometry.boundingBox) m.geometry.computeBoundingBox()
        it.tmp.copy(m.geometry.boundingBox!).applyMatrix4(m.matrixWorld)
        it.box.union(it.tmp)
      })
      if (it.box.isEmpty()) return
      it.box.getCenter(v)
      if (it.side === 'top') v.y = it.box.max.y + it.m
      else if (it.side === 'bottom') v.y = it.box.min.y - it.m
      else if (it.side === 'left') v.x = it.box.min.x - it.m
      else v.x = it.box.max.x + it.m
      anchors[i].position.copy(v)
    })
    if (group.current) group.current.visible = op > 0.01
  })

  return (
    <group ref={group}>
      {items.map((it, i) => (
        <primitive object={anchors[i]} key={it.node!.name}>
          <Html center zIndexRange={[20, 0]} style={{ pointerEvents: 'none' }}>
            <div className={`callout callout-${it.side}`} ref={(el) => (refs.current[i] = el)} style={{ opacity: 0 }}>
              {it.text}
            </div>
          </Html>
        </primitive>
      ))}
    </group>
  )
}
