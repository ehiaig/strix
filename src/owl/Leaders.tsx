import * as THREE from 'three'
import { useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import type { Director } from './director'

/**
 * Exploded-view trace lines: thin dashed lines from each exploded component back to the
 * point where it attaches, the convention used in engineering assembly drawings.
 */
export function Leaders({ director }: { director: Director }) {
  const { lines, geo, mat } = useMemo(() => {
    const n = director.assembly.leaders.length
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 6), 3))
    const mat = new THREE.LineDashedMaterial({ color: '#cdbb9c', dashSize: 0.006, gapSize: 0.005, transparent: true, opacity: 0, depthWrite: false })
    const lines = new THREE.LineSegments(geo, mat)
    lines.name = 'LEADERS'
    lines.frustumCulled = false
    lines.renderOrder = 4
    lines.raycast = () => {}
    director.owl.add(lines)
    return { lines, geo, mat }
  }, [director])
  const inv = useMemo(() => new THREE.Matrix4(), [])
  const v = useMemo(() => new THREE.Vector3(), [])

  useFrame(() => {
    const a = director.assembly
    const prog = a.progress
    const target = THREE.MathUtils.smoothstep(prog, 0.55, 0.95) * 0.55
    mat.opacity = target
    lines.visible = target > 0.01
    if (!lines.visible) return
    inv.copy(director.owl.matrixWorld).invert()
    const pos = geo.attributes.position as THREE.BufferAttribute
    a.leaders.forEach((l, i) => {
      l.part.node.getWorldPosition(v).applyMatrix4(inv)
      pos.setXYZ(i * 2, v.x, v.y, v.z)
      pos.setXYZ(i * 2 + 1, l.p0.x, l.p0.y, l.p0.z)
    })
    pos.needsUpdate = true
    lines.computeLineDistances()
    geo.computeBoundingSphere()
  })
  return null
}
