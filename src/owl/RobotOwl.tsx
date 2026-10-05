import * as THREE from 'three'
import { useMemo, useEffect, useRef } from 'react'
import { useFrame, useThree, type ThreeEvent } from '@react-three/fiber'
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js'
import { createMaterials, highlightOf } from './materials'
import { OwlRig } from './rig'
import { AssemblyController, type FastenerSet } from './assembly'
import { Director, NEUTRAL } from './director'
import { ui } from '../store'
import { infoFor } from './parts'
import { Leaders } from './Leaders'
import { Labels } from './Labels'
import { useOwlModel, preloadOwlModel } from './model'
import report from '../../blender/build_report.json'

const ENCLOSED = ['P_battery', 'P_fc', 'P_vision', 'P_pdu', 'P_frame', 'P_rootMotor_L', 'P_rootMotor_R', 'P_capstan_L', 'P_capstan_R']

function setup(scene: THREE.Group) {
  const mats = createMaterials()
  const owl = scene.getObjectByName('OWL')!
  const perch = scene.getObjectByName('PERCH')!
  const tpl = scene.getObjectByName('TPL_screw')

  // ---------------------------------------------------------------- materials & shadows
  const swap = (m: THREE.Material) => mats[m.name] ?? m
  // parts fully enclosed by the shells never cast a visible shadow while assembled (and the exploded robot
  // casts none), so they are left out of the shadow pass
  const enclosed = new Set<THREE.Object3D>()
  for (const n of ENCLOSED) scene.getObjectByName(n)?.traverse((o) => enclosed.add(o))
  scene.traverse((o) => {
    const mesh = o as THREE.Mesh
    if (!mesh.isMesh) return
    mesh.material = Array.isArray(mesh.material) ? mesh.material.map(swap) : swap(mesh.material)
    const name = (mesh.material as THREE.Material).name
    const transparent = name === 'glass'
    mesh.userData.shadow = !transparent && !enclosed.has(mesh)
    mesh.castShadow = mesh.userData.shadow
    mesh.receiveShadow = true
    if (transparent) mesh.renderOrder = 5
  })

  // ---------------------------------------------------------------- instanced fasteners
  const fastenerSets = new Map<string, FastenerSet>()
  if (tpl) {
    tpl.updateMatrixWorld(true)
    const geos: THREE.BufferGeometry[] = []
    const tmats: THREE.Material[] = []
    tpl.traverse((o) => {
      const m = o as THREE.Mesh
      if (m.isMesh) {
        const g = m.geometry.clone()
        g.applyMatrix4(new THREE.Matrix4().copy(tpl.matrixWorld).invert().multiply(m.matrixWorld))
        for (const k of Object.keys(g.attributes)) if (!['position', 'normal'].includes(k)) g.deleteAttribute(k)
        geos.push(g)
        tmats.push(m.material as THREE.Material)
      }
    })
    tpl.parent?.remove(tpl)
    // (the lighter fallback model can lose the tiny screw template to simplification: then no screws are drawn)
    const geo = geos.length ? mergeGeometries(geos.map((g) => (g.index ? g.toNonIndexed() : g)), true) : null
    const byParent = new Map<THREE.Object3D, THREE.Object3D[]>()
    owl.traverse((o) => {
      if (o.name.startsWith('FAST_') && o.parent) {
        const arr = byParent.get(o.parent) ?? []
        arr.push(o)
        byParent.set(o.parent, arr)
      }
    })
    for (const [parent, list] of byParent) {
      if (!geo) break
      const inst = new THREE.InstancedMesh(geo, tmats, list.length)
      inst.name = 'INST_' + parent.name
      inst.castShadow = true
      inst.receiveShadow = true
      const base: THREE.Matrix4[] = []
      list.forEach((f, i) => {
        f.updateMatrix()
        base.push(f.matrix.clone())
        inst.setMatrixAt(i, f.matrix)
      })
      inst.instanceMatrix.needsUpdate = true
      inst.computeBoundingSphere()
      for (const f of list) parent.remove(f)
      parent.add(inst)
      fastenerSets.set(parent.name, { mesh: inst, base })
    }
  }

  // ---------------------------------------------------------------- rig, assembly, director
  const rig = new OwlRig(owl)
  rig.apply(NEUTRAL)
  owl.updateMatrixWorld(true)
  const assembly = new AssemblyController(owl, fastenerSets)

  const owlRoot = new THREE.Group()
  owlRoot.name = 'OWL_ROOT'
  owl.parent?.remove(owl)
  owlRoot.add(owl)
  const root = new THREE.Group()
  root.add(owlRoot)
  perch.parent?.remove(perch)
  root.add(perch)
  // the stand fades out for the exploded view (own material copies, so the robot is unaffected)
  const perchMats: THREE.Material[] = []
  perch.traverse((o) => {
    const m = o as THREE.Mesh
    if (m.isMesh) {
      const c = (m.material as THREE.Material).clone()
      c.onBeforeCompile = (m.material as THREE.Material).onBeforeCompile
      c.customProgramCacheKey = (m.material as THREE.Material).customProgramCacheKey
      m.material = c
      perchMats.push(c)
    }
  })

  const perchBar = new THREE.Vector3(0, (report as any).perch_bar_height_m ?? 0.92, 0)
  const director = new Director(owlRoot, owl, rig, assembly, perchBar)
  return { root, owl, director, mats, perch, perchMats }
}

function partOf(o: THREE.Object3D | null): THREE.Object3D | null {
  while (o) {
    if (o.name.startsWith('P_') || (o.name.startsWith('A_') && infoFor(o.name))) {
      if (infoFor(o.name)) return o
    }
    o = o.parent
  }
  return null
}

export function RobotOwl() {
  const gltf = useOwlModel()
  const { root, director, perch, perchMats } = useMemo(() => setup(gltf.scene as THREE.Group), [gltf])
  const perchFade = useRef(1)
  const shadowState = useRef(true)
  const { camera, pointer } = useThree()
  const hlCache = useRef(new Map<THREE.Material, THREE.Material>())
  const hlState = useRef<{ part: THREE.Object3D | null; saved: Map<THREE.Mesh, THREE.Material | THREE.Material[]> }>({ part: null, saved: new Map() })
  const look = useMemo(() => new THREE.Vector3(), [])

  useEffect(() => {
    ui.set({ director })
    return () => ui.set({ director: null })
  }, [director])

  const setHighlight = (part: THREE.Object3D | null) => {
    const hs = hlState.current
    if (hs.part === part) return
    for (const [m, mat] of hs.saved) m.material = mat
    hs.saved.clear()
    hs.part = part
    if (part) {
      part.traverse((o) => {
        const m = o as THREE.Mesh
        if (!m.isMesh || (m as any).isInstancedMesh) return
        hs.saved.set(m, m.material)
        m.material = Array.isArray(m.material)
          ? m.material.map((x) => highlightOf(x, hlCache.current))
          : highlightOf(m.material, hlCache.current)
      })
    }
  }

  const interactive = () => director.state === 'ASSEMBLED' || director.state === 'DETACHED'

  useFrame((_, dt) => {
    // pointer -> a look target in front of the camera for idle head tracking
    // (a point between the viewer's eye and the pointer projected to the robot's depth)
    const depth = camera.position.distanceTo(director.owlRoot.position)
    look.set(pointer.x * 0.9, pointer.y * 0.6, 0.5).unproject(camera)
    look.sub(camera.position).normalize().multiplyScalar(depth * 0.4).add(camera.position)
    director.update(dt, look)
    // no cast shadows while exploded (a spread-out assembly smears a large shadow over the floor)
    const castNow = director.assembly.progress < 0.5
    if (castNow !== shadowState.current) {
      shadowState.current = castNow
      director.owl.traverse((o) => {
        const m = o as THREE.Mesh
        if (m.isMesh) m.castShadow = castNow && m.userData.shadow !== false
      })
    }
    // fade the display stand while the robot is exploded
    const fade = 1 - THREE.MathUtils.smoothstep(director.assembly.progress, 0.02, 0.3)
    if (Math.abs(fade - perchFade.current) > 1e-3 || (fade === 1) !== (perchFade.current === 1)) {
      perchFade.current = fade
      for (const m of perchMats) {
        m.transparent = fade < 1
        m.opacity = fade
        m.depthWrite = fade > 0.5
      }
      perch.visible = fade > 0.01
      perch.traverse((o) => { o.castShadow = fade > 0.5 })
    }
    if (!interactive() && hlState.current.part) {
      setHighlight(null)
      ui.set({ hovered: null })
    }
  })

  const onMove = (e: ThreeEvent<PointerEvent>) => {
    if (!interactive()) return
    e.stopPropagation()
    const part = partOf(e.object)
    setHighlight(part)
    const name = part?.name ?? null
    if (ui.get().hovered !== name) ui.set({ hovered: name })
    document.body.style.cursor = part ? 'pointer' : ''
  }
  const onOut = () => {
    setHighlight(null)
    ui.set({ hovered: null })
    document.body.style.cursor = ''
  }
  const onClick = (e: ThreeEvent<MouseEvent>) => {
    if (!interactive()) return
    e.stopPropagation()
    const part = partOf(e.object)
    ui.set({ pinned: part?.name ?? null })
  }

  return (
    <>
      <primitive object={root} onPointerMove={onMove} onPointerOut={onOut} onClick={onClick} />
      <Leaders director={director} />
      <Labels director={director} />
    </>
  )
}

preloadOwlModel()
