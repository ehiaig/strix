import * as THREE from 'three'
import { useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { useUI } from '../store'
import { Environment, Lightformer } from '@react-three/drei'

export const STUDIO = {
  bg: '#1c1d20', // horizon / fog colour
  top: '#0c0d0f',
  floor: '#0e0f11',
}

/** Seamless cove: a backdrop dome whose horizon colour equals the fog colour the floor fades into. */
function Cove() {
  const mat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        side: THREE.BackSide,
        depthWrite: false,
        fog: false,
        uniforms: { horizon: { value: new THREE.Color(STUDIO.bg) }, top: { value: new THREE.Color(STUDIO.top) } },
        vertexShader: `varying vec3 vDir; void main(){ vDir = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
        fragmentShader: `uniform vec3 horizon; uniform vec3 top; varying vec3 vDir;
          void main(){ float e = clamp(vDir.y, 0.0, 1.0); vec3 c = mix(horizon, top, smoothstep(0.0, 0.35, e));
          gl_FragColor = vec4(c, 1.0);
          #include <colorspace_fragment>
          }`,
      }),
    [],
  )
  const ref = useRef<THREE.Mesh>(null!)
  useFrame(({ camera }) => ref.current && ref.current.position.copy(camera.position))
  return (
    <mesh ref={ref} material={mat} renderOrder={-10} frustumCulled={false}>
      <sphereGeometry args={[60, 48, 24]} />
    </mesh>
  )
}

/**
 * Studio floor: matte with a soft large-scale tonal variation that gives motion cues during the
 * flight demo. Distance fade comes from the scene fog, so it follows the camera.
 */
function Floor() {
  const director = useUI((s) => s.director)
  const uniforms = useMemo(() => ({ uCenter: { value: new THREE.Vector2() }, uHorizon: { value: new THREE.Color(STUDIO.bg) } }), [])
  const mat = useMemo(() => {
    const m = new THREE.MeshStandardMaterial({ color: STUDIO.floor, roughness: 0.82, metalness: 0 })
    m.onBeforeCompile = (sh) => {
      sh.uniforms.uCenter = uniforms.uCenter
      sh.uniforms.uHorizon = uniforms.uHorizon
      sh.vertexShader = sh.vertexShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vWorldPosFloor;')
        .replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\nvWorldPosFloor = (modelMatrix * vec4(transformed,1.0)).xyz;')
      sh.fragmentShader = sh.fragmentShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vWorldPosFloor;\nuniform vec2 uCenter;\nuniform vec3 uHorizon;')
        .replace(
          '#include <fog_fragment>',
          `// soft, large-scale tonal variation (polished-concrete feel) for motion cues in flight
           vec2 g = vWorldPosFloor.xz * 0.45;
           vec2 i0 = floor(g), f0 = fract(g);
           vec2 u = f0 * f0 * (3.0 - 2.0 * f0);
           float h00 = fract(sin(dot(i0, vec2(127.1, 311.7))) * 43758.5453);
           float h10 = fract(sin(dot(i0 + vec2(1.0, 0.0), vec2(127.1, 311.7))) * 43758.5453);
           float h01 = fract(sin(dot(i0 + vec2(0.0, 1.0), vec2(127.1, 311.7))) * 43758.5453);
           float h11 = fract(sin(dot(i0 + vec2(1.0, 1.0), vec2(127.1, 311.7))) * 43758.5453);
           float n = mix(mix(h00, h10, u.x), mix(h01, h11, u.x), u.y);
           gl_FragColor.rgb *= 0.9 + 0.2 * n;
           float pool = smoothstep(1.5, 8.0, distance(vWorldPosFloor.xz, uCenter));
           gl_FragColor.rgb = mix(gl_FragColor.rgb, uHorizon, pool);
           #include <fog_fragment>`,
        )
    }
    m.customProgramCacheKey = () => 'studio-floor'
    return m
  }, [uniforms])
  useFrame(() => {
    if (director) uniforms.uCenter.value.set(director.owlRoot.position.x, director.owlRoot.position.z)
  })
  return (
    <mesh rotation-x={-Math.PI / 2} receiveShadow material={mat}>
      <planeGeometry args={[160, 160]} />
    </mesh>
  )
}

export function Studio({ shadows = true }: { shadows?: boolean }) {
  return (
    <>
      <color attach="background" args={[STUDIO.top]} />
      <fog attach="fog" args={[STUDIO.bg, 4, 26]} />
      <Cove />
      <Floor />
      <hemisphereLight args={['#c9ccd2', '#4a4640', 0.45]} />
      <KeyLight shadows={shadows} />
      {/* fill: cool, low right */}
      <directionalLight position={[-3.5, 1.6, 2.0]} intensity={0.45} color="#d6e2ff" />
      {/* rims / kickers: separate the dark shells from the background */}
      <directionalLight position={[-2.2, 2.4, -3.2]} intensity={3.0} color="#e8eeff" />
      <directionalLight position={[2.6, 1.4, -2.8]} intensity={2.0} color="#ffd9b0" />
      <directionalLight position={[-3.0, 1.2, -0.6]} intensity={1.1} color="#dfe6ff" />
      {/* soft bounce from below: keeps wing undersides readable in flight */}
      <directionalLight position={[0.6, -3.0, 1.2]} intensity={0.7} color="#e9e2d6" />
      <directionalLight position={[0, 4.5, -1.0]} intensity={0.6} color="#ffffff" />
      <Environment resolution={256} frames={1} environmentIntensity={0.75}>
        <color attach="background" args={['#0d0e10']} />
        <Lightformer form="rect" intensity={1.5} color="#ffffff" position={[0, 6, 1]} rotation-x={Math.PI / 2} scale={[8, 4, 1]} />
        <Lightformer form="rect" intensity={3.0} color="#fff3e6" position={[5, 2.5, 4]} rotation-y={-Math.PI / 3} scale={[3, 5, 1]} />
        <Lightformer form="rect" intensity={1.2} color="#dfe8ff" position={[-6, 2, 2]} rotation-y={Math.PI / 2.4} scale={[3, 6, 1]} />
        <Lightformer form="rect" intensity={2.4} color="#ffffff" position={[-2, 3, -6]} rotation-y={Math.PI} scale={[10, 1.2, 1]} />
        <Lightformer form="rect" intensity={1.8} color="#ffffff" position={[0, -1.5, 3]} rotation-x={-Math.PI / 2.2} scale={[8, 3, 1]} />
        <Lightformer form="ring" intensity={1.5} color="#ffe2c2" position={[3, 4, -3]} scale={2} />
      </Environment>
    </>
  )
}

/** Warm key light with a tight shadow frustum that follows the robot (sharp shadows in flight too). */
function KeyLight({ shadows }: { shadows: boolean }) {
  const light = useRef<THREE.DirectionalLight>(null!)
  const director = useUI((s) => s.director)
  const tgt = useMemo(() => new THREE.Object3D(), [])
  const offset = useMemo(() => new THREE.Vector3(2.4, 3.6, 3.0), [])
  useFrame(() => {
    if (!light.current) return
    const p = director ? director.owlRoot.position : new THREE.Vector3(0, 1.1, 0)
    const c = new THREE.Vector3(p.x, Math.min(p.y, 1.4) * 0.55, p.z)
    tgt.position.copy(c)
    light.current.position.copy(c).add(offset)
    tgt.updateMatrixWorld()
  })
  return (
    <>
      <primitive object={tgt} />
      <directionalLight
        ref={light}
        target={tgt}
        intensity={2.1}
        color="#fff2e4"
        castShadow={shadows}
        shadow-mapSize={[2048, 2048]}
        shadow-bias={-0.0002}
        shadow-normalBias={0.01}
        shadow-camera-left={-1.3}
        shadow-camera-right={1.3}
        shadow-camera-top={1.3}
        shadow-camera-bottom={-1.3}
        shadow-camera-near={0.5}
        shadow-camera-far={12}
      />
    </>
  )
}
