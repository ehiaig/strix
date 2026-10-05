import * as THREE from 'three'
import { Suspense, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { PerformanceMonitor, AdaptiveDpr } from '@react-three/drei'
import { EffectComposer, N8AO, Bloom, Vignette, ToneMapping, SMAA } from '@react-three/postprocessing'
import { ToneMappingMode } from 'postprocessing'
import { RobotOwl } from './owl/RobotOwl'
import { Studio } from './scene/Studio'
import { CameraRig } from './scene/CameraRig'
import { DebugBridge } from './scene/DebugBridge'
import { Controls, InfoCard, Header, Loader } from './ui/Overlay'

const params = new URLSearchParams(location.search)
const LOW = params.has('low')
const FIXED = params.has('hq') || params.has('capture')
const NOPOST = params.has('nopost')
const NOAO = params.has('noao')
const NOBLOOM = params.has('nobloom')
const MS = params.has('ms0') ? 0 : 4
const FBT = params.has('u8') ? THREE.UnsignedByteType : params.has('f32') ? THREE.FloatType : THREE.HalfFloatType

export default function App() {
  const [quality, setQuality] = useState(LOW ? 0 : 2) // 0 low, 1 medium, 2 high
  return (
    <>
      <Canvas
        frameloop={params.has('capture') ? 'demand' : 'always'}
        shadows={quality > 0 ? 'soft' : false}
        dpr={quality === 2 ? [1, 2] : [1, 1.25]}
        camera={{ fov: 30, near: 0.02, far: 80, position: [0.6, 1.2, 1.3] }}
        gl={{ antialias: false, powerPreference: 'high-performance', preserveDrawingBuffer: params.has('capture') }}
        onCreated={({ gl }) => {
          gl.toneMapping = NOPOST ? THREE.ACESFilmicToneMapping : THREE.NoToneMapping
          gl.outputColorSpace = THREE.SRGBColorSpace
        }}
      >
        {FIXED ? null : <PerformanceMonitor onDecline={() => setQuality((q) => Math.max(0, q - 1))} flipflops={2} />}
        <Suspense fallback={null}>
          <Studio shadows={quality > 0} />
          <RobotOwl />
          <CameraRig />
          <DebugBridge />
        </Suspense>
        {NOPOST ? null : <EffectComposer multisampling={quality === 2 ? MS : 0} enableNormalPass={false} frameBufferType={FBT}>
          {quality > 0 && !NOAO ? <N8AO aoRadius={0.06} distanceFalloff={0.6} intensity={2.2} quality={quality === 2 ? 'high' : 'medium'} halfRes={quality < 2} /> : <></>}
          {NOBLOOM ? <></> : <Bloom intensity={0.45} luminanceThreshold={3} luminanceSmoothing={1} mipmapBlur />}
          <ToneMapping mode={ToneMappingMode.ACES_FILMIC} />
          <Vignette offset={0.28} darkness={0.5} />
          {quality < 2 ? <SMAA /> : <></>}
        </EffectComposer>}
        {quality < 2 && !FIXED ? <AdaptiveDpr pixelated={false} /> : null}
      </Canvas>
      <Header />
      <InfoCard />
      <Controls />
      <Loader />
    </>
  )
}
