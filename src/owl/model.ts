import * as THREE from 'three'
import { useLoader } from '@react-three/fiber'
import { useGLTF } from '@react-three/drei'
import { GLTFLoader, MeshoptDecoder, type GLTF } from 'three-stdlib'
import MESHOPT_URL from '../assets/strix.glb?url'

/**
 * Model loading.
 *
 * The model ships meshopt-compressed (≈4.7 MB), which needs WebAssembly to decode. Two build-time options
 * exist for hosts with restrictions (both are used by `npm run build:hosted`, neither by `npm run build`):
 *
 *  - VITE_GLB_FALLBACK=<file>: an uncompressed, lighter-tessellation copy placed next to the main GLB, used
 *    when the page's content-security policy refuses WebAssembly;
 *  - VITE_GLB_JSON=1: the GLB files are published base64-wrapped as `<name>.glb.json` (`{ "glb": "…" }`),
 *    for hosts that only serve standard web file types.
 */
function wasmAllowed() {
  try {
    new WebAssembly.Module(new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0]))
    return true
  } catch {
    return false
  }
}
const FALLBACK = import.meta.env.VITE_GLB_FALLBACK as string | undefined
const AS_JSON = import.meta.env.VITE_GLB_JSON === '1'
export const USE_MESHOPT = !FALLBACK || (typeof WebAssembly !== 'undefined' && wasmAllowed())
const GLB_URL = USE_MESHOPT ? MESHOPT_URL : new URL(FALLBACK!, new URL(MESHOPT_URL, location.href)).href
export const MODEL_URL = AS_JSON ? GLB_URL + '.json' : GLB_URL

function base64ToBuffer(b64: string) {
  const s = atob(b64)
  const out = new Uint8Array(s.length)
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i)
  return out.buffer
}

/** Loads `{ "glb": "<base64>" }` and parses it as a binary glTF (no fetch of data:/blob: URLs needed). */
class EmbeddedGLBLoader extends THREE.Loader<GLTF> {
  load(url: string, onLoad: (gltf: GLTF) => void, onProgress?: (e: ProgressEvent) => void, onError?: (e: unknown) => void) {
    const files = new THREE.FileLoader(this.manager)
    files.setResponseType('json')
    files.load(
      url,
      (data) => {
        try {
          const loader = new GLTFLoader(this.manager)
          if (USE_MESHOPT) loader.setMeshoptDecoder(typeof MeshoptDecoder === 'function' ? (MeshoptDecoder as any)() : MeshoptDecoder)
          loader.parse(base64ToBuffer((data as any).glb), '', onLoad, onError)
        } catch (e) {
          onError?.(e)
        }
      },
      onProgress,
      onError,
    )
  }
}

export function useOwlModel(): GLTF {
  // AS_JSON is a build-time constant, so the hook order never changes
  // eslint-disable-next-line react-hooks/rules-of-hooks
  return AS_JSON ? (useLoader(EmbeddedGLBLoader as any, MODEL_URL) as GLTF) : (useGLTF(MODEL_URL, false, USE_MESHOPT) as unknown as GLTF)
}

export function preloadOwlModel() {
  if (AS_JSON) useLoader.preload(EmbeddedGLBLoader as any, MODEL_URL)
  else useGLTF.preload(MODEL_URL, false, USE_MESHOPT)
}
