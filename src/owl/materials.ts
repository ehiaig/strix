import * as THREE from 'three'

/**
 * Physically based material library. Blender exports only material *names*;
 * every surface is re-created here so the look is tuned in one place.
 * Procedural maps (carbon twill, brushed grain, feather barbs) are generated on
 * canvases at load time, so the GLB carries geometry only.
 */

function canvas(w: number, h: number) {
  const c = document.createElement('canvas')
  c.width = w
  c.height = h
  return c
}

function rng(seed: number) {
  let s = seed >>> 0
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0
    return s / 4294967296
  }
}

/** Height field -> tangent-space normal map. */
function heightToNormal(h: Float32Array, w: number, hgt: number, strength: number) {
  const c = canvas(w, hgt)
  const ctx = c.getContext('2d')!
  const img = ctx.createImageData(w, hgt)
  for (let y = 0; y < hgt; y++) {
    for (let x = 0; x < w; x++) {
      const xl = h[y * w + ((x - 1 + w) % w)]
      const xr = h[y * w + ((x + 1) % w)]
      const yu = h[((y - 1 + hgt) % hgt) * w + x]
      const yd = h[((y + 1) % hgt) * w + x]
      let nx = (xl - xr) * strength
      let ny = (yd - yu) * strength
      let nz = 1
      const l = Math.hypot(nx, ny, nz)
      nx /= l; ny /= l; nz /= l
      const i = (y * w + x) * 4
      img.data[i] = (nx * 0.5 + 0.5) * 255
      img.data[i + 1] = (ny * 0.5 + 0.5) * 255
      img.data[i + 2] = (nz * 0.5 + 0.5) * 255
      img.data[i + 3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.NoColorSpace
  t.anisotropy = 8
  return t
}

function carbonTextures() {
  const N = 256
  const h = new Float32Array(N * N)
  const col = canvas(N, N)
  const ctx = col.getContext('2d')!
  const img = ctx.createImageData(N, N)
  const tow = 16 // 2x2 twill with 16px tows
  const r = rng(7)
  for (let y = 0; y < N; y++) {
    for (let x = 0; x < N; x++) {
      const tx = Math.floor(x / tow), ty = Math.floor(y / tow)
      const over = ((tx + ty) >> 1) % 2 === 0 // twill diagonal
      const fx = (x % tow) / tow, fy = (y % tow) / tow
      const along = over ? fx : fy
      const across = over ? fy : fx
      const bulge = Math.sin(Math.PI * across) * 0.8 + 0.2 * Math.sin(Math.PI * along)
      const fibre = 0.06 * Math.sin(across * 60 + r() * 0.5)
      h[y * N + x] = bulge + fibre + (over ? 0.25 : 0)
      const v = (over ? 34 : 22) + bulge * 10 + fibre * 40
      const i = (y * N + x) * 4
      img.data[i] = v; img.data[i + 1] = v + 1; img.data[i + 2] = v + 4; img.data[i + 3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  const map = new THREE.CanvasTexture(col)
  map.wrapS = map.wrapT = THREE.RepeatWrapping
  map.colorSpace = THREE.SRGBColorSpace
  map.anisotropy = 8
  return { map, normal: heightToNormal(h, N, N, 2.2) }
}

function brushedTextures(seed: number) {
  const W = 512, H = 64
  const h = new Float32Array(W * H)
  const r = rng(seed)
  // long horizontal streaks
  const rows = new Float32Array(H)
  for (let y = 0; y < H; y++) rows[y] = r()
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      h[y * W + x] = rows[y] * 0.6 + 0.4 * rows[(y + 1) % H] + 0.08 * Math.sin(x * 0.05 + rows[y] * 10)
    }
  }
  const rc = canvas(W, H)
  const ctx = rc.getContext('2d')!
  const img = ctx.createImageData(W, H)
  for (let i = 0; i < W * H; i++) {
    const v = 150 + h[i] * 70
    img.data[i * 4] = v; img.data[i * 4 + 1] = v; img.data[i * 4 + 2] = v; img.data[i * 4 + 3] = 255
  }
  ctx.putImageData(img, 0, 0)
  const rough = new THREE.CanvasTexture(rc)
  rough.wrapS = rough.wrapT = THREE.RepeatWrapping
  rough.colorSpace = THREE.NoColorSpace
  rough.anisotropy = 8
  return { normal: heightToNormal(h, W, H, 0.9), rough }
}

/** Fine bead-blast / ceramic grain: tiny roughness breakup + micro normal. */
function grainTextures() {
  const N = 256
  const h = new Float32Array(N * N)
  const r = rng(23)
  for (let i = 0; i < N * N; i++) h[i] = r()
  // blur once for a softer grain
  const b = new Float32Array(N * N)
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    let acc = 0
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) acc += h[((y + dy + N) % N) * N + ((x + dx + N) % N)]
    b[y * N + x] = acc / 9
  }
  const rc = canvas(N, N)
  const ctx = rc.getContext('2d')!
  const img = ctx.createImageData(N, N)
  for (let i = 0; i < N * N; i++) {
    const v = 190 + (b[i] - 0.5) * 110
    img.data[i * 4] = v; img.data[i * 4 + 1] = v; img.data[i * 4 + 2] = v; img.data[i * 4 + 3] = 255
  }
  ctx.putImageData(img, 0, 0)
  const rough = new THREE.CanvasTexture(rc)
  rough.wrapS = rough.wrapT = THREE.RepeatWrapping
  rough.colorSpace = THREE.NoColorSpace
  return { rough, normal: heightToNormal(b, N, N, 3.0) }
}

/**
 * Breast/flank markings: tapered vertical streaks with fine cross-bars (tawny/eagle-owl breast
 * pattern), printed as a subtle pad-print on the ceramic. One tile = 60 mm on the shell (planar UVs).
 */
function markingTextures() {
  const N = 512
  const c = canvas(N, N)
  const ctx = c.getContext('2d')!
  ctx.fillStyle = '#ffffff'
  ctx.fillRect(0, 0, N, N)
  const r = rng(77)
  // faint vermiculation
  for (let i = 0; i < 3200; i++) {
    const x = r() * N, y = r() * N, l = 3 + r() * 8
    ctx.strokeStyle = `rgba(70,64,56,${0.03 + r() * 0.04})`
    ctx.lineWidth = 1 + r()
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + l, y + (r() - 0.5) * 3); ctx.stroke()
  }
  // soft tapered drop-streaks, staggered, varied in size and strength
  const marks: [number, number, number, number][] = []
  const rows = 5, cols = 4
  for (let row = 0; row < rows; row++) for (let col = 0; col < cols; col++) {
    const x = (col + 0.5 + (row % 2) * 0.5) * (N / cols) + (r() - 0.5) * 34
    const y = (row + 0.5) * (N / rows) + (r() - 0.5) * 30
    marks.push([x, y, 0.6 + r() * 0.6, 0.35 + r() * 0.3])
  }
  ctx.filter = 'blur(1.6px)'
  const drawMark = (x: number, y: number, s: number, a: number) => {
    const L = 64 * s, W = 7.5 * s
    ctx.fillStyle = `rgba(58,52,46,${a})`
    ctx.beginPath()
    ctx.moveTo(x, y - L / 2)
    ctx.bezierCurveTo(x + W * 0.6, y - L / 6, x + W * 1.1, y + L / 4, x, y + L / 2)
    ctx.bezierCurveTo(x - W * 1.1, y + L / 4, x - W * 0.6, y - L / 6, x, y - L / 2)
    ctx.fill()
  }
  for (const [x, y, s, a] of marks) for (const dx of [-N, 0, N]) for (const dy of [-N, 0, N]) drawMark(x + dx, y + dy, s, a)
  ctx.filter = 'none'
  const map = new THREE.CanvasTexture(c)
  map.wrapS = map.wrapT = THREE.RepeatWrapping
  map.colorSpace = THREE.SRGBColorSpace
  map.anisotropy = 8
  return { map }
}

function featherTextures() {
  // UV: u across chord (0 = leading edge, 1 = trailing), v along rachis
  const W = 256, H = 1024
  const h = new Float32Array(W * H)
  const col = canvas(W, H)
  const ctx = col.getContext('2d')!
  const img = ctx.createImageData(W, H)
  const r = rng(11)
  for (let y = 0; y < H; y++) {
    const v = y / H
    for (let x = 0; x < W; x++) {
      const u = x / W
      // barbs: fine lines angled away from the rachis toward the tip
      const ang = 0.55
      const t = (v * 180 + Math.abs(u - 0.32) * 180 * ang)
      const barb = Math.pow(0.5 + 0.5 * Math.sin(t * Math.PI * 2), 3)
      h[y * W + x] = barb * 0.6 + r() * 0.05
      // subtle owl barring: soft transverse bands, stronger on the trailing vane
      const band = 0.5 + 0.5 * Math.sin(v * Math.PI * 2 * 5.5 + Math.sin(u * 3.0) * 0.6)
      const bar = Math.pow(band, 3) * (0.3 + 0.7 * u) * (0.75 + 0.25 * Math.sin(x * 0.11 + y * 0.023))
      const base = 46 + barb * 7
      const i = (y * W + x) * 4
      img.data[i] = base + bar * 36
      img.data[i + 1] = base + bar * 31
      img.data[i + 2] = base + bar * 25
      img.data[i + 3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  const map = new THREE.CanvasTexture(col)
  map.wrapS = map.wrapT = THREE.RepeatWrapping
  map.colorSpace = THREE.SRGBColorSpace
  map.anisotropy = 8
  return { map, normal: heightToNormal(h, W, H, 1.4) }
}

export type MaterialLib = Record<string, THREE.Material>

export function createMaterials(): MaterialLib {
  const carbon = carbonTextures()
  carbon.map.repeat.set(1, 1)
  const brushed = brushedTextures(3)
  const feather = featherTextures()
  const grain = grainTextures()
  const marks = markingTextures()

  const P = (o: THREE.MeshPhysicalMaterialParameters) => new THREE.MeshPhysicalMaterial(o)

  const lib: MaterialLib = {
    ti_brushed: P({
      color: new THREE.Color('#8f8c87'), metalness: 1, roughness: 0.46, roughnessMap: brushed.rough,
      normalMap: brushed.normal, normalScale: new THREE.Vector2(0.25, 0.25), envMapIntensity: 1.0,
    }),
    ti_fairing: P({
      color: new THREE.Color('#a39e95'), metalness: 1, roughness: 0.34, roughnessMap: brushed.rough,
      normalMap: brushed.normal, normalScale: new THREE.Vector2(0.22, 0.22), envMapIntensity: 1.15,
      clearcoat: 0.25, clearcoatRoughness: 0.3,
    }),
    graphite: P({ color: new THREE.Color('#2b2d31'), metalness: 0.3, roughness: 0.5, roughnessMap: grain.rough, clearcoat: 0.25, clearcoatRoughness: 0.4 }),
    graphite_satin: P({ color: new THREE.Color('#202226'), metalness: 0.25, roughness: 0.4, clearcoat: 0.55, clearcoatRoughness: 0.3 }),
    ceramic: P({
      color: new THREE.Color('#d2cec5'), metalness: 0.0, roughness: 0.48, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.05, 0.05), clearcoat: 0.4, clearcoatRoughness: 0.35,
      sheen: 0.15, sheenColor: new THREE.Color('#ffffff'), sheenRoughness: 0.6,
    }),
    shell_dark: P({
      color: new THREE.Color('#3b3d42'), metalness: 0.05, roughness: 0.55, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.06, 0.06), clearcoat: 0.25, clearcoatRoughness: 0.45,
      sheen: 0.2, sheenColor: new THREE.Color('#9da1aa'), sheenRoughness: 0.5,
    }),
    covert: P({
      color: new THREE.Color('#5a5650'), metalness: 0.05, roughness: 0.62, map: feather.map, normalMap: feather.normal,
      normalScale: new THREE.Vector2(0.3, 0.3), envMapIntensity: 0.7, side: THREE.DoubleSide,
    }),
    accent: P({ color: new THREE.Color('#b8692c'), metalness: 1, roughness: 0.38 }),
    cable: P({ color: new THREE.Color('#d8d3c6'), metalness: 0, roughness: 0.7 }),
    ceramic_dark: P({ color: new THREE.Color('#141518'), metalness: 0.0, roughness: 0.22, clearcoat: 1.0, clearcoatRoughness: 0.08 }),
    ceramic_barred: P({
      color: new THREE.Color('#d2cdc3'), map: marks.map, metalness: 0.0, roughness: 0.5, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.05, 0.05), clearcoat: 0.5, clearcoatRoughness: 0.2,
      sheen: 0.15, sheenColor: new THREE.Color('#ffffff'), sheenRoughness: 0.6,
    }),
    shell_barred: P({
      color: new THREE.Color('#6a6761'), map: marks.map, metalness: 0.05, roughness: 0.52, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.06, 0.06), clearcoat: 0.5, clearcoatRoughness: 0.22,
    }),
    shell_marked: P({
      color: new THREE.Color('#57544f'), map: marks.map, metalness: 0.05, roughness: 0.55, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.06, 0.06), clearcoat: 0.55, clearcoatRoughness: 0.22,
      sheen: 0.2, sheenColor: new THREE.Color('#a29d94'), sheenRoughness: 0.5,
    }),
    ceramic_mid: P({
      color: new THREE.Color('#8b867d'), metalness: 0.0, roughness: 0.5, roughnessMap: grain.rough, normalMap: grain.normal,
      normalScale: new THREE.Vector2(0.05, 0.05), clearcoat: 0.3, clearcoatRoughness: 0.4,
    }),
    beak: P({ color: new THREE.Color('#2b2926'), metalness: 0.0, roughness: 0.46, roughnessMap: grain.rough, clearcoat: 0.25, clearcoatRoughness: 0.5 }),
    carbon: P({
      color: new THREE.Color('#ffffff'), map: carbon.map, normalMap: carbon.normal, normalScale: new THREE.Vector2(0.35, 0.35),
      metalness: 0.2, roughness: 0.42, clearcoat: 1.0, clearcoatRoughness: 0.09,
    }),
    alu: P({
      color: new THREE.Color('#b4b5b8'), metalness: 1, roughness: 0.4, roughnessMap: brushed.rough,
      normalMap: brushed.normal, normalScale: new THREE.Vector2(0.1, 0.1),
    }),
    alu_dark: P({ color: new THREE.Color('#202125'), metalness: 0.8, roughness: 0.48, roughnessMap: grain.rough }),
    steel: P({ color: new THREE.Color('#a9abae'), metalness: 1, roughness: 0.3 }),
    membrane: P({
      color: new THREE.Color('#18191b'), metalness: 0, roughness: 0.78, sheen: 0.8, sheenColor: new THREE.Color('#5a5c60'),
      sheenRoughness: 0.45, side: THREE.DoubleSide,
    }),
    feather: P({
      color: new THREE.Color('#ffffff'), map: feather.map, normalMap: feather.normal, normalScale: new THREE.Vector2(0.45, 0.45),
      metalness: 0.05, roughness: 0.64, envMapIntensity: 0.7, side: THREE.DoubleSide,
      sheen: 0.25, sheenColor: new THREE.Color('#8a8379'), sheenRoughness: 0.6,
    }),
    rachis: P({ color: new THREE.Color('#3b3732'), metalness: 0.2, roughness: 0.45 }),
    glass: P({
      color: new THREE.Color('#0d1014'), metalness: 0, roughness: 0.02, transparent: true, opacity: 0.22,
      clearcoat: 1, clearcoatRoughness: 0.0, ior: 1.52, specularIntensity: 1, envMapIntensity: 1.6, depthWrite: false,
    }),
    lens_inner: P({
      color: new THREE.Color('#07080b'), metalness: 0.5, roughness: 0.04, iridescence: 1.0, iridescenceIOR: 1.7,
      iridescenceThicknessRange: [220, 520], clearcoat: 1, clearcoatRoughness: 0,
    }),
    gold: P({ color: new THREE.Color('#c79a4b'), metalness: 1, roughness: 0.32 }),
    amber: P({ color: new THREE.Color('#2a1404'), emissive: new THREE.Color('#ff9531'), emissiveIntensity: 1.1, roughness: 0.3 }),
    amber_plastic: P({ color: new THREE.Color('#a8611c'), roughness: 0.38, metalness: 0, clearcoat: 0.5 }),
    led: P({ color: new THREE.Color('#2a1404'), emissive: new THREE.Color('#ffb04a'), emissiveIntensity: 16.0, roughness: 0.3 }),
    copper: P({ color: new THREE.Color('#b9774a'), metalness: 1, roughness: 0.3 }),
    rubber: P({ color: new THREE.Color('#141415'), metalness: 0, roughness: 0.88 }),
    pcb: P({ color: new THREE.Color('#0f1a16'), metalness: 0.1, roughness: 0.55, clearcoat: 0.6, clearcoatRoughness: 0.3 }),
    chip: P({ color: new THREE.Color('#0c0c0d'), metalness: 0.1, roughness: 0.45 }),
    connector: P({ color: new THREE.Color('#e3dfd6'), metalness: 0, roughness: 0.6 }),
    flex: P({ color: new THREE.Color('#a8611c'), metalness: 0.15, roughness: 0.35, clearcoat: 0.7, clearcoatRoughness: 0.2 }),
  }
  for (const [k, m] of Object.entries(lib)) {
    m.name = k
    sanitize(m)
  }
  return lib
}

/**
 * Guard against NaN/Inf fragments (degenerate tangent frames at grazing angles can produce
 * them). A single NaN pixel would otherwise spread through the bloom mip chain.
 */
export function sanitize(m: THREE.Material) {
  m.onBeforeCompile = (sh) => {
    sh.fragmentShader = sh.fragmentShader.replace(
      '#include <dithering_fragment>',
      `#include <dithering_fragment>
       if (!(gl_FragColor.r >= 0.0 && gl_FragColor.g >= 0.0 && gl_FragColor.b >= 0.0)) gl_FragColor.rgb = vec3(0.0);
       gl_FragColor.rgb = min(gl_FragColor.rgb, vec3(48.0));`,
    )
  }
  m.customProgramCacheKey = () => 'sanitized'
}

/** Highlight clones (subtle warm lift) for hover feedback. */
export function highlightOf(base: THREE.Material, cache: Map<THREE.Material, THREE.Material>) {
  let h = cache.get(base)
  if (!h) {
    const c = (base as THREE.MeshPhysicalMaterial).clone()
    c.emissive = new THREE.Color('#ffae5a')
    c.emissiveIntensity = 0.16
    sanitize(c)
    cache.set(base, c)
    h = c
  }
  return h
}
