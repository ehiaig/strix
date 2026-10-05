// Wraps every .glb in a directory as base64 JSON (<name>.glb.json = { "glb": "..." }) and removes the .glb,
// for static hosts that only serve standard web file types. Used by `npm run build:hosted`.
import { readdirSync, readFileSync, writeFileSync, unlinkSync } from 'node:fs'
import { join } from 'node:path'
const dir = process.argv[2]
for (const f of readdirSync(dir).filter((n) => n.endsWith('.glb'))) {
  const p = join(dir, f)
  writeFileSync(p + '.json', JSON.stringify({ glb: readFileSync(p).toString('base64') }))
  unlinkSync(p)
  console.log('embedded', f)
}
