"""
Functional tests for the STRIX-01 web app, run in headless Chromium (SwiftShader software GL).

    python3 tests/functional.py [BASE_URL]        # default http://localhost:5173/

UI commands go through the real buttons / keyboard shortcuts / mouse. Simulated time is stepped with
window.__owl.advance(), which runs exactly the same Director.update() code as a rendered frame, so
the long sequences (explode, assemble, a full flight) finish in seconds on a software renderer.
Tests that depend on rendering (camera, hover, head tracking, performance) use real frames.
Results are written to tests/results.json and printed as a table.
"""
import json, math, sys, time, os
from playwright.sync_api import sync_playwright

BASE = sys.argv[1] if len(sys.argv) > 1 else 'http://localhost:5173/'
OUT = os.path.join(os.path.dirname(__file__), 'results.json')
results = []


def record(tid, name, ok, detail):
    results.append({'id': tid, 'name': name, 'pass': bool(ok), 'detail': detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {tid:>2} {name}: {detail}", flush=True)


JS_HELPERS = r"""
window.__t = {
  state: () => window.__owl.director.state,
  snapshot() {
    // part/joint transforms relative to the robot root in a fixed reference pose
    const o = window.__owl, d = o.director, T = o.THREE
    const pose = o.clonePose(d.perched)
    d.rig.apply(pose)
    d.owl.updateMatrixWorld(true)
    const inv = new T.Matrix4().copy(d.owl.matrixWorld).invert()
    const out = {}
    d.owl.traverse((n) => {
      if (!/^(P_|A_|J_|K_)/.test(n.name)) return
      const m = new T.Matrix4().multiplyMatrices(inv, n.matrixWorld)
      out[n.name] = m.elements.slice()
    })
    d.rig.apply(d.pose)
    return out
  },
  compare(a, b) {
    const T = window.__owl.THREE
    let dp = 0, dq = 0, n = 0, de = 0
    for (const k of Object.keys(a)) {
      if (!b[k]) continue
      const ma = new T.Matrix4().fromArray(a[k]), mb = new T.Matrix4().fromArray(b[k])
      const pa = new T.Vector3(), qa = new T.Quaternion(), sa = new T.Vector3()
      const pb = new T.Vector3(), qb = new T.Quaternion(), sb = new T.Vector3()
      ma.decompose(pa, qa, sa); mb.decompose(pb, qb, sb)
      qa.normalize(); qb.normalize()
      dp = Math.max(dp, pa.distanceTo(pb)); dq = Math.max(dq, qa.angleTo(qb)); n++
      for (let i = 0; i < 16; i++) de = Math.max(de, Math.abs(a[k][i] - b[k][i]))
    }
    return { dp, dq, n, de }
  },
  runUntil(pred, maxS, step = 1 / 60) {
    const o = window.__owl
    let t = 0
    while (t < maxS) { o.advance(step); t += step; if (pred()) return t }
    return -1
  },
  partSpread() {
    // largest distance of any explodable part from its assembled position (owl space)
    const d = window.__owl.director, T = window.__owl.THREE
    let mx = 0
    for (const p of d.assembly.parts) mx = Math.max(mx, p.node.position.distanceTo(p.pos0) * (p.node.parent ? p.node.parent.getWorldScale(new T.Vector3()).x : 1))
    return mx
  },
  frames(n) {
    return new Promise((res) => { let k = 0; const f = () => { if (++k >= n) res(k); else requestAnimationFrame(f) }; requestAnimationFrame(f) })
  },
  settleCamera(maxFrames = 400) {
    // wait until the camera stops moving (OrbitControls damping and framing animations finish)
    const c = window.__owl.camera
    return new Promise((res) => {
      let k = 0, still = 0; const last = c.position.clone()
      const f = () => { k++; const d = c.position.distanceTo(last); last.copy(c.position); still = d < 1e-5 ? still + 1 : 0
        if (still >= 8 || k >= maxFrames) res(k); else requestAnimationFrame(f) }
      requestAnimationFrame(f)
    })
  },
  ndc(name) {
    const o = window.__owl, T = o.THREE
    const n = o.director.owl.getObjectByName(name)
    const b = new T.Box3().setFromObject(n)
    const c = b.getCenter(new T.Vector3()).project(o.camera)
    return [c.x, c.y, c.z]
  },
}
"""


def main():
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_page(viewport={'width': 1100, 'height': 700})
        logs = []
        pg.on('console', lambda m: logs.append((m.type, m.text)))
        pg.on('pageerror', lambda e: logs.append(('pageerror', str(e))))
        pg.set_default_timeout(300000)

        def boot(url):
            t0 = time.time()
            pg.goto(url)
            pg.wait_for_function('window.__owl && window.__owl.director', timeout=300000)
            pg.wait_for_selector('.loader.is-done', state='attached', timeout=300000)
            pg.evaluate(JS_HELPERS)
            return time.time() - t0

        def errors():
            return [l for l in logs if l[0] in ('error', 'pageerror') and 'TUNNEL' not in l[1]]

        ev = pg.evaluate
        btn = lambda c: pg.locator(f'button[data-cmd="{c}"]')

        # ---------------------------------------------------------------- 1. load
        t_load = boot(BASE)
        names = ev("""(() => { const s = new Set(); window.__owl.director.owl.traverse(n => s.add(n.name)); return [...s] })()""")
        need = ['A_core', 'A_head', 'A_wing_L', 'A_wing_R', 'A_tail', 'A_landing', 'J_shoulder_L', 'J_elbow_L', 'J_wrist_L',
                'J_sweep_R', 'J_neckYaw', 'P_rootMotor_L', 'P_pri_L_10', 'P_sec_R_11', 'P_rect_12', 'P_foot_L']
        missing = [n for n in need if n not in names]
        nparts = len([n for n in names if n.startswith('P_')])
        record(1, 'Loads; GLB hierarchy present; no console errors', not missing and not errors(),
               f'ready in {t_load:.1f}s (software GL); {nparts} P_ parts, {len([n for n in names if n.startswith("J_")])} joints; '
               f'missing={missing}; errors={errors()[:3]}')

        # ---------------------------------------------------------------- 2. initial state & button states
        st = ev('__t.state()')
        dis = {c: btn(c).is_disabled() for c in ('DETACH', 'ASSEMBLE', 'FLY')}
        record(2, 'Initial state ASSEMBLED with correct button enablement',
               st == 'ASSEMBLED' and dis == {'DETACH': False, 'ASSEMBLE': True, 'FLY': False}, f'state={st} disabled={dis}')

        # reference snapshot (fixed pose) for drift checks
        ev('window.__owl.pause(true)')
        ref = ev('__t.snapshot()')

        # ---------------------------------------------------------------- 4. DETACH
        ev('window.__owl.pause(true)')
        btn('DETACH').click()
        s_after_click = ev('__t.state()')
        t_det = ev("__t.runUntil(() => __t.state() === 'DETACHED', 20)")
        prog = ev('window.__owl.director.assembly.progress')
        spread = ev('__t.partSpread()')
        dis = {c: btn(c).is_disabled() for c in ('DETACH', 'ASSEMBLE', 'FLY')}
        ev('__t.frames(3)')
        callouts = ev("[...document.querySelectorAll('.callout')].filter(e => parseFloat(getComputedStyle(e).opacity) > 0.5).length")
        record(4, 'DETACH: staged explode to DETACHED with labelled exploded view',
               s_after_click == 'DETACHING' and t_det > 0 and abs(prog - 1) < 1e-9 and spread > 0.2 and callouts >= 5 and dis['DETACH'] and not dis['ASSEMBLE'],
               f'click->{s_after_click}; DETACHED after {t_det:.2f}s sim; progress={prog:.3f}; max part travel {spread:.2f} m; {callouts} visible callouts; disabled={dis}')

        # ---------------------------------------------------------------- 5. ASSEMBLE exact restore
        btn('ASSEMBLE').click()
        s_after = ev('__t.state()')
        t_as = ev("__t.runUntil(() => __t.state() === 'ASSEMBLED', 20)")
        dev = ev('window.__owl.director.assembly.deviation()')
        cmp = ev('__t.compare(arguments[0], __t.snapshot())'.replace('arguments[0]', json.dumps(ref)))
        record(5, 'ASSEMBLE: returns to ASSEMBLED with exact restoration',
               s_after == 'ASSEMBLING' and t_as > 0 and dev['dp'] < 1e-9 and cmp['dp'] < 1e-6 and cmp['dq'] < 1e-6,
               f'click->{s_after}; ASSEMBLED after {t_as:.2f}s sim; part deviation dp={dev["dp"]:.2e} m dq={dev["dq"]:.2e} rad; '
               f'{cmp["n"]} nodes vs load snapshot: dp={cmp["dp"]:.2e} m dq={cmp["dq"]:.2e} rad')

        # ---------------------------------------------------------------- 6. repeated cycles (drift)
        worst = {'dp': 0, 'dq': 0}
        for i in range(6):
            pg.keyboard.press('d'); ev("__t.runUntil(() => __t.state() === 'DETACHED', 20)")
            pg.keyboard.press('a'); ev("__t.runUntil(() => __t.state() === 'ASSEMBLED', 20)")
            c = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
            worst = {k: max(worst[k], c[k]) for k in worst}
        record(6, 'Six DETACH/ASSEMBLE cycles (keyboard D/A): no drift', worst['dp'] < 1e-6 and worst['dq'] < 1e-6,
               f'worst node error after any cycle: {worst["dp"]:.2e} m, {worst["dq"]:.2e} rad')

        # ---------------------------------------------------------------- 7. interrupt detach with assemble, and vice versa
        seq = []
        btn('DETACH').click(); ev("__t.runUntil(() => window.__owl.director.assembly.progress > 0.5, 20)")
        mid1 = ev('window.__owl.director.assembly.progress'); seq.append(ev('__t.state()'))
        btn('ASSEMBLE').click(); seq.append(ev('__t.state()'))
        ev("__t.runUntil(() => window.__owl.director.assembly.progress < 0.25, 20)"); mid2 = ev('window.__owl.director.assembly.progress')
        pg.keyboard.press('d'); seq.append(ev('__t.state()'))
        ev('window.__owl.advance(0.5)')
        pg.keyboard.press('a'); seq.append(ev('__t.state()'))
        t_end = ev("__t.runUntil(() => __t.state() === 'ASSEMBLED', 20)")
        c = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
        record(7, 'Interrupts: ASSEMBLE mid-detach, DETACH mid-assemble, ASSEMBLE again', t_end > 0 and c['dp'] < 1e-6 and c['dq'] < 1e-6
               and seq == ['DETACHING', 'ASSEMBLING', 'DETACHING', 'ASSEMBLING'],
               f'states {seq}; progress at interrupts {mid1:.2f}, {mid2:.2f}; settled ASSEMBLED after {t_end:.2f}s; node error {c["dp"]:.1e} m / {c["dq"]:.1e} rad')

        # ---------------------------------------------------------------- 8. full flight from ASSEMBLED
        flight_js = r"""(() => {
          const o = window.__owl, d = o.director, T = o.THREE
          const states = [], phases = []; let t = 0, maxJump = 0, minY = 1e9, maxRoll = 0, maxSpeed = 0
          const prev = d.owlRoot.position.clone(); let glide = false
          const samples = []
          while (t < 240) {
            o.advance(1/60); t += 1/60
            const s = d.state
            if (states[states.length-1] !== s) states.push(s)
            const sim = d.sim
            if (sim) {
              const ph = sim.frame.phase
              if (phases[phases.length-1] !== ph) phases.push(ph)
              maxRoll = Math.max(maxRoll, Math.abs(sim.roll)); maxSpeed = Math.max(maxSpeed, sim.v)
              if (ph === 'cruise' || ph === 'launch') samples.push([sim.time, d.pose.wing.L.stroke, d.pose.wing.R.stroke, d.pose.wing.L.fold, d.pose.wing.L.pitch, sim.roll, ph === 'cruise' ? 1 : 0])
            }
            const p = d.owlRoot.position
            maxJump = Math.max(maxJump, p.distanceTo(prev)); prev.copy(p)
            minY = Math.min(minY, p.y)
            if (s === 'ASSEMBLED' && t > 1) break
          }
          return { t, states, phases, maxJump, minY, maxRoll: T.MathUtils.radToDeg(maxRoll), maxSpeed, samples,
                   end: d.owlRoot.position.toArray(), perch: d.perchRootPos.toArray(), dev: d.assembly.deviation() }
        })()"""
        btn('FLY').click()
        s_click = ev('__t.state()')
        dis_fly = {c: btn(c).is_disabled() for c in ('DETACH', 'ASSEMBLE', 'FLY')}
        f = ev(flight_js)
        c = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
        ok_states = f['states'][:1] == ['TAKING_OFF'] and 'FLYING' in f['states'] and 'LANDING' in f['states'] and f['states'][-1] == 'ASSEMBLED'
        end_err = math.dist(f['end'], f['perch'])
        record(8, 'FLY: take-off, flight, landing, back on the perch', s_click == 'TAKING_OFF' and ok_states and 'glide' in f['phases']
               and end_err < 0.005 and f['maxJump'] < 0.25 and f['minY'] > 0.3 and c['dp'] < 1e-6,
               f'states {f["states"]}; phases {f["phases"]}; {f["t"]:.1f}s sim; max step {f["maxJump"]:.3f} m/frame (no teleports); '
               f'lowest root height {f["minY"]:.2f} m; peak speed {f["maxSpeed"]:.1f} m/s; final perch error {end_err*1000:.2f} mm; buttons during flight disabled={dis_fly}')

        # ---------------------------------------------------------------- 9. wingbeat kinematics
        # cruise samples: the course is mostly banked turns, where stroke-amplitude asymmetry (roll control) is
        # intended, so amplitude and frequency use the mean of both wings; L/R symmetry is checked wings-level
        allS = f['samples']
        C = [x for x in allS if x[6] == 1]
        level = [x for x in allS if x[6] == 0 and abs(x[5]) < math.radians(0.5)]  # straight climb-out after launch
        if len(C) > 50:
            m = [(x[1] + x[2]) / 2 for x in C]
            mean = sum(m) / len(m)
            periods, amps, last, seg = [], [], None, []
            for i in range(1, len(C)):
                if C[i][0] - C[i-1][0] > 0.05:
                    last, seg = None, []
                    continue
                seg.append(m[i])
                if m[i-1] < mean <= m[i]:
                    if last is not None:
                        periods.append(C[i][0] - last)
                        amps.append((max(seg) - min(seg)) / 2)
                    last, seg = C[i][0], []
            freq = len(periods) / sum(periods) if periods else 0
            amps.sort()
            amp = amps[len(amps) // 2] if amps else 0
            sym = max(abs(x[1] - x[2]) for x in level) if level else 99
            asym_turn = max(abs(x[1] - x[2]) for x in C)
            st = m
            fu = [C[i][3] for i in range(1, len(C)) if st[i] > st[i-1]]
            fd = [C[i][3] for i in range(1, len(C)) if st[i] < st[i-1]]
            fold_up, fold_dn = max(fu), max(fd)
            pu = [C[i][4] for i in range(1, len(C)) if st[i] > st[i-1]]
            pd = [C[i][4] for i in range(1, len(C)) if st[i] < st[i-1]]
            pitch_up, pitch_dn = sum(pu) / len(pu), sum(pd) / len(pd)
            ok = 30 < math.degrees(amp) < 45 and 3.4 < freq < 4.2 and math.degrees(sym) < 1 and fold_up > fold_dn + 0.1 and pitch_dn > 0 > pitch_up
            record(9, 'Wingbeat: amplitude, frequency, symmetry, upstroke flex, pitch reversal', ok,
                   f'cruise stroke ±{math.degrees(amp):.1f}° (median per beat, mean of both wings) at {freq:.2f} Hz over {len(periods)} beats (sim time); '
                   f'L/R difference {math.degrees(sym):.2f}° in wings-level straight flight ({len(level)} samples), up to {math.degrees(asym_turn):.1f}° in turns (roll control); '
                   f'peak fold upstroke {fold_up:.2f} vs downstroke {fold_dn:.2f}; mean arm pitch downstroke {math.degrees(pitch_dn):+.1f}° (pronated) vs upstroke {math.degrees(pitch_up):+.1f}°')
        else:
            record(9, 'Wingbeat kinematics', False, f'only {len(C)} cruise samples')

        # ---------------------------------------------------------------- 10. banking & route
        record(10, 'Flight path: banked turns within limit, glide, return', 20 < f['maxRoll'] <= 34.5 and 'glide' in f['phases'] and 'approach' in f['phases'],
               f'max bank {f["maxRoll"]:.1f}° (limit 34°); phases {f["phases"]}')

        # ---------------------------------------------------------------- 11. FLY while DETACHED assembles first
        pg.keyboard.press('d'); ev("__t.runUntil(() => __t.state() === 'DETACHED', 20)")
        btn('FLY').click()
        s1 = ev('__t.state()'); q1 = ev('window.__owl.director.queued')
        r = ev(r"""(() => { const o = window.__owl, d = o.director; let t = 0, seen = [], progAtTakeoff = -1, blendAtTakeoff = -1
            while (t < 30) { o.advance(1/60); t += 1/60; if (seen[seen.length-1] !== d.state) { seen.push(d.state); if (d.state === 'TAKING_OFF') { progAtTakeoff = d.assembly.progress; blendAtTakeoff = d.blend.w } } if (d.state === 'TAKING_OFF') break }
            return { seen, progAtTakeoff, blendAtTakeoff, t } })()""")
        ok11 = s1 == 'ASSEMBLING' and q1 == 'FLY' and r['seen'][-1] == 'TAKING_OFF' and r['progAtTakeoff'] == 0 and r['blendAtTakeoff'] == 0
        record(11, 'FLY while DETACHED: assembles first, then takes off', ok11,
               f'click->{s1} (queued {q1}); then {r["seen"]} (ASSEMBLED is reached and left within one update); at take-off explode progress {r["progAtTakeoff"]}, fixture blend {r["blendAtTakeoff"]}; {r["t"]:.1f}s sim')
        f2 = ev(flight_js)  # let that flight finish
        c = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
        record(12, 'Second flight completes and restores exactly', f2['states'][-1] == 'ASSEMBLED' and c['dp'] < 1e-6 and math.dist(f2['end'], f2['perch']) < 0.005,
               f'states {f2["states"]}; node error {c["dp"]:.1e} m')

        # ---------------------------------------------------------------- 13. command spam in every state
        import random
        random.seed(7)
        keys = ['d', 'a', 'f']
        visited = set()
        for i in range(120):
            pg.keyboard.press(random.choice(keys))
            visited.add(ev('__t.state()'))
            ev(f'window.__owl.advance({random.choice([0, 0.05, 0.2, 0.6, 1.5, 3.0])})')
        # DETACH during flight is queued: robot lands, then detaches
        ev("__t.runUntil(() => ['ASSEMBLED','DETACHED'].includes(__t.state()) && !window.__owl.director.queued && !window.__owl.director.sim && !window.__owl.director.assembly.busy, 300)")
        pg.keyboard.press('a'); ev("__t.runUntil(() => __t.state() === 'ASSEMBLED' && !window.__owl.director.queued, 60)")
        st = ev('__t.state()')
        c = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
        # explicit queued DETACH during a flight
        btn('FLY').click(); ev('window.__owl.advance(6)')
        mid_state = ev('__t.state()')
        fly_disabled = btn('FLY').is_disabled()
        for _ in range(5):
            pg.keyboard.press('f')
        btn('DETACH').click(); q = ev('window.__owl.director.queued')
        r = ev("""(() => { const o = window.__owl, d = o.director; let t = 0, seen = []
            while (t < 200) { o.advance(1/60); t += 1/60; if (seen[seen.length-1] !== d.state) seen.push(d.state); if (d.state === 'DETACHED') break } return { seen, t } })()""")
        pg.keyboard.press('a'); ev("__t.runUntil(() => __t.state() === 'ASSEMBLED', 30)")
        c2 = ev(f'__t.compare({json.dumps(ref)}, __t.snapshot())')
        record(13, 'Random command spam (120 key presses) and DETACH during flight', st == 'ASSEMBLED' and c['dp'] < 1e-6 and q == 'DETACH' and fly_disabled
               and r['seen'][-1] == 'DETACHED' and c2['dp'] < 1e-6 and not errors(),
               f'states visited {sorted(visited)}; settled {st}, node error {c["dp"]:.1e} m; DETACH in {mid_state} queued={q} -> {r["seen"]}; '
               f'after re-assembly node error {c2["dp"]:.1e} m; console errors {len(errors())}')

        # ---------------------------------------------------------------- 14. hover/click info + head tracking (real frames)
        ev('window.__owl.pause(false)')
        pg.click('button[aria-label="Reset view"]'); pg.wait_for_timeout(3000)
        nd = ev("__t.ndc('P_shell_front')")
        box = pg.locator('canvas').bounding_box()
        hx, hy = box['x'] + (nd[0] + 1) / 2 * box['width'], box['y'] + (1 - nd[1]) / 2 * box['height']
        pg.mouse.move(hx, hy); pg.wait_for_timeout(800)
        title = pg.locator('.info.is-on .info-title').all_inner_texts()
        pg.mouse.click(hx, hy); pg.wait_for_timeout(400)
        pinned = ev('(() => { return document.querySelector(".info.is-on .info-title")?.textContent ?? null })()')
        # head tracking: pointer far left vs far right of the robot
        def head_for(x):
            pg.mouse.move(x, box['y'] + box['height'] * 0.4); pg.wait_for_timeout(2500)
            ev('window.__owl.pause(true)'); ev('window.__owl.advance(4)')
            y = ev('window.__owl.director.neck.y'); ev('window.__owl.pause(false)')
            return y
        yl = head_for(box['x'] + 30)
        yr = head_for(box['x'] + box['width'] - 30)
        record(14, 'Hover/click part info, and idle head tracking follows the pointer', bool(title) and bool(pinned) and abs(yl - yr) > 0.15,
               f'hover -> "{title[0] if title else None}", click pins "{pinned}"; neck yaw pointer-left {math.degrees(yl):+.0f}°, pointer-right {math.degrees(yr):+.0f}°')

        # ---------------------------------------------------------------- 15. performance (software GL — not representative of a GPU)
        fps = ev("""new Promise(res => { let n = 0; const t0 = performance.now(); const f = () => { n++; if (performance.now() - t0 < 5000) requestAnimationFrame(f); else res(n * 1000 / (performance.now() - t0)) }; requestAnimationFrame(f) })""")
        info = ev("""new Promise(res => { const o = window.__owl, ctx = o.gl.getContext(); let calls = 0, tris = 0
            const names = ['drawElements', 'drawArrays', 'drawElementsInstanced', 'drawArraysInstanced', 'drawRangeElements'], orig = {}
            for (const n of names) if (ctx[n]) { orig[n] = ctx[n]; ctx[n] = function (...a) { calls++; return orig[n].apply(this, a) } }
            let k = 0; const f = () => { if (++k < 3) requestAnimationFrame(f); else { for (const n in orig) ctx[n] = orig[n];
              res({ calls: Math.round(calls / 2), geometries: o.gl.info.memory.geometries, textures: o.gl.info.memory.textures }) } }
            requestAnimationFrame(() => { calls = 0; requestAnimationFrame(f) }) })""")
        ev('window.__owl.pause(true)')
        cpu = ev("""(() => { const o = window.__owl, d = o.director; const t0 = performance.now(); o.advance(5); const a = (performance.now()-t0)/300;
            d.command('FLY'); o.advance(8); const t1 = performance.now(); o.advance(5); const b = (performance.now()-t1)/300; return [a, b] })()""")
        ev("__t.runUntil(() => __t.state() === 'ASSEMBLED', 300)")
        heap = ev('performance.memory ? performance.memory.usedJSHeapSize/1048576 : -1')
        perf = (cpu, info, fps, heap)
        _ = None


        # ---------------------------------------------------------------- 16. reload + mobile layout
        logs.clear()
        pg.set_viewport_size({'width': 390, 'height': 844})
        t_reload = boot(BASE)
        st = ev('__t.state()')
        dock = pg.locator('.dock').bounding_box()
        overflow = ev('document.documentElement.scrollWidth > window.innerWidth')
        record(16, 'Reload (phone-size viewport): clean start, controls on screen', st == 'ASSEMBLED' and not errors() and dock and dock['x'] >= 0
               and dock['x'] + dock['width'] <= 390 and not overflow,
               f'reloaded in {t_reload:.1f}s; state {st}; dock at x={dock["x"]:.0f}..{dock["x"]+dock["width"]:.0f} of 390 px; horizontal overflow={overflow}; errors={errors()[:2]}')
        pg.goto('about:blank')  # stop the full-quality page rendering while the next page is measured

        # ---------------------------------------------------------------- 3. camera orbit / zoom / pan / reset (real frames)
        # OrbitControls damping and the framing animations advance per rendered frame, so this test runs on the
        # reduced-quality render path (?low&nopost) where the software renderer manages several frames per second.
        sep = '&' if '?' in BASE else '?'
        pg2 = b.new_page(viewport={'width': 900, 'height': 600})
        pg2.on('pageerror', lambda e: logs.append(('pageerror', str(e))))
        pg2.set_default_timeout(300000)
        pg2.goto(BASE + sep + 'low&nopost')
        pg2.wait_for_function('window.__owl && window.__owl.director', timeout=300000)
        pg2.wait_for_selector('.loader.is-done', state='attached', timeout=300000)
        pg2.evaluate(JS_HELPERS)
        e2 = pg2.evaluate
        fps_low = e2("""new Promise(res => { let n = 0; const t0 = performance.now(); const f = () => { n++; if (performance.now() - t0 < 5000) requestAnimationFrame(f); else res(n * 1000 / (performance.now() - t0)) }; requestAnimationFrame(f) })""")
        e2('__t.settleCamera()')
        cam2 = lambda: e2("(() => { const o = window.__owl, T = o.THREE, c = o.camera, d = new T.Vector3(); c.getWorldDirection(d); return [c.position.x, c.position.y, c.position.z, d.x, d.y, d.z] })()")
        box2 = pg2.locator('canvas').bounding_box()
        cx, cy = box2['x'] + box2['width'] * 0.30, box2['y'] + box2['height'] * 0.5
        c0 = cam2()
        pg2.mouse.move(cx, cy); pg2.mouse.down(); pg2.mouse.move(cx + 160, cy + 20, steps=12); pg2.mouse.up()
        e2('__t.settleCamera()')
        c1 = cam2()
        orbit = math.dist(c0[:3], c1[:3])
        tdist = lambda: e2("window.__owl.camera.position.distanceTo(window.__owl.director.defaultTarget())")
        dist0 = tdist()
        pg2.mouse.move(cx, cy); pg2.mouse.wheel(0, 400); e2('__t.settleCamera()')
        dist1 = tdist()
        p0 = cam2()
        pg2.mouse.move(cx, cy); pg2.mouse.down(button='right'); pg2.mouse.move(cx + 80, cy - 40, steps=10); pg2.mouse.up(button='right')
        e2('__t.settleCamera()')
        p1 = cam2()
        pan = math.dist(p0[:3], p1[:3]); pan_dir_change = math.dist(p0[3:], p1[3:])
        pg2.click('button[aria-label="Reset view"]')
        e2('__t.frames(3)'); e2('__t.settleCamera()')
        c3 = cam2()
        exp = e2("""(() => { const o = window.__owl, T = o.THREE, d = o.director; const t = d.defaultTarget();
            const D = T.MathUtils.degToRad; const dir = new T.Vector3(Math.sin(D(28))*Math.cos(D(8)), Math.sin(D(8)), Math.cos(D(28))*Math.cos(D(8)));
            return t.addScaledVector(dir, 1.3).toArray() })()""")
        reset_err = math.dist(c3[:3], exp)
        record(3, 'Camera orbit, zoom, pan and reset (mouse + reset button)', orbit > 0.05 and dist1 > dist0 + 0.05 and pan > 0.02 and pan_dir_change < 0.02 and reset_err < 0.01,
               f'drag moved camera {orbit:.3f} m; wheel distance {dist0:.2f}->{dist1:.2f} m; right-drag panned {pan:.3f} m '
               f'(view direction change {pan_dir_change:.4f}); reset lands {reset_err*1000:.1f} mm from the default view')
        calls_low = e2("""new Promise(res => { const o = window.__owl, ctx = o.gl.getContext(); let calls = 0
            const names = ['drawElements', 'drawArrays', 'drawElementsInstanced', 'drawArraysInstanced', 'drawRangeElements'], orig = {}
            for (const n of names) if (ctx[n]) { orig[n] = ctx[n]; ctx[n] = function (...a) { calls++; return orig[n].apply(this, a) } }
            let k = 0; const f = () => { if (++k < 3) requestAnimationFrame(f); else { for (const n in orig) ctx[n] = orig[n]; res(Math.round(calls / 2)) } }
            requestAnimationFrame(() => { calls = 0; requestAnimationFrame(f) }) })""")
        pg2.close()
        record(15, 'Performance and resource use', cpu[1] < 4 and info['calls'] < 1500,
               f'software-GL (SwiftShader, CPU-rendered; a GPU is far faster) frame rate {fps:.1f} fps full quality at 1100×700, {fps_low:.1f} fps low quality at 900×600; '
               f'{info["calls"]} WebGL draw calls per frame at full quality (shadow, AO and post passes included; target < 1500), {calls_low} in low quality; {info["geometries"]} geometries, {info["textures"]} textures; '
               f'CPU per simulation update {cpu[0]:.2f} ms perched / {cpu[1]:.2f} ms in flight; JS heap {heap:.0f} MB')

        warns = [l for l in logs if l[0] == 'warning']
        b.close()

    results.sort(key=lambda r: r['id'])
    json.dump({'base': BASE, 'results': results, 'warnings': [w[1][:200] for w in warns]}, open(OUT, 'w'), indent=1)
    npass = sum(r['pass'] for r in results)
    print(f'\n{npass}/{len(results)} passed')


if __name__ == '__main__':
    main()
