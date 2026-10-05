"""Headless WebGL screenshot harness (Chromium + SwiftShader).

python3 tools/shoot.py OUTDIR [--url URL] [--size 1600x1000] [states...]
States: front side rear top below detached detached_side flight_down flight_up flight_glide takeoff landing
"""
import sys, os, json, time, argparse
from playwright.sync_api import sync_playwright

ap = argparse.ArgumentParser()
ap.add_argument('out')
ap.add_argument('states', nargs='*')
ap.add_argument('--url', default='http://localhost:5173/?capture')
ap.add_argument('--size', default='1600x1000')
ap.add_argument('--wait', type=int, default=2600)
a = ap.parse_args()
W, H = map(int, a.size.split('x'))
os.makedirs(a.out, exist_ok=True)
STATES = a.states or ['front', 'side', 'rear', 'detached', 'flight_down', 'flight_up']

logs = []
with sync_playwright() as p:
    b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
    pg = b.new_page(viewport={'width': W, 'height': H}, device_scale_factor=1)
    pg.on('console', lambda m: logs.append(f'{m.type}: {m.text}'))
    pg.on('pageerror', lambda e: logs.append(f'pageerror: {e}'))
    t0 = time.time()
    pg.goto(a.url)
    pg.wait_for_function('window.__owl && window.__owl.director', timeout=240000)
    pg.wait_for_selector('.loader.is-done', state='attached', timeout=240000)
    pg.wait_for_timeout(1500)
    print('ready in', round(time.time() - t0, 1), 's')
    pg.evaluate('window.__owl.pause(true)')

    pg.set_default_timeout(300000)
    def shot(name):
        t1 = time.time()
        pg.evaluate('window.__owl.frame()')
        pg.evaluate('window.__owl.frame()')
        path = os.path.join(a.out, name + '.png')
        pg.screenshot(path=path, timeout=300000)
        print('  frame+shot', round(time.time() - t1, 1), 's')
        print('saved', path)

    def ensure_assembled():
        st = pg.evaluate('window.__owl.director.state')
        if st != 'ASSEMBLED':
            pg.evaluate("window.__owl.director.command('ASSEMBLE')")
            pg.evaluate('window.__owl.advance(8)')

    def fly_to(t):
        ensure_assembled()
        pg.evaluate("window.__owl.director.command('FLY')")
        pg.evaluate(f'window.__owl.advance({t})')

    for s in STATES:
        if s in ('front', 'side', 'rear', 'top', 'below', 'front_close'):
            ensure_assembled()
            v = {'front': (28, 8, 1.25, -0.035), 'side': (90, 6, 1.25, -0.035), 'rear': (200, 14, 1.45, -0.09),
                 'top': (20, 70, 1.3, 0), 'below': (30, -30, 1.1, 0), 'front_close': (16, 4, 0.6, 0.07)}[s]
            pg.evaluate(f'window.__owl.viewOwl({v[0]}, {v[1]}, {v[2]}, {v[3]})')
            pg.evaluate('window.__owl.frame()')        # one rendered frame updates the viewer look target
            pg.evaluate('window.__owl.advance(2.0)')  # let the head settle on the viewer
            shot(s)
        elif s.startswith('detached'):
            if pg.evaluate('window.__owl.director.state') != 'DETACHED':
                ensure_assembled()
                pg.evaluate("window.__owl.director.command('DETACH')")
                pg.evaluate('window.__owl.advance(9)')
            az, el, dist = {'detached': (0, 34, 3.05), 'detached_side': (95, 8, 2.3), 'detached_top': (15, 62, 2.5),
                            'detached_close': (30, 20, 1.1)}[s]
            d = pg.evaluate('(()=>{const b=window.__owl.director.assembly.explodedBox; const c=new window.__owl.THREE.Vector3(); b.getCenter(c); c.add(window.__owl.director.displayPos); return [c.x,c.y,c.z]})()')
            pg.evaluate(f'window.__owl.setView([{d[0]},{d[1]},{d[2]}], {az}, {el}, {dist})')
            shot(s)
        else:
            ensure_assembled()
            key = {'flight_down': 'down', 'flight_up': 'up', 'flight_bank': 'bank', 'flight_glide': 'glide',
                   'takeoff': 'takeoff', 'landing': 'land', 'prep': 'prep'}[s]
            print(s, pg.evaluate(f"window.__owl.flyTo('{key}')"))
            shot(s)
            pg.evaluate("window.__owl.advance(40)")
    info = pg.evaluate('window.__owl.info()')
    print('render info', json.dumps(info))
    b.close()
errs = [l for l in logs if l.startswith(('error', 'pageerror'))]
print('console errors:', len(errs))
for l in errs[:20]:
    print('  ', l[:300])
