"""Wing: quasi-direct-drive root actuator, capstan transmission, shoulder hub, spars, fold linkage,
fairings and feathers (with coverts carried on each feather).

Left wing in Blender space; call builders inside `with side(s)` for both wings.
"""
import math
import numpy as np
from mathutils import Vector as V, Matrix as M
from lib import *

WS = 1.26                             # wing scale vs. the first prototype (span 1.22 m -> ~1.43 m)
CF = 1.06                            # extra chordwise scale on feathers
AXLE = V((0.072, -0.035, 0.020))     # stroke axis (along Y)
SP = V((0.090, -0.036, 0.020))       # sweep hinge / spar root
W = SP + (V((0.272, -0.048, 0.022)) - SP) * WS    # wrist
T = SP + (V((0.404, -0.036, 0.022)) - SP) * WS    # hand spar tip
DRUM_R = 0.030                       # capstan sector drum radius (on the stroke axle)
CAP_R = 0.0065                       # capstan radius -> 4.6:1 second stage
MX = AXLE.x - DRUM_R - CAP_R - 0.0004  # capstan shaft axis x
MZ = -0.015                          # motor axis z (belt first stage, 3:1)
PUL_BIG, PUL_SMALL = 0.0180, 0.0060
DRUM_Y = -0.030
ARM_R = frame_from_x(W - SP)
HAND_R = frame_from_x(T - W)
ARM_LEN = (W - SP).length
HAND_LEN = (T - W).length
ELB_X = 0.072 * WS                    # elbow distance from the spar root
ELB = SP + (W - SP).normalized() * ELB_X
FORE_LEN = ARM_LEN - ELB_X

PRI_F = [0.04, 0.13, 0.22, 0.31, 0.41, 0.51, 0.61, 0.72, 0.84, 0.97]
PRI_A = [86, 79, 71, 62, 52, 41, 31, 22, 14, 8]
# owl hand: shorter, broader primaries (rounded, low-aspect-ratio tip) than a falcon-like wing
# owl hand: graded primaries (P7 longest, short P10) give a broad, ROUNDED tip rather than a pointed one
PRI_L = [x * WS * CF * 0.90 for x in [0.200, 0.205, 0.212, 0.220, 0.226, 0.232, 0.235, 0.230, 0.214, 0.188]]
PRI_WL = [x * WS * 1.15 for x in [0.0120, 0.0118, 0.0112, 0.0108, 0.0100, 0.0090, 0.0080, 0.0072, 0.0066, 0.0060]]
PRI_WT = [x * WS * 1.08 * 1.30 for x in [0.0260, 0.0260, 0.0258, 0.0252, 0.0248, 0.0240, 0.0232, 0.0222, 0.0205, 0.0185]]
SEC_G = [0.03 + 0.092 * j for j in range(11)]   # fraction from wrist toward elbow
SEC_L = [x * WS * CF * 1.25 for x in [0.198, 0.200, 0.201, 0.201, 0.200, 0.199, 0.197, 0.194, 0.190, 0.185, 0.178]]
XZ = M(((1, 0, 0), (0, 0, -1), (0, 1, 0)))       # plate local z -> world -Y


# ------------------------------------------------------------------ feather generator
def feather_geom(L, wl, wt, emarg=0.0, notch=0.0, curve=0.03, camber=0.0016, droop=0.008, comb=False,
                 fringe=True, nl=44, nc=10, tilt=0.0, rachis=True, s0=0.07, tip_start=0.80, tip_point=0.0):
    """Returns dict of (verts, faces, uvs) per material in feather-local space.
    X along rachis, +Y toward broad (trailing) vane, +Z up."""
    def yc(s): return curve * L * s * s
    def zc(s): return -droop * s * s

    def tipf(s):
        if s <= tip_start: return 1.0
        q = (s - tip_start) / (1 - tip_start)
        r = math.sqrt(max(0.0, 1 - q * q))
        return r * (1 - tip_point) + (1 - q) * tip_point

    def rise(s): return smoothstep(s0, s0 + 0.16, s) ** 0.55

    def lead(s): return max(0.0004, wl * rise(s) * (1 - emarg * smoothstep(0.42, 0.60, s)) * tipf(s))
    def trail(s): return max(0.0004, wt * rise(s) * (1 - notch * smoothstep(0.50, 0.64, s)) * tipf(s))

    out = {}
    verts = []; uvs = []
    ss = [s0 + (1 - s0) * (i / (nl - 1)) for i in range(nl)]
    for i, s in enumerate(ss):
        a = lead(s); b = trail(s)
        for j in range(nc):
            c = j / (nc - 1)
            y = -a + c * (a + b)
            half = b if y > 0 else a
            q = min(1.0, abs(y) / max(half, 1e-6))
            z = zc(s) + camber * (a + b) / 0.035 * (1 - q * q)
            if fringe and j == nc - 1 and s > 0.15 and i % 2 == 1:
                y += 0.0010
            if comb and j == 0 and 0.12 < s < 0.86 and i % 2 == 1:
                y -= 0.0013
            verts.append(V((s * L, yc(s) + y, z))); uvs.append((c, s))
    faces = []
    for i in range(nl - 1):
        for j in range(nc - 1):
            a_ = i * nc + j; b_ = (i + 1) * nc + j; c_ = (i + 1) * nc + j + 1; d_ = i * nc + j + 1
            faces.append((a_, b_, c_, d_))
    out['vane'] = (verts, faces, uvs)
    if rachis:
        pts = []; rad = []
        for k in range(30):
            s = -0.05 + 1.0 * k / 29
            s = min(s, 0.975)
            ss_ = max(s, s0)
            zz = zc(s) + camber * (lead(ss_) + trail(ss_)) / 0.035 + 0.0003
            pts.append(V((s * L, yc(max(s, 0)), zz if s > s0 else zc(0) + 0.0002)))
            rad.append((0.0019 * (1 - 0.78 * max(s, 0)) if s > s0 else 0.0022, 0.0014 * (1 - 0.75 * max(s, 0)) if s > s0 else 0.0022))
        out['rachis'] = sweep(pts, rad, nseg=8)
    if comb:
        cv = []; cf = []; cu = []
        for i, s in enumerate(ss):
            if not (0.14 < s < 0.84) or i % 2: continue
            a = lead(s)
            base = V((s * L, yc(s) - a, zc(s)))
            tip = base + V((0.0016, -0.0026, 0.0002))
            k = len(cv)
            cv += [base + V((-0.0007, 0.0002, 0.0003)), base + V((0.0007, 0.0002, 0.0003)), base + V((0, 0.0002, -0.0003)), tip]
            cu += [(0, 0)] * 4
            cf += [(k, k + 1, k + 3), (k + 1, k + 2, k + 3), (k + 2, k, k + 3), (k, k + 2, k + 1)]
        out['comb'] = (cv, cf, cu)
    if abs(tilt) > 1e-9:
        Rt = rot_axis((1, 0, 0), tilt)
        for key in out:
            v, f, u = out[key]
            out[key] = ([Rt @ V(p) for p in v], f, u)
    return out


def feather_obj(name, origin, R, L, wl, wt, mat_vane='feather', thick=0.0008, covert=None, **kw):
    """covert: optional (length, wl, wt, lift) overlapping covert blade carried by the same feather."""
    g = feather_geom(L, wl, wt, **kw)
    objs = []
    v, f, u = g['vane']
    # per-feather variation of the printed barring (phase and pitch), so neighbouring feathers
    # never line up into continuous stripes
    import random
    rr = random.Random(name)
    dv, sv, du = rr.uniform(0.0, 0.2), rr.uniform(0.88, 1.12), rr.uniform(-0.04, 0.04)
    u = [(a + du, b * sv + dv) for a, b in u]
    vane = mesh_obj(name + '_v', [origin + R @ V(p) for p in v], f, uvs=u, mat_names=(mat_vane,))
    solidify(vane, thick, offset=0.0)
    objs.append(vane)
    if 'rachis' in g:
        v, f, u = g['rachis']
        objs.append(mesh_obj(name + '_r', [origin + R @ V(p) for p in v], f, uvs=u, mat_names=('rachis',)))
    if 'comb' in g:
        v, f, u = g['comb']
        o = mesh_obj(name + '_c', [origin + R @ V(p) for p in v], f, uvs=u, mat_names=('ti_brushed',), smooth=False)
        objs.append(o)
    if covert:
        cl, cwl, cwt, lift = covert
        cg = feather_geom(cl, cwl, cwt, curve=0.0, camber=0.0010, droop=0.002, fringe=False, rachis=False,
                          s0=0.0, tip_start=0.55, tip_point=0.6, nl=18, nc=7, tilt=kw.get('tilt', 0.0))
        v, f, u = cg['vane']
        u = [(a, b * sv + dv * 0.7) for a, b in u]
        cv = mesh_obj(name + '_cov', [origin + R @ (V(p) + V((-0.006, 0.0, lift))) for p in v], f, uvs=u, mat_names=('covert',))
        solidify(cv, 0.0008, offset=0.0)
        objs.append(cv)
    o = join(objs, name)
    return finalize(o, origin=origin, R=R)


# ------------------------------------------------------------------ root actuator & transmission
def build_root_motor():
    """Quasi-direct-drive 58 mm pancake outrunner with integrated FOC driver and absolute encoder."""
    c = V((MX, 0.010, MZ))
    Ry = rot_to_z((0, 1, 0))
    p = []
    p.append(cyl_obj('mshaft', V((MX, -0.008, MZ)), V((MX, 0.004, MZ)), 0.0020, m='steel', nseg=14))
    p.append(lathe_obj('mpul', [(0.0020, -0.0025), (PUL_SMALL + 0.0008, -0.0025), (PUL_SMALL, -0.0018), (PUL_SMALL, 0.0018), (PUL_SMALL + 0.0008, 0.0025), (0.0020, 0.0025), (0.0020, -0.0025)],
                       36, V((MX, -0.004, MZ)), rot_to_z((0, -1, 0)), m='alu'))
    p.append(lathe_obj('can', [(0.0030, -0.0080), (0.0273, -0.0080), (0.0290, -0.0066), (0.0290, 0.0060), (0.0275, 0.0072), (0.0030, 0.0072)],
                       72, c, Ry, m='graphite'))
    for k in range(26):
        a = TAU * k / 26
        R2 = Ry @ rot_axis((0, 0, 1), a)
        p.append(rbox(f'slot{k}', (0.0011, 0.0060, 0.0006), c + R2 @ V((0, 0.0188, -0.0082)), R=R2, m='copper', bev=0.0001, seg=1))
    p.append(lathe_obj('stripe', [(0.0291, -0.0010), (0.0293, -0.0010), (0.0293, 0.0012), (0.0291, 0.0012), (0.0291, -0.0010)], 72, c, Ry, m='accent'))
    # integrated driver board + encoder on the back face
    p.append(lathe_obj('drv', [(0, 0.0072), (0.0264, 0.0072), (0.0264, 0.0088), (0, 0.0088)], 48, c, Ry, m='pcb'))
    p.append(rbox('enc', (0.007, 0.0014, 0.007), c + V((0, 0.0095, 0)), m='chip', bev=0.0003, seg=1))
    p.append(rbox('conn', (0.009, 0.004, 0.004), c + V((0.008, 0.0105, -0.008)), m='connector', bev=0.0004, seg=1))
    for k, dz in enumerate((-0.0035, 0.0035)):
        p.append(sweep_obj(f'pw{k}', [c + V((0.010, 0.0110, -0.004 + dz * 0.3)), c + V((0.006, 0.026, -0.010)), V((0.010 + dz, 0.062, -0.030))], 0.0012,
                           m='rubber', nseg=8))
    o = join(p, S('P_rootMotor_{s}'))
    o = finalize(o, origin=c, uv_scale=60)
    for i in range(4):
        a = TAU * i / 4 + 0.785
        e = empty(S(f'FAST_P_rootMotor_{{s}}_{i:02d}'), c + V((0.0205 * math.cos(a), 0.0089, 0.0205 * math.sin(a))), rot_to_z((0, 1, 0)), 0.003)
        link_parent(e.name, o.name)
    return o


def build_capstan():
    """Two-stage drive: 3:1 toothed belt from the motor, then a 4.6:1 capstan cable stage (≈13.8:1 total).
    The capstan shaft runs in its own bearing pair (front and rear girdle bulkheads)."""
    p = []
    cc = V((MX, DRUM_Y, AXLE.z))
    p.append(cyl_obj('cshaft', V((MX, -0.0635, AXLE.z)), V((MX, -0.0005, AXLE.z)), 0.0026, m='steel', nseg=16))
    p.append(lathe_obj('capstan', [(0, -0.0050), (CAP_R + 0.0007, -0.0050), (CAP_R, -0.0043), (CAP_R, 0.0043), (CAP_R + 0.0007, 0.0050), (0, 0.0050)], 36,
                       cc, rot_to_z((0, -1, 0)), m='steel', sharp=35))
    hp = helix(cc + V((0, 0.0036, 0)), rot_to_z((0, -1, 0)), CAP_R + 0.00045, 0.0012, 5.5, 22)
    p.append(sweep_obj('cturns', hp, 0.00038, m='cable', nseg=6))
    p.append(cyl_obj('cbearR', V((MX, -0.0150, AXLE.z)), V((MX, -0.0080, AXLE.z)), 0.0058, m='alu', r_in=0.0026))
    p.append(cyl_obj('cbearF', V((MX, -0.0640, AXLE.z)), V((MX, -0.0565, AXLE.z)), 0.0058, m='alu', r_in=0.0026))
    # big belt pulley behind the rear bulkhead
    pc = V((MX, -0.004, AXLE.z))
    p.append(lathe_obj('bigpul', [(0.0026, -0.0028), (PUL_BIG + 0.0009, -0.0028), (PUL_BIG, -0.0020), (PUL_BIG, 0.0020), (PUL_BIG + 0.0009, 0.0028), (0.0026, 0.0028), (0.0026, -0.0028)],
                       64, pc, rot_to_z((0, -1, 0)), m='alu'))
    for k in range(5):
        a = TAU * k / 5 + 0.3
        p.append(cyl_obj(f'bp_h{k}', pc + V((0.011 * math.cos(a), -0.0030, 0.011 * math.sin(a))), pc + V((0.011 * math.cos(a), 0.0030, 0.011 * math.sin(a))),
                         0.0028, m='alu_dark', nseg=12))
    # toothed belt loop (open-belt tangents between the two pulleys)
    c1 = V((MX, -0.004, AXLE.z)); c2 = V((MX, -0.004, MZ))
    r1, r2 = PUL_BIG + 0.0006, PUL_SMALL + 0.0006
    dz = c1.z - c2.z
    ang = math.asin((r1 - r2) / dz)
    belt = []
    for k in range(25):
        a = -ang + (math.pi + 2 * ang) * k / 24
        belt.append(c1 + V((r1 * math.cos(a), 0, r1 * math.sin(a))))
    for k in range(13):
        a = math.pi + ang + (math.pi - 2 * ang) * k / 12
        belt.append(c2 + V((r2 * math.cos(a), 0, r2 * math.sin(a))))
    belt.append(belt[0])
    p.append(sweep_obj('belt', belt, (0.0007, 0.0024), m='rubber', nseg=8, cap=False, twist_up=(0, 1, 0)))
    # tensioner
    p.append(rbox('tens', (0.004, 0.006, 0.012), cc + V((-0.011, 0.012, -0.004)), m='alu', bev=0.0008, seg=2))
    p.append(cyl_obj('tensr', cc + V((-0.011, 0.009, 0.004)), cc + V((-0.011, 0.015, 0.004)), 0.0020, m='accent', nseg=14))
    o = join(p, S('P_capstan_{s}'))
    return finalize(o, origin=cc, uv_scale=60)


def build_shoulder_hub():
    """Stroke axle, hub and yoke, sweep clevis with its servo, spar pitch bearing."""
    p = []
    p.append(cyl_obj('axle', V((AXLE.x, -0.064, AXLE.z)), V((AXLE.x, -0.010, AXLE.z)), 0.0036, m='steel', nseg=20))
    p.append(cyl_obj('boss', V((AXLE.x, -0.047, AXLE.z)), V((AXLE.x, -0.034, AXLE.z)), 0.0100, m='alu', nseg=40))
    p.append(cyl_obj('bossring', V((AXLE.x, -0.0352, AXLE.z)), V((AXLE.x, -0.0340, AXLE.z)), 0.0104, m='accent', nseg=40))
    yoke_pts = [V((AXLE.x + 0.004, -0.0405, AXLE.z)), V((0.083, -0.0395, AXLE.z)), V((SP.x - 0.006, -0.037, SP.z))]
    p.append(sweep_obj('yoke', yoke_pts, [(0.0090, 0.0060), (0.0082, 0.0056), (0.0078, 0.0054)], m='alu', nseg=24,
                       profile=lambda th: (spow(math.cos(th), 0.6), spow(math.sin(th), 0.6)), twist_up=(0, 1, 0)))
    # sweep clevis (vertical hinge at SP)
    for sz in (1, -1):
        p.append(rbox(f'swlug{sz}', (0.014, 0.016, 0.0026), SP + V((-0.004, 0, 0.0080 * sz)), m='ti_brushed', bev=0.0008, seg=2))
    p.append(cyl_obj('swpin', SP + V((0, 0, -0.0105)), SP + V((0, 0, 0.0105)), 0.0023, m='accent', nseg=16))
    # flexible bellows boot sealing the shell slot around the yoke
    bc = V((AXLE.x + 0.0075, -0.0405, AXLE.z))
    for k in range(4):
        r = 0.0112 - 0.0007 * k
        ring = [bc + V((0.0018 * k, r * math.cos(a) * 0.95, r * math.sin(a))) for a in np.linspace(0, TAU, 33)]
        p.append(sweep_obj(f'boot{k}', ring, 0.0013, m='rubber', nseg=8, cap=False, twist_up=(1, 0, 0)))
    # sweep servo on the yoke underside (stow / landing trim only)
    sc = V((0.081, -0.030, 0.006))
    p.append(rbox('swservo', (0.014, 0.010, 0.009), sc, m='graphite', bev=0.0012, seg=2))
    p.append(sweep_obj('swlink', [sc + V((0.006, -0.004, 0.003)), SP + V((-0.002, 0.006, -0.009))], 0.0007, m='steel', nseg=8))
    o = join(p, S('P_shoulderHub_{s}'))
    return finalize(o, origin=AXLE, uv_scale=60)


def build_sector_drum():
    """Capstan sector drum on the stroke axle with its cable wrap (rotates with the stroke)."""
    p = []
    # asymmetric sector: covers the flapping range (-50..+62 deg) and the -90 deg stow, plus cable terminations
    a0, a1 = math.pi - math.radians(70), math.pi + math.radians(100)
    outer = [(DRUM_R * math.cos(a0 + (a1 - a0) * k / 40), DRUM_R * math.sin(a0 + (a1 - a0) * k / 40)) for k in range(41)]
    hub = [(0.0105 * math.cos(a1 + (a0 - a1 + TAU) * k / 30), 0.0105 * math.sin(a1 + (a0 - a1 + TAU) * k / 30)) for k in range(31)]
    holes = []
    for ang in (-35, 15, 65):
        a = math.pi + math.radians(ang)
        holes.append(circle_pts(0.0048, 22, 0.0195 * math.cos(a), 0.0195 * math.sin(a)))
    cen = V((AXLE.x, DRUM_Y, AXLE.z))
    p.append(plate('drum', outer + hub, holes, thick=0.0034, m='ti_brushed', center=cen, R=XZ, bev=0.0003, seg=1))
    # grooved rim flanges
    for dy in (-0.0029, 0.0029):
        rim = [cen + XZ @ V((DRUM_R * math.cos(a0 + (a1 - a0) * k / 60), DRUM_R * math.sin(a0 + (a1 - a0) * k / 60), 0)) + V((0, dy, 0)) for k in range(61)]
        p.append(sweep_obj(f'flange{dy}', rim, (0.0012, 0.0009), m='alu', nseg=8))
    cab = [cen + XZ @ V(((DRUM_R + 0.0004) * math.cos(a0 + 0.08 + (a1 - a0 - 0.16) * k / 60), (DRUM_R + 0.0004) * math.sin(a0 + 0.08 + (a1 - a0 - 0.16) * k / 60), 0)) for k in range(61)]
    p.append(sweep_obj('cable', cab, 0.00055, m='cable', nseg=6))
    for a in (a0 + 0.06, a1 - 0.06):
        p.append(cyl_obj(f'term{a:.2f}', cen + XZ @ V((0.026 * math.cos(a), 0.026 * math.sin(a), -0.0024)),
                         cen + XZ @ V((0.026 * math.cos(a), 0.026 * math.sin(a), 0.0024)), 0.0016, m='accent', nseg=12))
    p.append(cyl_obj('dclamp', V((AXLE.x, DRUM_Y - 0.004, AXLE.z)), V((AXLE.x, DRUM_Y + 0.004, AXLE.z)), 0.0072, m='alu', nseg=32))
    o = join(p, S('P_sectorDrum_{s}'))
    return finalize(o, origin=cen, uv_scale=60)


def build_torsion_spring():
    """Parallel torsion spring with a solenoid dog-clutch so it can be declutched for stowing."""
    p = []
    Ry = rot_to_z((0, -1, 0))
    # filament-wound GFRP torsion tube (+/-45 deg plies), titanium end fittings
    p.append(cyl_obj('ttube', V((AXLE.x, -0.0555, AXLE.z)), V((AXLE.x, -0.0445, AXLE.z)), 0.0086, m='ceramic_mid', nseg=40, r_in=0.0062))
    for yy in (-0.0560, -0.0450):
        p.append(cyl_obj(f'tfit{yy}', V((AXLE.x, yy, AXLE.z)), V((AXLE.x, yy + 0.0012, AXLE.z)), 0.0092, m='ti_brushed', nseg=40, r_in=0.0040))
    p.append(cyl_obj('dog', V((AXLE.x, -0.0592, AXLE.z)), V((AXLE.x, -0.0565, AXLE.z)), 0.0078, m='alu_dark', nseg=32))
    p.append(cyl_obj('sol', V((AXLE.x - 0.0125, -0.060, AXLE.z - 0.004)), V((AXLE.x - 0.0125, -0.050, AXLE.z - 0.004)), 0.0038, m='graphite', nseg=20))
    p.append(cyl_obj('solr', V((AXLE.x - 0.0125, -0.0605, AXLE.z - 0.004)), V((AXLE.x - 0.0125, -0.0598, AXLE.z - 0.004)), 0.0039, m='accent', nseg=20))
    o = join(p, S('P_torsionSpring_{s}'))
    return finalize(o, origin=V((AXLE.x, -0.054, AXLE.z)), uv_scale=60)


def build_pitch_servo():
    p = []
    sc = V((0.082, -0.044, 0.032))
    p.append(rbox('pservo', (0.016, 0.010, 0.010), sc, m='graphite', bev=0.0012, seg=2))
    p.append(rbox('pservo_lbl', (0.010, 0.0004, 0.006), sc + V((0, -0.0052, 0)), m='ti_brushed', bev=0.0001, seg=1))
    p.append(cyl_obj('phorn', sc + V((0.004, 0.003, 0.005)), sc + V((0.004, 0.003, 0.0075)), 0.0028, m='accent', nseg=16))
    p.append(sweep_obj('plink', [sc + V((0.006, 0.004, 0.0068)), SP + V((0.004, 0.004, 0.0105))], 0.0007, m='steel', nseg=8))
    o = join(p, S('P_pitchServo_{s}'))
    return finalize(o, origin=sc, uv_scale=60)


# ------------------------------------------------------------------ fairings
def fairing_section(n, t, b, rt, rb, npt_top=12, npt_nose=14, npt_bot=8):
    """Cambered D-nose: blunt rounded leading edge (owl-like), thick upper surface carried back over the
    feather roots as the covert layer, flatter lower surface."""
    pts = []
    for k in range(npt_top):
        y = rt * (1 - k / npt_top)
        q = y / rt
        pts.append((y, 0.0012 + (t - 0.0012) * (1 - q ** 1.8) ** 0.8))
    for k in range(npt_nose + 1):
        ph = (math.pi / 2) * k / npt_nose
        # super-elliptic upper nose: fuller, rounder leading edge
        pts.append((-n * spow(math.sin(ph), 0.85), t * spow(math.cos(ph), 0.7)))
    for k in range(1, npt_nose + 1):
        ps = (math.pi / 2) * k / npt_nose
        pts.append((-n * spow(math.cos(ps), 0.85), -b * spow(math.sin(ps), 0.6)))
    for k in range(1, npt_bot + 1):
        y = rb * k / npt_bot
        q = y / rb
        pts.append((y, -0.0008 - (b - 0.0008) * (1 - q ** 1.7) ** 0.75))
    return pts


def loft(sections, closed=True, cap=True):
    """sections: list of lists of 3D points (same count)."""
    n = len(sections); m = len(sections[0])
    verts = [p for sec in sections for p in sec]
    uvs = [(j / m, i / (n - 1)) for i in range(n) for j in range(m)]
    faces = []
    for i in range(n - 1):
        for j in range(m if closed else m - 1):
            a = i * m + j; b = i * m + (j + 1) % m; c = (i + 1) * m + (j + 1) % m; d = (i + 1) * m + j
            faces.append((a, d, c, b))
    if cap:
        faces.append(tuple(range(m))[::-1])
        faces.append(tuple(range((n - 1) * m, n * m)))
    return verts, faces, uvs


def arm_fairing(name, x0, x1, origin, N=24):
    secs = []
    for i in range(N):
        s_ = i / (N - 1)
        x = x0 + (x1 - x0) * s_
        q = x / ARM_LEN
        n = 0.046 - 0.012 * q; t = 0.0175 - 0.0045 * q; b = 0.0105 - 0.0030 * q
        rt = 0.046 - 0.010 * q; rb = 0.0135 - 0.003 * q
        e = min(1.0, min(s_, 1 - s_) / 0.06) if (s_ < 0.06 or s_ > 0.94) else 1.0
        e = 0.84 + 0.16 * math.sqrt(max(0, e))
        sec = fairing_section(n * e, t * e, b * e, rt * e, rb * e)
        secs.append([SP + ARM_R @ V((x, y, z)) for (y, z) in sec])
    v, f, uv = loft(secs)
    o = mesh_obj(name + '_skin', v, f, uvs=[(a * 1.6, b * (x1 - x0) / 0.06) for a, b in uv], mat_names=('shell_marked',))
    recalc_outward(o)
    o.data.set_sharp_from_angle(angle=math.radians(50))
    parts = [o] + median_coverts(name, lambda x: SP + ARM_R @ V((x, 0, 0)), ARM_R, x0 + 0.006, x1 - 0.004,
                                  lambda q: 0.0175 - 0.0045 * q, lambda q: 0.046 - 0.010 * q, ARM_LEN)
    o = join(parts, name)
    return finalize(o, origin=origin, R=ARM_R)


def median_coverts(name, base_fn, Rf, xa, xb, t_of, rt_of, Lref, pitch=0.0165):
    """Row of median coverts lying on the fairing's upper surface and over the greater coverts."""
    out = []
    n = max(2, int((xb - xa) / pitch))
    for i in range(n):
        x = xa + (xb - xa) * (i + 0.5) / n
        q = x / Lref
        t = t_of(q); rt = rt_of(q)
        y0 = 0.30 * rt
        z0 = 0.0012 + (t - 0.0012) * (1 - 0.30 ** 1.8) ** 0.8 + 0.0008
        org = base_fn(x) + Rf @ V((0, y0, z0))
        d = Rf @ V((0.30, 1.0, -0.16))
        L = 0.034 + 0.004 * math.sin(i * 1.7)
        out.append(feather_obj(f'{name}_mc{i:02d}', org, frame_from_x(d.normalized()), L, 0.0090, 0.0110, mat_vane='shell_marked', thick=0.0008,
                               curve=0.0, camber=0.0022, droop=0.0035, fringe=False, rachis=False, s0=0.0, tip_start=0.50, tip_point=0.5,
                               nl=10, nc=6))
    return out


def build_hum_fairing():
    return arm_fairing(S('P_humFairing_{s}'), 0.012, ELB_X - 0.0095, SP, N=14)


def build_le_fairing():
    return arm_fairing(S('P_leFairing_{s}'), ELB_X + 0.0095, ARM_LEN - 0.011, ELB, N=26)


def build_hand_fairing():
    secs = []
    N = 26
    for i in range(N):
        s = i / (N - 1)
        x = 0.009 + (HAND_LEN - 0.002) * s
        n = 0.034 - 0.020 * s; t = 0.0122 - 0.0060 * s; b = 0.0074 - 0.0036 * s
        rt = 0.034 - 0.018 * s; rb = 0.0090 - 0.004 * s
        e = 1.0
        if s > 0.82:
            e = math.sqrt(max(0.02, 1 - ((s - 0.82) / 0.18) ** 2))
        if s < 0.04:
            e = 0.85 + 0.15 * s / 0.04
        sec = fairing_section(n * e, t * e, b * e, rt * e, rb * e)
        secs.append([W + HAND_R @ V((x, y, z)) for (y, z) in sec])
    v, f, uv = loft(secs)
    o = mesh_obj(S('P_handFairing_{s}') + '_skin', v, f, uvs=[(a * 1.2, b * HAND_LEN / 0.06) for a, b in uv], mat_names=('shell_marked',))
    recalc_outward(o)
    o.data.set_sharp_from_angle(angle=math.radians(50))
    o = join([o] + median_coverts(S('P_handFairing_{s}'), lambda x: W + HAND_R @ V((x, 0, 0)), HAND_R, 0.014, HAND_LEN * 0.80,
                                  lambda q: 0.0122 - 0.0060 * q, lambda q: 0.034 - 0.018 * q, HAND_LEN, pitch=0.0175),
             S('P_handFairing_{s}'))
    return finalize(o, origin=W, R=HAND_R)


# ------------------------------------------------------------------ spars & linkage
def socket_geom(prefix, origin, d, r=0.0026, L=0.008, m='alu_dark'):
    d = V(d).normalized()
    p0 = origin - d * 0.0035
    return cyl_obj(prefix, p0, p0 + d * L, r, m=m, nseg=14, r_in=0.0)


def sec_frame(j):
    g = SEC_G[j]
    base = W + (ELB - W) * g
    origin = base + ARM_R @ V((0, 0.0058, -0.0015))
    a = math.radians(82 + j * 1.4)
    d = V((math.cos(a), math.sin(a), -0.02))
    return origin, frame_from_x(d), d


def pri_frame(k):
    base = W + (T - W) * PRI_F[k]
    origin = base + HAND_R @ V((0, 0.0045, -0.0010))
    a = math.radians(PRI_A[k])
    d = V((math.cos(a), math.sin(a), -0.015))
    return origin, frame_from_x(d), d


def clevis(p, prefix, center, R, along=-0.004):
    for sz in (1, -1):
        lug = center + R @ V((along, 0.0, 0.0076 * sz))
        p.append(rbox(f'{prefix}lug{sz}', (0.016, 0.014, 0.0026), lug, R=R, m='ti_brushed', bev=0.0008, seg=2))
    p.append(cyl_obj(f'{prefix}pin', center + V((0, 0, -0.0102)), center + V((0, 0, 0.0102)), 0.0022, m='accent', nseg=16))
    # over-centre lock pawl (positive lock in the extended position)
    p.append(rbox(f'{prefix}lock', (0.006, 0.004, 0.003), center + R @ V((-0.010, 0.006, 0.0062)), R=R, m='alu_dark', bev=0.0005, seg=1))


def build_humerus():
    p = []
    d = (W - SP).normalized()
    # pitch bearing housing at the root (rotates with sweep, carries the spar pitch)
    p.append(cyl_obj('pbear', SP + d * 0.002, SP + d * 0.012, 0.0082, m='ti_brushed', nseg=40))
    p.append(cyl_obj('pbear2', SP + d * 0.0055, SP + d * 0.0072, 0.0088, m='alu_dark', nseg=40))
    p.append(cyl_obj('htube', SP + d * 0.010, ELB - d * 0.008, 0.0062, m='carbon', nseg=32))
    p.append(cyl_obj('rootclamp', SP + d * 0.012, SP + d * 0.026, 0.0074, m='ti_brushed', nseg=32))
    p.append(rbox('rootclampslot', (0.014, 0.0012, 0.0030), SP + d * 0.019 + V((0, 0, 0.0072)), R=ARM_R, m='alu_dark', bev=0.0002, seg=1))
    p.append(cyl_obj('esleeve', ELB - d * 0.024, ELB - d * 0.008, 0.0074, m='ti_brushed', nseg=32))
    clevis(p, 'e', ELB, ARM_R)
    # preloaded return spring + hard extension stop at the elbow (passive upstroke flex)
    hp = helix(ELB + V((0, 0, 0.0090)), M.Identity(3), 0.0040, 0.0010, 2.5, 18)
    p.append(sweep_obj('espring', hp, 0.0006, m='steel', nseg=6))
    p.append(rbox('estop', (0.005, 0.004, 0.004), ELB + ARM_R @ V((-0.009, -0.007, 0.0)), R=ARM_R, m='rubber', bev=0.0008, seg=1))
    o = join(p, S('P_humerus_{s}'))
    o = finalize(o, origin=SP, R=ARM_R, uv_scale=80)
    for i, x in enumerate((0.015, 0.023, ELB_X - 0.021, ELB_X - 0.013)):
        e = empty(S(f'FAST_P_humerus_{{s}}_{i:02d}'), SP + ARM_R @ V((x, 0, 0.0075)), ARM_R @ rot_to_z((0, 0, 1)), 0.003)
        link_parent(e.name, o.name)
    return o


def build_spar_primary():
    """Forearm spar: elbow journal + bellcrank, carbon tube, wrist clevis, secondary sockets."""
    p = []
    d = (W - SP).normalized()
    p.append(lathe_obj('ejournal', [(0.0022, -0.0048), (0.0072, -0.0048), (0.0078, -0.0042), (0.0078, 0.0042), (0.0072, 0.0048), (0.0022, 0.0048), (0.0022, -0.0048)],
                       48, ELB, m='alu'))
    horn = [(0.0040, 0.0), (0.0035, -0.003), (0.0018, -0.0095), (0.0, -0.0115), (-0.0018, -0.0095), (-0.0035, -0.003),
            (-0.0040, 0.0), (-0.004, 0.004), (0.004, 0.004)]
    p.append(plate('ehorn', horn, [circle_pts(0.0011, 12, 0, -0.0085)], thick=0.0022, m='ti_brushed',
                   center=ELB + ARM_R @ V((0, 0, -0.0054)), R=ARM_R, bev=0.0003))
    p.append(cyl_obj('ftube', ELB + d * 0.007, W - d * 0.006, 0.0056, m='carbon', nseg=32))
    p.append(cyl_obj('fsleeve', ELB + d * 0.006, ELB + d * 0.019, 0.0070, m='ti_brushed', nseg=32))
    p.append(cyl_obj('wsleeve', W - d * 0.025, W - d * 0.009, 0.0070, m='ti_brushed', nseg=32))
    clevis(p, 'w', W, ARM_R)
    # compliant feather-root sockets (passive feathering against a stop)
    for j in range(11):
        o, R, dd = sec_frame(j)
        p.append(socket_geom(f'ss{j}', o, dd))
    o = join(p, S('P_sparPrimary_{s}'))
    o = finalize(o, origin=ELB, R=ARM_R, uv_scale=80)
    for i, x in enumerate((ARM_LEN - 0.021, ARM_LEN - 0.014)):
        e = empty(S(f'FAST_P_sparPrimary_{{s}}_{i:02d}'), SP + ARM_R @ V((x, 0, 0.0073)), ARM_R @ rot_to_z((0, 0, 1)), 0.003)
        link_parent(e.name, o.name)
    return o


FOLD_OFF = V((0, -0.0085, -0.0004))


def build_fold_actuator():
    p = []
    a = SP + ARM_R @ (V((0.014, 0, 0)) + FOLD_OFF)
    b = SP + ARM_R @ (V((0.046, 0, 0)) + FOLD_OFF)
    p.append(cyl_obj('fbody', a, b, 0.0042, m='graphite', nseg=24))
    p.append(cyl_obj('fcap0', a - (b - a).normalized() * 0.0015, a + (b - a).normalized() * 0.002, 0.0045, m='alu', nseg=24))
    p.append(cyl_obj('fcap1', b - (b - a).normalized() * 0.002, b + (b - a).normalized() * 0.0015, 0.0045, m='alu', nseg=24))
    p.append(sweep_obj('fclamp', [a + V((0, 0.0040, 0.0)), SP + ARM_R @ V((0.020, 0, 0))], (0.0012, 0.003), m='ti_brushed', nseg=8))
    pin = SP + ARM_R @ (V((ELB_X, 0, 0)) + FOLD_OFF)
    p.append(cyl_obj('frod', b, pin - (pin - b).normalized() * 0.003, 0.0014, m='steel', nseg=12))
    p.append(lathe_obj('feye', [(0.0012, -0.0014), (0.0030, -0.0014), (0.0030, 0.0014), (0.0012, 0.0014), (0.0012, -0.0014)], 24, pin, m='ti_brushed'))
    o = join(p, S('P_foldActuator_{s}'))
    return finalize(o, origin=SP, R=ARM_R)


FOLD_ROD_OFF = V((0, -0.0085, 0.0040))


def build_fold_rod():
    p = []
    a = SP + ARM_R @ (V((ELB_X + 0.016, 0, 0)) + FOLD_ROD_OFF)
    b = SP + ARM_R @ (V((ARM_LEN - 0.0032, 0, 0)) + FOLD_ROD_OFF)
    p.append(cyl_obj('rod', a, b, 0.0013, m='steel', nseg=12))
    for nm, c in (('eye0', a - ARM_R @ V((0.0032, 0, 0))), ('eye', b + ARM_R @ V((0.0032, 0, 0)))):
        p.append(lathe_obj(nm, [(0.0012, -0.0014), (0.0030, -0.0014), (0.0030, 0.0014), (0.0012, 0.0014), (0.0012, -0.0014)], 24, c, m='ti_brushed'))
    o = join(p, S('P_foldRod_{s}'))
    return finalize(o, origin=a, R=ARM_R)


def build_wrist_joint():
    """Wrist bellcrank, fold bearing journal and the hand-twist tendon pulley (works at any fold angle)."""
    p = []
    p.append(lathe_obj('journal', [(0.0022, -0.0048), (0.0078, -0.0048), (0.0084, -0.0042), (0.0084, 0.0042), (0.0078, 0.0048), (0.0022, 0.0048), (0.0022, -0.0048)],
                       48, W, m='alu'))
    horn = [(0.0040, 0.0), (0.0035, -0.003), (0.0018, -0.0095), (0.0, -0.0115), (-0.0018, -0.0095), (-0.0035, -0.003),
            (-0.0040, 0.0), (-0.004, 0.004), (0.004, 0.004)]
    p.append(plate('horn', horn, [circle_pts(0.0011, 12, 0, -0.0085)], thick=0.0022, m='ti_brushed',
                   center=W + ARM_R @ V((0, 0, 0.0052)), R=ARM_R, bev=0.0003))
    p.append(cyl_obj('hpin', W + ARM_R @ V((0, -0.0085, -0.0012)), W + ARM_R @ V((0, -0.0085, 0.0064)), 0.0010, m='steel', nseg=10))
    # tendon pulley pair for active hand twist
    p.append(lathe_obj('pulley', [(0.0022, -0.0022), (0.0062, -0.0022), (0.0054, 0.0), (0.0062, 0.0022), (0.0022, 0.0022), (0.0022, -0.0022)], 40,
                       W + V((0, 0, -0.0072)), m='steel', sharp=35))
    o = join(p, S('P_wristJoint_{s}'))
    return finalize(o, origin=W, R=ARM_R)


def build_carpal_cover():
    """Rounded wrist pad (carpal covert analogue). Rotationally symmetric about the wrist fold axis,
    so it hides the wrist clevis, journal and tendon pulley at every fold angle."""
    R0, ZT, ZB, RE = 0.0195, 0.0098, -0.0070, 0.0060
    prof = [(0.0, ZB + 0.0010), (R0 - 0.010, ZB)]
    for k in range(13):
        a = -math.pi / 2 + math.pi * k / 12
        prof.append((R0 - RE + RE * math.cos(a), 0.5 * (ZT + ZB) + (0.5 * (ZT - ZB)) * math.sin(a)))
    prof += [(R0 - 0.010, ZT), (0.0, ZT + 0.0012)]
    p = [lathe_obj('cpad', prof, 72, W + V((0, 0, 0.0005)), M.Identity(3), m='shell_marked', sharp=0)]
    p += pad_hardware('c', W, R0, ZT, ZB)
    o = join(p, S('P_carpalCover_{s}'))
    o.data.set_sharp_from_angle(angle=math.radians(80))
    return finalize(o, origin=W, R=ARM_R)


def pad_hardware(pre, c, R0, ZT, ZB):
    """Joint-pad detailing: titanium bearing band at the split line, hex pivot cap, amber health pip."""
    zm = 0.5 * (ZT + ZB) + 0.0005
    out = [lathe_obj(pre + 'band', [(R0 - 0.0004, zm - 0.0013), (R0 + 0.0005, zm - 0.0010), (R0 + 0.0005, zm + 0.0010),
                                     (R0 - 0.0004, zm + 0.0013), (R0 - 0.0004, zm - 0.0013)], 72, c, M.Identity(3), m='ti_brushed')]
    out.append(lathe_obj(pre + 'hex', [(0, 0), (0.0046, 0), (0.0046, 0.0016), (0.0040, 0.0022), (0, 0.0022)], 6,
                         c + V((0, 0, ZT + 0.0005)), M.Identity(3), m='alu', sharp=30))
    out.append(lathe_obj(pre + 'pip', [(0, 0), (0.0014, 0), (0.0014, 0.0006), (0, 0.0009)], 20, c + V((0.0095, 0.0, ZT - 0.0001)),
                         M.Identity(3), m='amber'))
    return out


def build_elbow_cover():
    """Rounded elbow pad, symmetric about the elbow fold axis (hides clevis, return spring and stop)."""
    R0, ZT, ZB, RE = 0.0175, 0.0100, -0.0072, 0.0056
    prof = [(0.0, ZB + 0.0010), (R0 - 0.009, ZB)]
    for k in range(13):
        a = -math.pi / 2 + math.pi * k / 12
        prof.append((R0 - RE + RE * math.cos(a), 0.5 * (ZT + ZB) + (0.5 * (ZT - ZB)) * math.sin(a)))
    prof += [(R0 - 0.009, ZT), (0.0, ZT + 0.0012)]
    p = [lathe_obj('epad', prof, 72, ELB + V((0, 0, 0.0005)), M.Identity(3), m='shell_marked', sharp=0)]
    p += pad_hardware('e', ELB, R0, ZT, ZB)
    o = join(p, S('P_elbowCover_{s}'))
    o.data.set_sharp_from_angle(angle=math.radians(80))
    return finalize(o, origin=ELB, R=ARM_R)


def build_spar_secondary():
    p = []
    d = (T - W).normalized()
    p.append(cyl_obj('htube', W + d * 0.012, T, 0.0045, m='carbon', nseg=28))
    p.append(cyl_obj('hsleeve', W + d * 0.010, W + d * 0.025, 0.0059, m='ti_brushed', nseg=28))
    p.append(lathe_obj('tipcap', [(0, -0.002), (0.0049, -0.002), (0.0049, 0.004), (0.0032, 0.007), (0, 0.008)], 24, T, rot_to_z(d), m='ti_brushed'))
    for k in range(10):
        o, R, dd = pri_frame(k)
        p.append(socket_geom(f'ps{k}', o, dd, r=0.0024, L=0.007))
    o = join(p, S('P_sparSecondary_{s}'))
    o = finalize(o, origin=W, R=HAND_R, uv_scale=80)
    for i, x in enumerate((0.015, 0.021)):
        e = empty(S(f'FAST_P_sparSecondary_{{s}}_{i:02d}'), W + HAND_R @ V((x, 0, 0.0060)), HAND_R, 0.003)
        link_parent(e.name, o.name)
    return o


# ------------------------------------------------------------------ feathers
def build_secondaries():
    out = []
    n = len(SEC_G)
    for j in range(n):
        o, R, d = sec_frame(j)
        o = o + V((0, 0, 0.0009 * (n - j)))   # outer feathers stack on top
        f = feather_obj(S(f'P_sec_{{s}}_{j + 1:02d}'), o, R, SEC_L[j], 0.0125 * WS, 0.0285 * WS, curve=0.018, camber=0.0014,
                        droop=0.006, tilt=math.radians(-3), fringe=True,
                        covert=(0.084 * WS, 0.0100 * WS, 0.0150 * WS, 0.0042))
        out.append(f)
    return out


def build_primaries():
    out = []
    for k in range(10):
        o, R, d = pri_frame(k)
        o = o + V((0, 0, 0.0010 * k + 0.0012))
        outer = k >= 6
        f = feather_obj(S(f'P_pri_{{s}}_{k + 1:02d}'), o, R, PRI_L[k], PRI_WL[k], PRI_WT[k],
                        emarg=0.22 if k >= 8 else 0.0, notch=(0.18 + 0.04 * (k - 7)) if k >= 7 else 0.0,
                        curve=0.04 + 0.01 * (k / 9), camber=0.0016, droop=0.010, comb=(k == 9),
                        tilt=math.radians(-3.5), fringe=True, tip_start=0.74 if outer else 0.80,
                        covert=((0.066 - 0.0015 * k) * WS, 0.0080 * WS, 0.0110 * WS, 0.0040))
        out.append(f)
    return out


def build_membrane():
    """Dark elastomer root membrane closing the gap between the tertials and the flank."""
    def fn(s, t):
        x = -0.016 + 0.064 * s
        y = 0.004 + 0.125 * WS * t * (1 - 0.45 * s)
        z = -0.0016 - 0.004 * t * t + 0.0012 * math.sin(math.pi * s)
        return SP + ARM_R @ V((x, y, z))
    v, f, uv = grid_surface(fn, 14, 16)
    o = mesh_obj(S('P_membrane_{s}'), v, f, uvs=uv, mat_names=('membrane',))
    if o.data.polygons[0].normal.z < 0:
        import bmesh
        bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(o.data); bm.free()
    solidify(o, 0.0008, offset=-1)
    bevel(o, 0.0003, seg=1, angle=35, harden=False)
    return finalize(o, origin=SP, R=ARM_R)


def build_alula():
    piv = W + HAND_R @ V((0.004, -0.011, 0.0048))
    objs = []
    for k in range(3):
        d = HAND_R @ V((math.cos(math.radians(-12 + 6 * k)), math.sin(math.radians(-12 + 6 * k)), 0.0))
        R = frame_from_x(d)
        o = piv + V((0, 0, 0.0008 * k))
        g = feather_geom((0.050 - 0.009 * k) * WS, 0.0045, 0.0085, curve=0.03, camber=0.0010, droop=0.002, fringe=False, rachis=True,
                         s0=0.06, tip_start=0.7, nl=20, nc=7)
        v, f, u = g['vane']
        ob = mesh_obj(f'al{k}', [o + R @ V(p) for p in v], f, uvs=u, mat_names=('feather',))
        solidify(ob, 0.0008, offset=0)
        objs.append(ob)
        v, f, u = g['rachis']
        objs.append(mesh_obj(f'alr{k}', [o + R @ V(p) for p in v], f, uvs=u, mat_names=('ti_brushed',)))
    objs.append(cyl_obj('alpiv', piv + V((0, 0, -0.002)), piv + V((0, 0, 0.004)), 0.0026, m='accent', nseg=16))
    o = join(objs, S('P_alula_{s}'))
    return finalize(o, origin=piv, R=HAND_R), piv


TER_X = [0.036, 0.052, 0.068]


def ter_frame(k):
    origin = SP + ARM_R @ V((TER_X[k], 0.0064, -0.0012))
    a = math.radians(98 - 3 * k)
    d = ARM_R @ V((math.cos(a), math.sin(a), -0.02))
    return origin, frame_from_x(d), d


def build_tertials():
    out = []
    for k in range(3):
        o, R, d = ter_frame(k)
        o = o + V((0, 0, 0.0016 + 0.0008 * k))
        f = feather_obj(S(f'P_ter_{{s}}_{k + 1:02d}'), o, R, (0.172 + 0.009 * k) * WS, 0.0120 * WS, 0.0255 * WS, curve=0.01, camber=0.0014,
                        droop=0.005, tilt=math.radians(-3), fringe=True)
        out.append(f)
    return out
