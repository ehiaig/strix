"""Fuselage: parametric body surface, shell panels, central frame, battery, avionics."""
import math
import numpy as np
from mathutils import Vector as V, Matrix as M
from lib import *

# --------------------------------------------------------------------- body surface
BODY_Y0, BODY_L = -0.125, 0.277
_U = [0, 0.08, 0.20, 0.35, 0.50, 0.65, 0.80, 0.92, 1.0]
_top = pchip(_U, [0.044, 0.060, 0.064, 0.061, 0.056, 0.049, 0.039, 0.029, 0.022])
_bot = pchip(_U, [0.016, -0.024, -0.064, -0.084, -0.087, -0.077, -0.056, -0.031, -0.012])
_hw = pchip(_U, [0.020, 0.054, 0.075, 0.083, 0.082, 0.076, 0.064, 0.050, 0.034])


def shoulder_bulge(u, vdeg):
    # sculpted shoulder cowl that houses the stroke axle, spring and drum
    vv = ((vdeg + 180) % 360) - 180
    if abs(vv) > 90:
        vv = 180 - vv if vv > 0 else -180 - vv  # mirror right side onto left range
    return 0.0060 * bump(u, 0.22, 0.40, 0.11) * bump(vv, 10, 36, 30)
SE = 2.4  # superellipse exponent


def body_pt(u, vdeg, off=0.0):
    v = math.radians(vdeg)
    top = float(_top(u)); bot = float(_bot(u)); hw = float(_hw(u))
    zc = 0.5 * (top + bot); hh = 0.5 * (top - bot)
    off = off + shoulder_bulge(u, vdeg)
    c, s = math.cos(v), math.sin(v)
    x = (hw + off) * spow(c, 2 / SE)
    z = zc + (hh + off) * spow(s, 2 / SE)
    return V((x, BODY_Y0 + BODY_L * u, z))


def body_normal(u, vdeg):
    e = 1e-3
    p = body_pt(u, vdeg)
    du = body_pt(min(1, u + e), vdeg) - body_pt(max(0, u - e), vdeg)
    dv = body_pt(u, vdeg + 0.2) - body_pt(u, vdeg - 0.2)
    n = du.cross(dv)
    return n.normalized() if n.length > 1e-12 else V((math.cos(math.radians(vdeg)), 0, math.sin(math.radians(vdeg))))


def leg_slot(p):
    """Belly openings the legs swing through (tucked back in flight, forward for landing)."""
    return abs(abs(p.x) - 0.0305) < 0.0108 and -0.014 < p.y < 0.036 and p.z < -0.035


def body_panel(name, u0, u1, vlo, vhi, nu=72, nv=40, off=0.0, thick=0.0018, m='ceramic', u1_fn=None, u0_fn=None, uvp=None, T=0.06,
               hole=None):
    """Panel over (u,v). vlo/vhi: callables of u (deg). u1_fn(t): optional scalloped rear edge.
    uvp: None = parametric UVs; 'xy' = planar from below (breast); 'zy' = planar from the side (flanks),
    both at T metres per texture tile with V running along the body."""
    def fn(s, t):
        a = u0_fn(t) if u0_fn else u0
        b = u1_fn(t) if u1_fn else u1
        u = a + (b - a) * s
        v = vlo(u) + (vhi(u) - vlo(u)) * t
        return body_pt(u, v, off)
    verts, faces, uvs = grid_surface(fn, nu, nv)
    if hole is not None:
        faces = [f for f in faces if not hole(sum((verts[i] for i in f), V((0, 0, 0))) / len(f))]
    if uvp == 'xy':
        uvs = [(p.x / T, -p.y / T) for p in verts]
    elif uvp == 'zy':
        uvs = [(p.z / T, -p.y / T) for p in verts]
    else:
        uvs = [(a * 3, b * 3) for a, b in uvs]
    o = mesh_obj(name, verts, faces, uvs=uvs, mat_names=(m,))
    weld(o, 1e-7)
    solidify(o, thick, offset=-1)
    bevel(o, 0.0006, seg=2, angle=35, harden=False)
    return o


BAY_LO = lambda u: 7.0 + 40.0 * bump(u, 0.29, 0.355, 0.018)


def screw_at(name, parent, p, n, i):
    """Fastener empty at point p with local Z along n."""
    nm = f'FAST_{parent}_{i:02d}'
    R = rot_to_z(n)
    e = empty(nm, p, R, 0.003)
    link_parent(nm, parent)
    return e


def shell_screws(parent, uv_list, off=0.0):
    for i, (u, v) in enumerate(uv_list):
        p = body_pt(u, v, off + 0.0001)
        n = body_normal(u, v)
        if sx() < 0:
            pass
        screw_at(parent, parent, p, n, i)


def build_shells():
    parts = []
    # dorsal shell: shingled bands (scapular-feather analogue), dark satin ceramic
    dbands = []
    dedges = [0.0, 0.40, 0.68, 0.985]
    for k in range(3):
        def du1(t, k=k):
            if k == 2:
                return dedges[3]
            return dedges[k + 1] + 0.012 + 0.07 * (1 - abs(2 * t - 1)) ** 1.4
        off = (2 - k) * 0.0026
        bd = body_panel(f'dband{k}', dedges[k] - (0.0 if k == 0 else 0.004), dedges[k + 1], BAY_LO, lambda u: 180 - BAY_LO(u),
                        nu=44 if k < 2 else 40, nv=80, off=off, thick=0.0016, m='shell_marked', u1_fn=du1, uvp='xy')
        dbands.append(bd)
    o = join(dbands, 'P_shell_rear')
    o = finalize(o, origin=body_pt(0.55, 90))
    parts.append(o)
    # breast: shingled bands with scalloped trailing edges (overlap rearward like contour feathers)
    bands = []
    edges = [0.0, 0.15, 0.29, 0.455]
    for k in range(3):
        def u1f(t, k=k):
            return edges[k + 1] + 0.010 + (0.055 if k < 2 else 0.03) * (1 - abs(2 * t - 1)) ** 1.3
        off = (2 - k) * 0.0026
        b = body_panel(f'band{k}', edges[k] - (0.0 if k == 0 else 0.004), edges[k + 1], lambda u: -141.0, lambda u: -39.0,
                       nu=40 if k < 2 else 56, nv=90, off=off, thick=0.0016, u1_fn=u1f, m='ceramic_barred', uvp='xy',
                       hole=leg_slot if k == 2 else None)
        bands.append(b)
    o = join(bands, 'P_shell_front')
    o = finalize(o, origin=body_pt(0.22, -90))
    parts.append(o)
    # ventral hatch with cooling louvres
    vh = body_panel('ventral', 0.462, 0.985, lambda u: -141.0, lambda u: -39.0, nu=60, nv=48, off=-0.0014, thick=0.0016, m='ceramic_barred',
                    uvp='xy', hole=leg_slot)
    louv = []
    for k in range(3):
        u = 0.62 + k * 0.045
        pts = [body_pt(u, v, 0.0010) for v in np.linspace(-108, -72, 14)]
        lo = sweep_obj(f'louv{k}', pts, (0.0009, 0.0032), m='ceramic_mid', nseg=12)
        louv.append(lo)
    o = join([vh] + louv, 'P_ventral')
    o = finalize(o, origin=body_pt(0.72, -90))
    parts.append(o)
    shell_screws('P_ventral', [(0.50, -60), (0.50, -120), (0.95, -60), (0.95, -120)])
    # flanks (mirrored pair)
    for s in (1, -1):
        with side(s):
            nm = S('P_flank_{s}')
            fl = body_panel(nm, 0.0, 0.985, lambda u: -38.2, lambda u: 6.2, nu=90, nv=26, m='shell_barred', uvp='zy')
            fl = finalize(fl, origin=body_pt(0.5, -15))
            parts.append(fl)
            shell_screws(nm, [(0.10, -33), (0.58, -33), (0.90, -30), (0.12, 2), (0.62, 2)])
    return parts


SCAP_U0, SCAP_U1, SCAP_VHI = 0.195, 0.475, 54.0


def build_scapular():
    """Left scapular stow cover (call inside side()): a feathered shoulder plate that drops over the
    stowed wing root (bird scapulars analogue) and swings in under the dorsal shell for flight.
    Returns (object, hinge point)."""
    def vlo(u):
        q = (u - SCAP_U0) / (SCAP_U1 - SCAP_U0)
        return -15.0 + 9.0 * (1 - abs(math.sin(q * math.pi * 4)) ** 0.55)

    def off(u, v):
        bu = bump(u, 0.26, 0.40, 0.07)
        bv = smoothstep(-24, 0, v) * (1 - smoothstep(22, SCAP_VHI, v))
        return 0.0030 + 0.0115 * bu * bv

    def fn(s_, t):
        u = SCAP_U0 + (SCAP_U1 - SCAP_U0) * s_
        v = vlo(u) + (SCAP_VHI - vlo(u)) * t
        return body_pt(u, v, off(u, v))
    verts, faces, uvs = grid_surface(fn, 56, 40)
    o = mesh_obj(S('P_scapular_{s}'), verts, faces, uvs=[(p.z / 0.06, -p.y / 0.06) for p in verts], mat_names=('shell_marked',))
    weld(o, 1e-7)
    me = o.data
    pc = me.polygons[len(me.polygons) // 2]
    if pc.normal.x * sx() < 0:
        import bmesh
        bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
    solidify(o, 0.0016, offset=-1)
    bevel(o, 0.0006, seg=2, angle=35, harden=False)
    hinge = body_pt(0.5 * (SCAP_U0 + SCAP_U1), SCAP_VHI, -0.002)
    o = finalize(o, origin=hinge)
    return o, hinge


# --------------------------------------------------------------------- frame
def yz_R():
    # plate local x->world Y, local y->world Z, local z->world X
    return M(((0, 0, 1), (1, 0, 0), (0, 1, 0)))


def xz_R():
    # plate local x->world X, local y->world Z, local z->world -Y (normal along Y)
    return M(((1, 0, 0), (0, 0, -1), (0, 1, 0)))


def tri_hole(cx, cy, w, h, up, r=0.002):
    """Rounded triangle lightening hole (truss look)."""
    pts = []
    if up:
        corners = [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2), (cx, cy + h / 2)]
    else:
        corners = [(cx - w / 2, cy + h / 2), (cx, cy - h / 2), (cx + w / 2, cy + h / 2)]
    # inset rounded corners
    n = len(corners)
    out = []
    for i in range(n):
        p0 = V(corners[i - 1] + (0,)); p1 = V(corners[i] + (0,)); p2 = V(corners[(i + 1) % n] + (0,))
        d0 = (p0 - p1).normalized(); d2 = (p2 - p1).normalized()
        a = d0.angle(d2)
        dist = r / math.tan(a / 2)
        c = p1 + (d0 + d2).normalized() * (r / math.sin(a / 2))
        s0 = p1 + d0 * dist; s2 = p1 + d2 * dist
        a0 = math.atan2(s0.y - c.y, s0.x - c.x); a2 = math.atan2(s2.y - c.y, s2.x - c.x)
        da = a2 - a0
        while da > math.pi: da -= TAU
        while da < -math.pi: da += TAU
        for k in range(5):
            aa = a0 + da * k / 4
            out.append((c.x + r * math.cos(aa), c.y + r * math.sin(aa)))
    return out


def build_frame():
    pieces = []
    # spine plate (keel) in YZ plane, truss-cut
    outer = [(-0.098, -0.004), (-0.090, -0.017), (-0.060, -0.022), (0.060, -0.022), (0.104, -0.014), (0.124, -0.004),
             (0.126, 0.006), (0.110, 0.012), (0.040, 0.017), (-0.060, 0.018), (-0.090, 0.011)]
    holes = []
    xs = np.linspace(-0.072, 0.096, 9)
    for i, cx in enumerate(xs):
        hh = 0.024 if abs(cx) < 0.07 else 0.016
        holes.append(tri_hole(cx, -0.002, 0.016, hh, i % 2 == 0, r=0.0022))
    pieces.append(plate('spine', outer, holes, thick=0.003, m='carbon', R=yz_R()))
    # battery label
    
    # front shoulder bulkhead (XZ plane, y=-0.060)
    ob = []
    for a in np.linspace(0, TAU, 72, endpoint=False):
        x = 0.079 * spow(math.cos(a), 0.8); z = 0.010 + 0.034 * spow(math.sin(a), 0.8)
        z = min(max(z, -0.021), 0.047)
        ob.append((x, z))
    from parts_wing import MX as CMX, MZ as CMZ
    hl = [circle_pts(0.0040, 24, 0.072, 0.020), circle_pts(0.0040, 24, -0.072, 0.020),
          circle_pts(0.0058, 24, CMX, 0.020), circle_pts(0.0058, 24, -CMX, 0.020),
          rounded_rect(0.034, 0.018, 0.005, cx=0.0, cy=0.012),
          rounded_rect(0.020, 0.016, 0.004, cx=0.036, cy=0.004), rounded_rect(0.020, 0.016, 0.004, cx=-0.036, cy=0.004)]
    pieces.append(plate('bulk_front', ob, hl, thick=0.0028, m='carbon', center=(0, -0.060, 0), R=xz_R()))
    for s in (1, -1):
        pieces.append(cyl_obj(f'brg_f{s}', (0.072 * s, -0.0640, 0.020), (0.072 * s, -0.0565, 0.020), 0.0070, m='alu', r_in=0.0036))
    # shoulder girdle: rear bearing bulkhead (y=-0.0115) + top/bottom closing plates -> box section
    MX = CMX
    ob_r = []
    for a in np.linspace(0, TAU, 72, endpoint=False):
        x = 0.080 * spow(math.cos(a), 0.8); z = 0.012 + 0.034 * spow(math.sin(a), 0.8)
        ob_r.append((x, min(max(z, -0.020), 0.047)))
    hl_r = [circle_pts(0.0040, 24, 0.072, 0.020), circle_pts(0.0040, 24, -0.072, 0.020),
            circle_pts(0.0058, 24, MX, 0.020), circle_pts(0.0058, 24, -MX, 0.020),
            rounded_rect(0.018, 0.020, 0.005, cx=0.0, cy=0.022)]
    pieces.append(plate('bulk_girdle', ob_r, hl_r, thick=0.0028, m='carbon', center=(0, -0.0115, 0), R=xz_R()))
    for s_ in (1, -1):
        pieces.append(cyl_obj(f'brg_r{s_}', (0.072 * s_, -0.0150, 0.020), (0.072 * s_, -0.0080, 0.020), 0.0070, m='alu', r_in=0.0036))
    for zz, nm in ((0.0470, 'gtop'), (-0.0205, 'gbot')):
        out = rounded_rect(0.064, 0.050, 0.008)
        hl = [rounded_rect(0.018, 0.026, 0.005, cx=dx, cy=0.0) for dx in (-0.015, 0.015)]
        pieces.append(plate(nm, out, hl, thick=0.0022, m='carbon', center=(0, -0.036, zz), R=M.Identity(3)))
    # motor bulkhead y=0.020
    ob2 = []
    for a in np.linspace(0, TAU, 72, endpoint=False):
        x = 0.070 * spow(math.cos(a), 0.7); z = 0.000 + 0.050 * spow(math.sin(a), 0.85)
        ob2.append((x, max(z, -0.048)))
    hl2 = [circle_pts(0.0060, 24, MX, CMZ), circle_pts(0.0060, 24, -MX, CMZ),
           rounded_rect(0.016, 0.014, 0.004, cx=0, cy=-0.010),
           rounded_rect(0.040, 0.012, 0.004, cx=0, cy=0.026)]
    for s_ in (1, -1):
        for k in range(4):
            a = TAU * k / 4 + 0.785
            hl2.append(circle_pts(0.0011, 12, MX * s_ + 0.0205 * math.cos(a), CMZ + 0.0205 * math.sin(a)))
    pieces.append(plate('bulk_motor', ob2, hl2, thick=0.0025, m='carbon', center=(0, 0.0205, 0), R=xz_R()))
    # rear bulkhead y=0.118
    ob3 = [(0.036 * spow(math.cos(a), 0.8), 0.004 + 0.020 * spow(math.sin(a), 0.8)) for a in np.linspace(0, TAU, 48, endpoint=False)]
    hl3 = [circle_pts(0.007, 24, 0, 0.004)]
    pieces.append(plate('bulk_rear', ob3, hl3, thick=0.0025, m='carbon', center=(0, 0.118, 0), R=xz_R()))
    # longerons
    for s in (1, -1):
        pieces.append(sweep_obj(f'lon_t{s}', [V((0.030 * s, -0.060, 0.034)), V((0.026 * s, 0.03, 0.033)), V((0.020 * s, 0.118, 0.019))], 0.0024, m='carbon', nseg=12))
        pieces.append(sweep_obj(f'lon_b{s}', [V((0.040 * s, -0.060, -0.020)), V((0.034 * s, 0.03, -0.030)), V((0.022 * s, 0.118, -0.014))], 0.0024, m='carbon', nseg=12))
    # battery cradle straps
    for yy in (-0.070, -0.022):
        pts = [V((0.020, yy, -0.022)), V((0.0275, yy, -0.026)), V((0.0275, yy, -0.052)), V((0.020, yy, -0.0565)),
               V((-0.020, yy, -0.0565)), V((-0.0275, yy, -0.052)), V((-0.0275, yy, -0.026)), V((-0.020, yy, -0.022))]
        pieces.append(sweep_obj(f'strap{yy}', pts, (0.0007, 0.0035), m='ti_brushed', nseg=8,
                                profile=lambda th: (math.cos(th), math.sin(th))))
    o = join(pieces, 'P_frame')
    o = finalize(o, origin=(0, 0, 0), uv_scale=40)
    for i, (x, y, z) in enumerate([(0.05, -0.0615, 0.03), (-0.05, -0.0615, 0.03), (0.05, -0.0615, -0.012), (-0.05, -0.0615, -0.012)]):
        screw_at('P_frame', 'P_frame', (x, y, z), (0, -1, 0), i)
    return o


# --------------------------------------------------------------------- battery & avionics
def build_battery():
    c = V((0, -0.046, -0.040))
    p = [rbox('bat_body', (0.050, 0.070, 0.027), c, m='graphite', bev=0.0025, seg=4)]
    # label/heat-spreader plate
    p.append(rbox('bat_plate', (0.040, 0.050, 0.0008), c + V((0, 0.002, -0.0138)), m='ti_brushed', bev=0.0003, seg=2))
    # cell seams
    for k in range(3):
        p.append(rbox(f'bat_seam{k}', (0.0508, 0.0012, 0.020), c + V((0, -0.0175 + k * 0.0175, 0)), m='alu_dark', bev=0.0003, seg=1))
    # connector + leads
    p.append(rbox('bat_conn', (0.014, 0.008, 0.007), c + V((0.012, 0.039, 0.006)), m='amber_plastic', bev=0.0008, seg=2))
    p.append(sweep_obj('bat_lead_p', [c + V((0.009, 0.035, 0.006)), c + V((0.009, 0.045, 0.012)), c + V((0.004, 0.06, 0.016))], 0.0016, m='rubber', nseg=10))
    p.append(sweep_obj('bat_lead_n', [c + V((0.015, 0.035, 0.006)), c + V((0.015, 0.045, 0.010)), c + V((0.010, 0.06, 0.012))], 0.0016, m='rubber', nseg=10))
    # XT30 main connector housing + JST-XH balance lead (5-way) with its plug
    xt = c + V((0.004, 0.068, 0.017))
    p.append(rbox('xt_body', (0.010, 0.006, 0.005), xt, m='amber_plastic', bev=0.0006, seg=2))
    for k, dx in enumerate((-0.0022, 0.0022)):
        p.append(cyl_obj(f'xt_pin{k}', xt + V((dx, 0.003, 0)), xt + V((dx, 0.0045, 0)), 0.0009, m='gold', nseg=10))
    for k in range(5):
        dx = -0.020 + k * 0.0016
        p.append(sweep_obj(f'bal{k}', [c + V((dx, 0.035, 0.010)), c + V((dx - 0.002, 0.041, 0.0155)), c + V((dx - 0.006, 0.040, 0.0175)),
                                         c + V((-0.026 + k * 0.0016, 0.030, 0.0180))], 0.00055,
                           m=('rubber' if k == 0 else 'cable' if k < 4 else 'flex'), nseg=6))
    p.append(rbox('bal_plug', (0.0095, 0.0045, 0.0040), c + V((-0.022, 0.026, 0.0180)), m='connector', bev=0.0004, seg=1))
    # status LEDs
    for k in range(4):
        p.append(rbox(f'bat_led{k}', (0.0025, 0.0016, 0.0006), c + V((-0.014 + k * 0.004, -0.030, -0.0138)), m='led', bev=0.0002, seg=1))
    o = join(p, 'P_battery')
    return finalize(o, origin=c, uv_scale=40)


def build_fc():
    c = V((0, 0.052, 0.028))
    p = []
    R_flat = M.Identity(3)
    for k, dz in enumerate((-0.004, 0.005)):
        o = plate(f'fc_board{k}', rounded_rect(0.040, 0.040, 0.002),
                  [circle_pts(0.0012, 12, dx * 0.0165, dy * 0.0165) for dx in (-1, 1) for dy in (-1, 1)],
                  thick=0.0016, m='pcb', center=c + V((0, 0, dz)), R=R_flat, bev=0.0002, seg=1)
        p.append(o)
    for dx in (-1, 1):
        for dy in (-1, 1):
            p.append(cyl_obj(f'fc_so{dx}{dy}', c + V((dx * 0.0165, dy * 0.0165, -0.0075)), c + V((dx * 0.0165, dy * 0.0165, 0.0075)), 0.0016, m='alu', nseg=6))
    # chips
    p.append(rbox('fc_mcu', (0.012, 0.012, 0.0014), c + V((0.004, -0.004, 0.0065)), m='chip', bev=0.0003, seg=1))
    p.append(rbox('fc_imu', (0.005, 0.005, 0.001), c + V((-0.010, 0.008, 0.0062)), m='chip', bev=0.0002, seg=1))
    p.append(rbox('fc_baro', (0.004, 0.003, 0.0012), c + V((-0.012, -0.010, 0.0062)), m='alu', bev=0.0002, seg=1))
    p.append(rbox('fc_mcu2', (0.010, 0.010, 0.0014), c + V((-0.002, 0.004, -0.0025)), m='chip', bev=0.0003, seg=1))
    for k in range(4):
        p.append(rbox(f'fc_cap{k}', (0.002, 0.0012, 0.0012), c + V((0.012, 0.010 - k * 0.003, 0.0062)), m='chip', bev=0.0001, seg=1))
    for k in range(3):
        p.append(rbox(f'fc_conn{k}', (0.008, 0.004, 0.003), c + V((-0.012 + k * 0.012, 0.0175, 0.007)), m='connector', bev=0.0003, seg=1))
    p.append(rbox('fc_shield', (0.016, 0.014, 0.002), c + V((-0.006, -0.008, -0.0052 + 0.0045)), m='alu', bev=0.0003, seg=1))
    o = join(p, 'P_fc')
    o = finalize(o, origin=c, uv_scale=60)
    return o


def build_imu():
    """Vibration-isolated IMU module near the CG, on the spine."""
    c = V((0, -0.030, 0.026))
    p = [rbox('imu_case', (0.014, 0.016, 0.009), c, m='alu', bev=0.0012, seg=3)]
    p.append(rbox('imu_lid', (0.0115, 0.0135, 0.0006), c + V((0, 0, 0.0047)), m='alu_dark', bev=0.0002, seg=1))
    for dx in (-1, 1):
        for dy in (-1, 1):
            p.append(lathe_obj(f'imu_g{dx}{dy}', [(0, -0.0002), (0.0024, -0.0002), (0.0029, 0.0012), (0.0024, 0.0026), (0, 0.0026)], 16,
                               c + V((dx * 0.0065, dy * 0.0075, -0.0074)), m='rubber'))
    p.append(rbox('imu_led', (0.002, 0.002, 0.0004), c + V((0.004, -0.005, 0.0051)), m='led', bev=0.0001, seg=1))
    return p


def build_vision():
    c = V((0, -0.090, -0.006))
    p = [rbox('vis_case', (0.030, 0.022, 0.017), c, m='alu', bev=0.0015, seg=3)]
    for k in range(6):
        p.append(rbox(f'vis_fin{k}', (0.0010, 0.020, 0.004), c + V((-0.0125 + k * 0.005, 0, 0.0100)), m='alu', bev=0.0003, seg=1))
    p.append(sweep_obj('vis_flex', [c + V((0, -0.006, 0.0085)), c + V((0, -0.016, 0.022)), c + V((0, -0.030, 0.040)), c + V((0, -0.046, 0.050))],
                       (0.0004, 0.0055), m='flex', nseg=8))
    p += build_imu()
    o = join(p, 'P_vision')
    return finalize(o, origin=c, uv_scale=60)


def build_pdu():
    c = V((0, 0.078, -0.030))
    p = []
    # busbars
    p.append(rbox('pdu_bus_p', (0.0025, 0.036, 0.006), c + V((0.004, 0, 0.004)), m='copper', bev=0.0004, seg=2))
    p.append(rbox('pdu_bus_n', (0.0025, 0.036, 0.006), c + V((-0.004, 0, 0.004)), m='copper', bev=0.0004, seg=2))
    p.append(rbox('pdu_iso', (0.0035, 0.038, 0.009), c + V((0, 0, 0.003)), m='rubber', bev=0.0005, seg=2))
    # capacitors
    for k in range(3):
        cc = c + V((0, -0.012 + k * 0.012, -0.007))
        p.append(lathe_obj(f'pdu_cap{k}', [(0, -0.0045), (0.0043, -0.0045), (0.0046, -0.004), (0.0046, 0.0035), (0.0040, 0.0045), (0, 0.0045)], 24,
                           cc, R=rot_to_z((0, 0, 1)), m='graphite'))
        p.append(lathe_obj(f'pdu_capt{k}', [(0, 0.0044), (0.0040, 0.0044), (0.0040, 0.0050), (0, 0.0050)], 24, cc, m='alu'))
    # ESC boards with heatsinks (both sides)
    for s in (1, -1):
        bc = c + V((0.014 * s, 0.0, -0.002))
        p.append(rbox(f'esc{s}', (0.0016, 0.040, 0.022), bc, m='pcb', bev=0.0002, seg=1))
        for k in range(7):
            p.append(rbox(f'esc_fin{s}{k}', (0.004, 0.0012, 0.020), bc + V((0.0028 * s, -0.016 + k * 0.0053, 0)), m='alu', bev=0.0003, seg=1))
        p.append(rbox(f'esc_base{s}', (0.0012, 0.038, 0.020), bc + V((0.0012 * s, 0, 0)), m='alu', bev=0.0002, seg=1))
    o = join(p, 'P_pdu')
    return finalize(o, origin=c, uv_scale=60)


# --------------------------------------------------------------------- surveillance payload
# The surveillance cameras are the head's own forward eyes on the neck gimbal (see parts_head.build_optic);
# the only separate payload part is the comms antenna below.
def build_antenna():
    """Dorsal comms blade antenna on the rear upper fuselage: command uplink and video/telemetry downlink."""
    bp = body_pt(0.62, 90.0)
    cx, cy, cz = 0.0, float(bp.y), float(bp.z)
    p = []
    # base collar
    p.append(lathe_obj('an_base', [(0.0, 0.0), (0.009, 0.0), (0.010, 0.003), (0.008, 0.006), (0.0, 0.007)],
                       nseg=24, center=(cx, cy, cz + 0.001), m='graphite_satin'))
    # raked blade (thin across X, raked back toward +Y)
    Rk = rot_axis((1, 0, 0), -0.42)
    p.append(rbox('an_blade', (0.0024, 0.012, 0.030), center=(cx, cy + 0.006, cz + 0.018), R=Rk, m='graphite_satin', bev=0.0008, seg=2))
    # gold tip
    tp = V((cx, cy + 0.018, cz + 0.030))
    p.append(cyl_obj('an_tip', tp, tp + V((0, 0.004, 0.006)), 0.0016, m='gold', nseg=10))
    o = join(p, 'P_antenna')
    return finalize(o, origin=(cx, cy, cz), uv_scale=60)
