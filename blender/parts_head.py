"""Head: cranial shell, facial discs with radiating composite blades, optics, beak, neck gimbal."""
import math
import numpy as np
from mathutils import Vector as V, Matrix as M
from lib import *

H = V((0.0, -0.150, 0.050))
AX, AY, AZ = 0.072, 0.053, 0.060
NE = 2.6                        # superellipsoid exponent: fuller cheeks, flatter face than an ellipsoid
EE = 2.0 / NE
CD = V((0.028, -0.008, 0))      # disc circle centre (x,z) rel H (left)
RD = 0.0435
E2 = (0.027, 0.005)             # eye centre (x,z) rel H (left)
DISH_D = 0.010
OPT_S = 0.84                    # optic scale
RHO_IN = 0.0190 * 0.84          # dish hole (eyelid slot)
MID = 0.0014                    # half-width of midline ridge gap


def yf(x, z):
    q = 1 - abs(x / AX) ** NE - abs(z / AZ) ** NE
    return H.y - AY * max(0.0016, q) ** (1.0 / NE)


def r_union(phi):
    d = V((math.cos(phi), math.sin(phi)))
    best = 0
    for c in (V((CD.x, CD.y)), V((-CD.x, CD.y))):
        b = d.dot(c)
        disc = b * b - c.length_squared + RD * RD
        t = b + math.sqrt(max(0, disc))
        best = max(best, t)
    return best


def r_left(theta):
    """Distance from eye centre E2 to the left-disc boundary along theta."""
    d = V((math.cos(theta), math.sin(theta)))
    e = V(E2)
    c = V((CD.x, CD.y))
    oc = e - c
    b = d.dot(oc)
    disc = b * b - (oc.length_squared - RD * RD)
    t = -b + math.sqrt(max(0, disc))
    if d.x < -1e-9:
        tm = (MID - e.x) / d.x
        t = min(t, tm)
    return t


def dish_depth(t):
    return DISH_D * max(0.0, 1 - t * t) ** 1.3


def dish_pt(rho, theta, off=0.0):
    x = E2[0] + rho * math.cos(theta); z = E2[1] + rho * math.sin(theta)
    t = rho / r_left(theta)
    y = yf(x, z) + dish_depth(t) - off
    return V((x, y, z)) + V((0, 0, H.z))


def rim_alpha(beta):
    cx, cz = AX * spow(math.cos(beta), EE), AZ * spow(math.sin(beta), EE)
    phi = math.atan2(cz, cx)
    den = math.hypot(cx, cz)
    return math.asin(min(0.995, r_union(phi) / den) ** (1.0 / EE))


def head_pt(alpha, beta, off=0.0):
    sa = spow(math.sin(alpha), EE)
    return H + V(((AX + off) * sa * spow(math.cos(beta), EE), -(AY + off) * spow(math.cos(alpha), EE), (AZ + off) * sa * spow(math.sin(beta), EE)))


def crown_bands():
    """Shingled plates flowing back from the brow over the crown (contour-feather analogue)."""
    out = []
    edges = [0.0, 0.26, 0.50, 0.74]
    b0, b1 = math.radians(-24), math.radians(204)
    for k in range(3):
        nsc = 7 - k
        def fn(s, t, k=k, nsc=nsc):
            beta = b0 + (b1 - b0) * t
            a0 = rim_alpha(beta) + 0.03
            span = (math.pi * 0.86 - a0)
            f = (t * nsc) % 1.0
            sc = 0.16 * math.sqrt(max(0.0, 1 - (2 * f - 1) ** 2))
            a_start = a0 + span * edges[k] - (0.0 if k == 0 else 0.05)
            a_end = a0 + span * edges[k + 1] + 0.05 + sc
            a = a_start + (a_end - a_start) * s
            # fade the band edges into the base near the sides of the head
            side = min(t, 1 - t)
            off = (3 - k) * 0.0013 * min(1.0, side / 0.08)
            return head_pt(a, beta, off)
        v, f, uv = grid_surface(fn, 16, 90)
        ob = mesh_obj(f'cband{k}', v, f, uvs=uv, mat_names=('shell_dark',))
        me = ob.data
        pc = V(me.polygons[len(me.polygons) // 2].center)
        if me.polygons[len(me.polygons) // 2].normal.dot(pc - H) < 0:
            import bmesh
            bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
        solidify(ob, 0.0012, offset=-1)
        bevel(ob, 0.0004, seg=1, angle=35, harden=False)
        out.append(ob)
    return out


def build_crown():
    def fn(s, t):
        beta = TAU * t
        a0 = rim_alpha(beta) + 0.022
        a = a0 + (math.pi - a0) * (s ** 0.9)
        return head_pt(a, beta)
    v, f, uv = grid_surface(fn, 46, 161)
    # cylindrical UVs about the vertical axis (~45 mm tiles): streaks run down the nape, the seam sits
    # at the front behind the facial discs
    uvs = []
    for p in v:
        d = p - H
        az = math.atan2(d.x, -d.y)
        if az < 0:
            az += TAU
        uvs.append((az * 9.0 / TAU, d.z / 0.045))
    o = mesh_obj('crown_base', v, f, uvs=uvs, mat_names=('shell_marked',))
    # unwrap faces that straddle the seam (9 whole tiles around, so the pattern stays continuous)
    uvl = o.data.uv_layers[0].data
    for poly in o.data.polygons:
        us = [uvl[li].uv[0] for li in poly.loop_indices]
        if max(us) - min(us) > 4.5:
            for li in poly.loop_indices:
                if uvl[li].uv[0] < 4.5:
                    uvl[li].uv[0] += 9.0
    weld(o, 1e-6)
    recalc_outward(o)
    # make sure normals point outward from H
    me = o.data
    p0 = me.polygons[len(me.polygons) // 3]
    if p0.normal.dot(V(p0.center) - H) < 0:
        import bmesh
        bm = bmesh.new(); bm.from_mesh(me)
        bmesh.ops.reverse_faces(bm, faces=bm.faces)
        bm.to_mesh(me); bm.free()
    solidify(o, 0.0018, offset=-1)
    bevel(o, 0.0006, seg=2, angle=35, harden=False)
    parts = [o]
    o = join(parts, 'P_crown')
    o = finalize(o, origin=H)
    # crown screws (rear, discreet)
    for i, (a, b) in enumerate(((2.35, 1.2), (2.35, 1.94))):
        p = H + V((AX * math.sin(a) * math.cos(b), -AY * math.cos(a), AZ * math.sin(a) * math.sin(b)))
        n = (p - H); n = V((n.x / AX**2, n.y / AY**2, n.z / AZ**2)).normalized()
        e = empty(f'FAST_P_crown_{i:02d}', p, rot_to_z(n), 0.003)
        link_parent(e.name, 'P_crown')
    return o


def blade_ring(name, n, t0, t1, off, width_scale, m, phase=0.0, tilt=0.0006, thick=0.0007):
    objs = []
    for k in range(n):
        th0 = phase + TAU * k / n
        hw = TAU / n * 0.5 * width_scale

        def fn(s, c, th0=th0, hw=hw):
            t = t0 + (t1 - t0) * s
            # taper toward the inner end, round the outer tip
            w = hw * (0.55 + 0.45 * s)
            ts = 0.72
            if s > ts:
                w *= math.sqrt(max(0.0, 1 - ((s - ts) / (1 - ts)) ** 2)) * 0.9 + 0.1 * (1 - (s - ts) / (1 - ts))
            th = th0 + (c * 2 - 1) * w
            rho = t * r_left(th)
            return dish_pt(rho, th, off + tilt * c)
        v, f, uv = grid_surface(fn, 12, 6)
        o = mesh_obj(f'{name}{k}', v, f, uvs=uv, mat_names=(m,))
        weld(o, 1e-7)
        # orient outward (toward viewer = -Y)
        me = o.data
        if me.polygons[len(me.polygons) // 2].normal.y > 0:
            import bmesh
            bm = bmesh.new(); bm.from_mesh(me)
            bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
        solidify(o, thick, offset=-1)
        objs.append(o)
    return objs


def midline_limited(th):
    """True where the disc boundary at angle th is the midline (between the discs), not the outer arc."""
    d = V((math.cos(th), math.sin(th)))
    e = V(E2); c = V((CD.x, CD.y)); oc = e - c
    b = d.dot(oc)
    t_c = -b + math.sqrt(max(0.0, b * b - (oc.length_squared - RD * RD)))
    return d.x < -1e-9 and (MID - e.x) / d.x < t_c * 0.995


def ruff_ring(name, n, t0, t1, off, width_scale, m, phase=0.0, tilt=0.0007, thick=0.0008):
    objs = []
    for k in range(n):
        th0 = phase + TAU * k / n
        if midline_limited(th0) or midline_limited(th0 - 0.1) or midline_limited(th0 + 0.1):
            continue
        hw = TAU / n * 0.5 * width_scale

        def fn(s_, c, th0=th0, hw=hw):
            t = t0 + (t1 - t0) * s_
            w = hw * (0.7 + 0.3 * s_)
            if s_ > 0.6:
                w *= math.sqrt(max(0.0, 1 - ((s_ - 0.6) / 0.4) ** 2)) * 0.85 + 0.15
            th = th0 + (c * 2 - 1) * w
            rho = t * r_left(th)
            return dish_pt(rho, th, off + tilt * c + 0.004 * max(0.0, t - 1.0))
        v, f, uv = grid_surface(fn, 8, 5)
        o = mesh_obj(f'{name}{k}', v, f, uvs=uv, mat_names=(m,))
        weld(o, 1e-7)
        me = o.data
        if me.polygons[len(me.polygons) // 2].normal.y > 0:
            import bmesh
            bm = bmesh.new(); bm.from_mesh(me)
            bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
        solidify(o, thick, offset=-1)
        objs.append(o)
    return objs


def build_face():
    """Left facial shell (call inside side())."""
    parts = []
    # dish
    def fn(s, t):
        th = TAU * t
        R = r_left(th)
        rho = RHO_IN + (R - RHO_IN) * s
        return dish_pt(rho, th)
    v, f, uv = grid_surface(fn, 22, 128, v_closed=True)
    dish = mesh_obj('dish', v, f, uvs=uv, mat_names=('shell_dark',))
    weld(dish, 1e-7)
    me = dish.data
    if me.polygons[len(me.polygons) // 2].normal.y > 0:
        import bmesh
        bm = bmesh.new(); bm.from_mesh(me)
        bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
    solidify(dish, 0.0014, offset=-1)
    bevel(dish, 0.0004, seg=2, angle=35, harden=False)
    parts.append(dish)
    # radiating composite blades: three concentric overlapping rings
    parts += blade_ring('bA', 22, 0.30, 0.60, 0.0012, 1.30, 'ceramic', phase=0.05)
    parts += blade_ring('bB', 28, 0.52, 0.80, 0.0020, 1.28, 'ceramic_mid', phase=0.11)
    parts += blade_ring('bC', 34, 0.72, 0.985, 0.0028, 1.26, 'ceramic', phase=0.02)
    # eye socket ring (sclerotic ring analogue)
    ring = []
    for k in range(97):
        th = TAU * k / 96
        ring.append(dish_pt(RHO_IN + 0.0006, th, 0.0006))
    parts.append(sweep_obj('socket', ring, (0.0012, 0.0016), m='alu_dark', nseg=10, cap=False))
    # rim lip along the outer arc (brow thicker)
    pts = []; rads = []
    e = V(E2); c = V((CD.x, CD.y))
    # parametrise the circle arc for x > MID
    a_top = math.atan2(math.sqrt(RD**2 - (MID - c.x) ** 2), MID - c.x)
    angles = np.linspace(a_top, -a_top, 160)   # outer arc (through theta = 0), the inner arc is the midline
    for a in angles:
        x = c.x + RD * math.cos(a); z = c.y + RD * math.sin(a)
        if x < MID - 1e-6:
            continue
        p = V((x, yf(x, z) - 0.0009, z + H.z))
        pts.append(p)
        brow = smoothstep(0.004, 0.026, z) * smoothstep(0.0, 0.030, x)
        rads.append((0.0026 + 0.0012 * brow, 0.0036 + 0.0016 * brow))
    # seam bead between disc and cranial shell (crown colour, slim)
    parts.append(sweep_obj('lip', pts, [(0.0012 + 0.0006 * (r[0] > 0.003), 0.0020) for r in rads], m='shell_marked', nseg=12,
                           twist_up=(0, -1, 0)))
    # facial ruff: two rows of short, overlapping scale-blades that ride over the disc edge onto the cranial
    # shell, framing the heart-shaped face (they stop where the two discs meet at the midline)
    parts += ruff_ring('rA', 44, 0.89, 1.02, 0.0034, 1.35, 'ceramic_mid', phase=0.03)
    parts += ruff_ring('rB', 50, 0.94, 1.05, 0.0046, 1.30, 'shell_marked', phase=0.07)
    o = join(parts, S('P_face_{s}'))
    o = finalize(o, origin=dish_pt(0.03, 0.0))
    return o


def eye_center():
    rho = RHO_IN
    y = yf(E2[0], E2[1]) + dish_depth(rho / r_left(0.0))
    return V((E2[0], y - 0.0035 * OPT_S, E2[1] + H.z))


def knurl_ring(name, r, h0, h1, center, R, n=120, depth=0.00025, m='alu'):
    verts = []; faces = []
    rows = [(r - 0.0004, h0), (r, h0 + 0.0004), (r, h1 - 0.0004), (r - 0.0004, h1)]
    for i, (rr, hh) in enumerate(rows):
        for j in range(n):
            a = TAU * j / n
            k = depth if (i in (1, 2) and j % 2 == 0) else 0.0
            verts.append(V(center) + R @ V(((rr + k) * math.cos(a), (rr + k) * math.sin(a), hh)))
    for i in range(len(rows) - 1):
        for j in range(n):
            a = i * n + j; b = i * n + (j + 1) % n; c = (i + 1) * n + (j + 1) % n; d = (i + 1) * n + j
            faces.append((a, b, c, d))
    # inner wall
    base = len(verts)
    for (rr, hh) in ((r - 0.0012, h1), (r - 0.0012, h0)):
        for j in range(n):
            a = TAU * j / n
            verts.append(V(center) + R @ V((rr * math.cos(a), rr * math.sin(a), hh)))
    for j in range(n):
        a = 3 * n + j; b = 3 * n + (j + 1) % n
        faces.append((a, b, base + (j + 1) % n, base + j))
        faces.append((base + j, base + (j + 1) % n, base + n + (j + 1) % n, base + n + j))
        faces.append((base + n + j, base + n + (j + 1) % n, (j + 1) % n, j))
    o = mesh_obj(name, verts, faces, mat_names=(m,))
    o.data.set_sharp_from_angle(angle=math.radians(40))
    return o


def build_optic():
    """Left optical assembly. Axis = -Y. Returns (part, [iris blade objs], [lid objs], eye_center)."""
    E = eye_center()
    R = rot_to_z((0, -1, 0))
    p = []
    p.append(lathe_obj('barrel', [(0, -0.034), (0.0112, -0.034), (0.0120, -0.033), (0.0120, -0.010), (0.0108, -0.009), (0, -0.009)], 48, E, R, m='graphite'))
    p.append(knurl_ring('focus', 0.0138, -0.010, -0.004, E, R, m='alu'))
    p.append(lathe_obj('bezel', [(0.0124, -0.0065), (0.0160, -0.0065), (0.0168, -0.0057), (0.0168, 0.0022), (0.0162, 0.0036),
                                 (0.0141, 0.0042), (0.0131, 0.0030), (0.0127, 0.0004), (0.0124, -0.0065)], 96, E, R, m='ti_brushed', sharp=35))
    # front glass dome
    Rc = 0.030; a = 0.0128; sag = Rc - math.sqrt(Rc * Rc - a * a)
    prof = [(0, -0.0004), (a, -0.0004)] + [(a * k / 10, 0.0010 + math.sqrt(Rc * Rc - (a * k / 10) ** 2) - (Rc - sag)) for k in range(10, -1, -1)]
    p.append(lathe_obj('glass', prof, 64, E, R, m='glass', sharp=60))
    p.append(lathe_obj('retainer', [(0.0099, -0.0034), (0.0127, -0.0034), (0.0127, -0.0010), (0.0099, -0.0010), (0.0099, -0.0034)], 64, E, R, m='alu_dark'))
    # inner lens elements + dark tube
    for k, (rr, hh, sg) in enumerate(((0.0095, -0.0060, 0.0012), (0.0068, -0.0100, 0.0009))):
        pr = [(0, hh - 0.0006), (rr, hh - 0.0006)] + [(rr * j / 6, hh + sg * (1 - (j / 6) ** 2)) for j in range(6, -1, -1)]
        p.append(lathe_obj(f'lens{k}', pr, 48, E, R, m='lens_inner', sharp=60))
    p.append(lathe_obj('tube', [(0.0100, -0.0170), (0.0118, -0.0170), (0.0118, -0.0035), (0.0100, -0.0035), (0.0100, -0.0170)], 48, E, R, m='alu_dark'))
    p.append(lathe_obj('sensorring', [(0.0028, -0.0142), (0.0056, -0.0142), (0.0056, -0.0136), (0.0028, -0.0136), (0.0028, -0.0142)], 48, E, R, m='gold'))
    p.append(lathe_obj('sensor', [(0, -0.0145), (0.0028, -0.0145), (0.0028, -0.0139), (0, -0.0139)], 32, E, R, m='amber'))
    p.append(lathe_obj('backplate', [(0, -0.0172), (0.0100, -0.0172), (0.0100, -0.0166), (0, -0.0166)], 32, E, R, m='alu_dark'))
    # focus motor + flex
    p.append(rbox('fmotor', (0.006, 0.010, 0.006), E + V((0.0, 0.020, 0.0135)), m='alu', bev=0.0008, seg=2))
    p.append(sweep_obj('oflex', [E + V((0, 0.033, 0)), E + V((0.0, 0.040, -0.004)), E + V((-0.012, 0.046, -0.012))], (0.0003, 0.004), m='flex', nseg=8))
    part = join(p, S('P_optic_{s}'))
    part = finalize(part, origin=E)
    part.data.transform(M.Scale(OPT_S, 4))
    # iris blades (gold anodised) – each a separate articulated leaf
    blades = []
    nb = 9
    for k in range(nb):
        a0 = TAU * k / nb
        outer = [(0.0112 * OPT_S * math.cos(a0 + math.radians(d)), 0.0112 * OPT_S * math.sin(a0 + math.radians(d))) for d in np.linspace(-6, 64, 14)]
        inner = []
        for d in np.linspace(64, -6, 14):
            aa = a0 + math.radians(d)
            rr = (0.0056 + 0.0010 * math.sin(math.radians(d + 6) / 70 * math.pi)) * OPT_S
            inner.append((rr * math.cos(aa), rr * math.sin(aa)))
        o = plate(S(f'K_iris_{{s}}_{k}'), outer + inner, thick=0.00022, m='gold', center=E + V((0, (0.0026 + 0.00005 * k) * OPT_S, 0)),
                  R=R, bev=0.0)
        piv = E + R @ (V((0.0106 * math.cos(a0 + math.radians(-2)), 0.0106 * math.sin(a0 + math.radians(-2)), -0.0026 - 0.00005 * k)) * OPT_S)
        o = finalize(o, origin=piv, R=R)
        blades.append(o)
    # eyelids: upper and lower flattened-spherical shutters rotating about local X through Ec.
    # Rest geometry = OPEN (rotated away); rig rotates toward closed to blink.
    lids = []
    Ec = E + V((0, 0.004 * OPT_S, 0))
    rl = 0.0195 * OPT_S
    for nm, sgn in (('U', 1), ('L', -1)):
        def fn(s, t, sgn=sgn):
            pol = math.radians(3 + 84 * s)
            az = math.radians(-93 + 186 * t) if sgn > 0 else math.radians(87 + 186 * t)
            d = V((math.sin(pol) * math.sin(az), -math.cos(pol) * 0.72, math.sin(pol) * math.cos(az)))
            return Ec + d * rl
        v, f, uv = grid_surface(fn, 14, 40)
        Ropen = rot_axis((1, 0, 0), math.radians(-78 if sgn > 0 else 78))
        v = [Ec + Ropen @ (V(q) - Ec) for q in v]
        o = mesh_obj(S(f'K_lid{nm}_{{s}}'), v, f, uvs=uv, mat_names=('graphite_satin',))
        me = o.data
        pc = V(me.polygons[len(me.polygons) // 2].center)
        if me.polygons[len(me.polygons) // 2].normal.dot(pc - Ec) < 0:
            import bmesh
            bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
        solidify(o, 0.0007, offset=-1)
        bevel(o, 0.0002, seg=1, angle=35, harden=False)
        o = finalize(o, origin=Ec)
        lids.append(o)
    return part, blades, lids, E, Ec


def build_beak():
    base_y = yf(0.0, -0.012)
    C = [V((0, base_y + 0.004, H.z - 0.006)), V((0, base_y - 0.006, H.z - 0.012)), V((0, base_y - 0.012, H.z - 0.021)),
         V((0, base_y - 0.0135, H.z - 0.031)), V((0, base_y - 0.0105, H.z - 0.040)), V((0, base_y - 0.0060, H.z - 0.0445))]
    # resample smoothly
    from mathutils.geometry import interpolate_bezier
    pts = []
    for i in range(len(C) - 1):
        a = C[i]; b = C[i + 1]
        ta = (C[min(i + 1, len(C) - 1)] - C[max(i - 1, 0)]) * 0.25
        tb = (C[min(i + 2, len(C) - 1)] - C[i]) * 0.25
        seg = interpolate_bezier(a, a + ta, b - tb, b, 16)
        pts += seg[:-1]
    pts.append(C[-1])
    n = len(pts)
    rads = []
    for i in range(n):
        s = i / (n - 1)
        w = 0.0088 * (1 - s) ** 0.85 + 0.0004
        h = 0.0090 * (1 - s) ** 0.9 + 0.0004
        rads.append((h, w))
    prof = lambda th: (spow(math.cos(th), 0.9) * (1.0 if math.cos(th) > 0 else 0.75), spow(math.sin(th), 0.8))
    o = sweep_obj('beak', pts, rads, m='beak', nseg=40, profile=prof, twist_up=(0, -0.6, 1))
    weld(o, 1e-7)
    p = [o]
    # cere sensor window (ToF) + status light
    cw = V((0, base_y - 0.0035, H.z - 0.0055))
    p.append(lathe_obj('tof', [(0, 0.0), (0.0032, 0.0), (0.0032, 0.0010), (0.0026, 0.0016), (0, 0.0018)], 32, cw,
                       rot_to_z(V((0, -0.55, 1))), m='lens_inner', sharp=50))
    p.append(lathe_obj('tofring', [(0.0032, -0.0002), (0.0040, -0.0002), (0.0040, 0.0012), (0.0032, 0.0012), (0.0032, -0.0002)], 32, cw,
                       rot_to_z(V((0, -0.55, 1))), m='alu_dark'))
    for s in (1, -1):
        p.append(rbox(f'nare{s}', (0.0022, 0.0040, 0.0012), V((0.0042 * s, base_y - 0.0060, H.z - 0.0110)), R=rot_axis((1, 0, 0), 0.9),
                      m='alu_dark', bev=0.0004, seg=2))
    o = join(p, 'P_beak')
    return finalize(o, origin=V((0, base_y, H.z - 0.010)))


def build_head_frame(E_left):
    p = []
    # facial perimeter ring (behind dishes)
    ring = []
    for k in range(121):
        phi = TAU * k / 120
        r = r_union(phi) * 0.86
        x = r * math.cos(phi); z = r * math.sin(phi)
        ring.append(V((x, yf(x, z) + 0.019, z + H.z)))
    p.append(sweep_obj('facering', ring, (0.0020, 0.0032), m='ti_brushed', nseg=10, cap=False))
    # midline ridge (visible keel between discs)
    pts = []
    for z in np.linspace(0.019, -0.013, 18):
        pts.append(V((0, yf(0, z) - 0.0012, z + H.z)))
    p.append(sweep_obj('ridge', pts, (0.0016, 0.0019), m='graphite_satin', nseg=12))
    # eye mounting rings + struts to hub
    hub = H + V((0, 0.004, -0.012))
    p.append(lathe_obj('hub', [(0, -0.003), (0.012, -0.003), (0.012, 0.003), (0, 0.003)], 32, hub, m='alu'))
    for s in (1, -1):
        ec = V((E_left.x * s, E_left.y + 0.024, E_left.z))
        p.append(lathe_obj(f'ering{s}', [(0.0108, -0.002), (0.0130, -0.002), (0.0130, 0.002), (0.0108, 0.002), (0.0108, -0.002)], 48, ec,
                           rot_to_z((0, -1, 0)), m='ti_brushed'))
        p.append(sweep_obj(f'strut{s}', [hub + V((0.006 * s, 0, 0.002)), ec + V((-0.004 * s, 0.006, -0.010)), ec + V((-0.006 * s, 0, -0.012))], 0.0020, m='ti_brushed', nseg=10))
        p.append(sweep_obj(f'strut2{s}', [hub + V((0.008 * s, -0.002, 0.003)), V((0.050 * s, H.y + 0.002, H.z + 0.018)), V((0.054 * s, H.y - 0.020, H.z + 0.020))], 0.0016, m='ti_brushed', nseg=10))
    p.append(sweep_obj('struttop', [hub + V((0, 0, 0.003)), H + V((0, -0.004, 0.030)), H + V((0, -0.022, 0.046))], 0.0018, m='ti_brushed', nseg=10))
    # acoustic sensor pods (asymmetric, barn-owl style)
    for s, dz in ((1, 0.006), (-1, -0.006)):
        c = V((0.060 * s, H.y - 0.020, H.z + dz))
        p.append(rbox(f'mic{s}', (0.006, 0.008, 0.010), c, m='alu_dark', bev=0.0012, seg=2))
    o = join(p, 'P_headFrame')
    return finalize(o, origin=hub, uv_scale=60)


def build_neck():
    p = []
    p.append(sweep_obj('post', [V((0, -0.094, 0.010)), V((0, -0.112, 0.022)), V((0, -0.128, 0.032))], 0.0065, m='carbon', nseg=20))
    p.append(lathe_obj('postcap', [(0, -0.004), (0.0085, -0.004), (0.0085, 0.004), (0, 0.004)], 32, V((0, -0.128, 0.032)),
                       rot_to_z(V((0, -0.016, 0.010))), m='alu'))
    for s in (1, -1):
        fork = [V((0.0, -0.130, 0.033)), V((0.012 * s, -0.135, 0.036)), V((0.022 * s, -0.142, 0.043)), V((0.0245 * s, -0.149, 0.049)), V((0.0245 * s, -0.150, 0.050))]
        p.append(sweep_obj(f'fork{s}', fork, (0.0018, 0.0042), m='ti_brushed', nseg=12, twist_up=(1, 0, 0)))
        p.append(cyl_obj(f'pbrg{s}', (0.0225 * s, H.y, H.z), (0.0280 * s, H.y, H.z), 0.0048, m='alu', nseg=24))
    p.append(rbox('pservo', (0.010, 0.012, 0.008), V((0.032, H.y + 0.004, H.z - 0.004)), m='graphite', bev=0.0012, seg=2))
    p.append(sweep_obj('nflex', [V((0, -0.098, -0.002)), V((-0.010, -0.112, 0.010)), V((-0.012, -0.132, 0.026)), V((-0.006, -0.146, 0.034))],
                       (0.0004, 0.005), m='flex', nseg=8))
    o = join(p, 'P_neck')
    return finalize(o, origin=V((0, -0.115, 0.024)), uv_scale=50)


def build_cradle():
    p = []
    p.append(cyl_obj('paxle', (-0.026, H.y, H.z), (0.026, H.y, H.z), 0.0026, m='steel', nseg=20))
    yc = H + V((0, 0.002, -0.016))
    p.append(lathe_obj('yawring', [(0.0135, -0.003), (0.0195, -0.003), (0.0205, -0.002), (0.0205, 0.002), (0.0195, 0.003), (0.0135, 0.003), (0.0135, -0.003)],
                       64, yc, m='alu'))
    p.append(lathe_obj('yawmotor', [(0, -0.009), (0.0128, -0.009), (0.0134, -0.008), (0.0134, -0.0035), (0, -0.0035)], 48, yc, m='graphite'))
    for s in (1, -1):
        p.append(sweep_obj(f'carm{s}', [V((0.021 * s, H.y, H.z)), V((0.021 * s, H.y + 0.002, H.z - 0.008)), yc + V((0.017 * s, 0, 0))], (0.0022, 0.0035), m='alu', nseg=10, twist_up=(1, 0, 0)))
    o = join(p, 'P_neckCradle')
    return finalize(o, origin=H, uv_scale=50)
