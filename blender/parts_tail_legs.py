"""Tail (2-axis actuated fan), landing gear (retractable legs, tendon-driven feet), perch, fastener template."""
import math
import bpy
import numpy as np
from mathutils import Vector as V, Matrix as M
from lib import *
from parts_wing import feather_geom, feather_obj

TAIL_PIV = V((0.0, 0.146, 0.008))
HIP = V((0.030, -0.002, -0.062))
ANKLE = V((0.031, -0.002, -0.126))
FOOT = V((0.032, -0.002, -0.170))
PERCH_Z = 1.35   # launch mast: the drop-assisted take-off needs ~0.5-1 m below the perch
N_RECT = 12


def build_tail_mount():
    p = []
    p.append(cyl_obj('boom', V((0, 0.116, 0.004)), V((0, 0.141, 0.008)), 0.0058, m='carbon', nseg=24))
    for s in (1, -1):
        p.append(sweep_obj(f'tyoke{s}', [V((0.004 * s, 0.128, 0.006)), V((0.013 * s, 0.136, 0.007)), V((0.0155 * s, 0.144, 0.008)), V((0.0155 * s, 0.1475, 0.008))],
                           (0.0016, 0.0042), m='ti_brushed', nseg=10, twist_up=(1, 0, 0)))
        p.append(cyl_obj(f'tbrg{s}', V((0.0140 * s, 0.146, 0.008)), V((0.0182 * s, 0.146, 0.008)), 0.0040, m='alu', nseg=20))
    p.append(rbox('tservo', (0.016, 0.012, 0.009), V((0, 0.133, -0.005)), m='graphite', bev=0.0012, seg=2))
    p.append(sweep_obj('tlink', [V((0.006, 0.137, -0.002)), V((0.010, 0.143, 0.001)), V((0.0125, 0.146, 0.002))], 0.0007, m='steel', nseg=8))
    o = join(p, 'P_tailMount')
    return finalize(o, origin=V((0, 0.130, 0.004)), uv_scale=60)


def rect_frame(k):
    t = k / (N_RECT - 1)
    x = -0.0165 + 0.033 * t
    y = 0.163 + 0.0035 * (1 - (x / 0.0175) ** 2)
    z = 0.0108 - 0.0009 * abs(k - 5.5)
    phi = math.radians(-21 + 42 * t)
    d = V((math.sin(phi), math.cos(phi), -0.035))
    return V((x, y, z)), frame_from_x(d), d


def build_tail_actuator():
    p = []
    c = V((0, 0.156, 0.009))
    p.append(rbox('thouse', (0.034, 0.018, 0.013), c, m='graphite', bev=0.0035, seg=4))
    p.append(rbox('tcover', (0.026, 0.012, 0.0008), c + V((0, 0, 0.0066)), m='ti_brushed', bev=0.0003, seg=1))
    for s in (1, -1):
        p.append(cyl_obj(f'ttrun{s}', V((0.0105 * s, 0.146, 0.008)), V((0.0145 * s, 0.146, 0.008)), 0.0028, m='steel', nseg=16))
    # socket bar
    arc = [rect_frame(k)[0] + V((0, -0.0035, -0.0008)) for k in range(N_RECT)]
    arc = [arc[0] + V((-0.003, -0.001, 0))] + arc + [arc[-1] + V((0.003, -0.001, 0))]
    p.append(sweep_obj('tbar', arc, (0.0030, 0.0042), m='alu', nseg=14))
    for k in range(N_RECT):
        o, R, d = rect_frame(k)
        p0 = o - d * 0.004
        p.append(cyl_obj(f'ts{k}', p0, p0 + d * 0.008, 0.0024, m='ti_brushed', nseg=12))
    # fan-spread cam ring (drives rectrix spread)
    p.append(cyl_obj('tcam', V((0, 0.160, 0.0170)), V((0, 0.160, 0.0185)), 0.0060, m='alu_dark', nseg=24))
    # model designation on the cover plate
    p.append(text_obj('tlabel', 'STRIX-01', 0.0034, c + V((0, -0.0005, 0.00705)), rot_axis((0, 0, 1), math.pi), m='alu_dark'))
    # upper and under tail coverts: overlap the rump and the rectrix roots and move with the fan,
    # so the tail root reads as one feathered unit instead of an exposed hinge
    for k, ph in enumerate((-15, -7.5, 0, 7.5, 15)):
        a = math.radians(ph)
        d = V((math.sin(a), math.cos(a), -0.16)).normalized()
        L = 0.064 - 0.006 * abs(ph) / 15
        p.append(feather_obj(f'tcov_u{k}', V((0.0045 * ph / 7.5, 0.133, 0.0265 - 0.0008 * abs(ph) / 7.5)), frame_from_x(d), L, 0.011, 0.011,
                             mat_vane='covert', thick=0.0010, curve=0.0, camber=0.0022, droop=0.003, fringe=False, rachis=False,
                             s0=0.0, tip_start=0.6, tip_point=0.7, nl=16, nc=7))
    for k, ph in enumerate((-9, 0, 9)):
        a = math.radians(ph)
        d = V((math.sin(a), math.cos(a), 0.10)).normalized()
        p.append(feather_obj(f'tcov_l{k}', V((0.004 * ph / 9, 0.136, -0.0075)), frame_from_x(d), 0.050, 0.010, 0.010,
                             mat_vane='covert', thick=0.0010, curve=0.0, camber=-0.0015, droop=0.0, fringe=False, rachis=False,
                             s0=0.0, tip_start=0.6, tip_point=0.7, nl=14, nc=7))
    o = join(p, 'P_tailActuator')
    o = finalize(o, origin=TAIL_PIV, uv_scale=60)
    for i, (x, y) in enumerate(((-0.010, 0.152), (0.010, 0.152), (-0.010, 0.160), (0.010, 0.160))):
        e = empty(f'FAST_P_tailActuator_{i:02d}', V((x, y, 0.0156)), None, 0.003)
        link_parent(e.name, 'P_tailActuator')
    return o


def build_rectrices():
    out = []
    for k in range(N_RECT):
        o, R, d = rect_frame(k)
        t = k / (N_RECT - 1)
        L = 0.160 + 0.016 * (1 - abs(t - 0.5) * 2)
        broad, narrow = 0.0198, (0.0092 if k in (0, N_RECT - 1) else 0.0122)
        wl, wt = (narrow, broad) if o.x >= 0 else (broad, narrow)
        f = feather_obj(f'P_rect_{k + 1:02d}', o, R, L, wl, wt, curve=0.0, camber=0.0018, droop=0.004,
                        tilt=math.radians(2.5 if o.x >= 0 else -2.5), fringe=True, tip_start=0.80)
        out.append(f)
    return out


# ---------------------------------------------------------------- landing gear
def build_hip_mount():
    p = []
    p.append(cyl_obj('hact', V((HIP.x - 0.012, HIP.y, HIP.z)), V((HIP.x + 0.009, HIP.y, HIP.z)), 0.0092, m='graphite', nseg=32))
    p.append(cyl_obj('hring', V((HIP.x - 0.013, HIP.y, HIP.z)), V((HIP.x - 0.010, HIP.y, HIP.z)), 0.0098, m='alu', nseg=32))
    p.append(sweep_obj('hbrk', [V((HIP.x - 0.004, HIP.y, HIP.z + 0.006)), V((HIP.x + 0.000, HIP.y + 0.001, HIP.z + 0.020)), V((0.033, -0.002, -0.030))],
                       (0.0018, 0.0060), m='ti_brushed', nseg=10, twist_up=(1, 0, 0)))
    o = join(p, S('P_hipMount_{s}'))
    return finalize(o, origin=HIP, uv_scale=60)


def leg_shell(name, z0, z1, r_of, center_y=0.0, ovality=1.22, m='ceramic_barred', thick=0.0012, cap_top=False):
    """Oval fairing between z0 (top) and z1 (bottom); r_of(s) gives the radius along s in [0,1]."""
    def fn(s, t):
        a = TAU * t
        z = z0 + (z1 - z0) * s
        r = r_of(s)
        return V((HIP.x + 0.0005 + r * math.cos(a), HIP.y + center_y + r * ovality * math.sin(a), z))
    v, f, uv = grid_surface(fn, 18, 49)
    o = mesh_obj(name, v, f, uvs=[(a * 2.0, b * abs(z1 - z0) / 0.06) for a, b in uv], mat_names=(m,))
    weld(o, 1e-7)
    me = o.data
    pc = V(me.polygons[len(me.polygons) // 2].center)
    if me.polygons[len(me.polygons) // 2].normal.dot(pc - V((HIP.x, HIP.y, pc.z))) < 0:
        import bmesh
        bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.reverse_faces(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
    solidify(o, thick, offset=-1)
    bevel(o, 0.0004, seg=1, angle=35, harden=False)
    return o


def build_leg_upper():
    """Hip hub, titanium tibia, two overlapping ceramic cowls (feathered-thigh analogue), toe-tendon actuator."""
    p = []
    p.append(cyl_obj('hhub', V((HIP.x - 0.009, HIP.y, HIP.z)), V((HIP.x + 0.009, HIP.y, HIP.z)), 0.0092, m='alu', nseg=32))
    p.append(cyl_obj('hcap', V((HIP.x + 0.009, HIP.y, HIP.z)), V((HIP.x + 0.0102, HIP.y, HIP.z)), 0.0060, m='accent', nseg=24))
    p.append(cyl_obj('tibia', HIP + V((0, 0, -0.004)), ANKLE + V((0, 0, 0.004)), 0.0042, m='ti_brushed', nseg=20))
    top = HIP.z + 0.010
    mid = HIP.z - 0.034
    p.append(leg_shell('cowl1', top, mid - 0.002, lambda s: 0.0185 * math.sqrt(max(0.0, 1 - (1 - s) ** 6)) * (1 - 0.18 * s) + 0.0005, center_y=-0.0015))
    p.append(leg_shell('cowl2', mid + 0.004, ANKLE.z + 0.006, lambda s: 0.0148 * (1 - 0.38 * s ** 1.2), center_y=-0.0010))
    for s_ in (1, -1):
        p.append(rbox(f'aclev{s_}', (0.0024, 0.010, 0.012), ANKLE + V((0.0066 * s_, 0, 0.003)), m='ti_brushed', bev=0.0008, seg=2))
    # toe-flexor tendon actuator riding on the back of the shin
    ta = V((HIP.x, HIP.y + 0.0150, HIP.z - 0.040))
    p.append(cyl_obj('tact', ta, ta + V((0, 0.0015, -0.024)), 0.0040, m='graphite', nseg=20))
    p.append(cyl_obj('tactc', ta + V((0, 0.0015, -0.024)), ta + V((0, 0.0016, -0.026)), 0.0042, m='accent', nseg=20))
    p.append(sweep_obj('tcab', [ta + V((0, 0.0016, -0.026)), ANKLE + V((0, 0.0090, 0.004)), ANKLE + V((0, 0.0062, -0.002))], 0.0007, m='cable', nseg=6))
    o = join(p, S('P_legUpper_{s}'))
    return finalize(o, origin=HIP, uv_scale=60)


def build_tarsus():
    p = []
    p.append(cyl_obj('knuckle', ANKLE + V((-0.0054, 0, 0)), ANKLE + V((0.0054, 0, 0)), 0.0058, m='alu', nseg=24))
    p.append(cyl_obj('strut', ANKLE + V((0, 0, -0.003)), FOOT + V((0, 0, 0.004)), 0.0038, m='ti_brushed', nseg=20))
    p.append(sweep_obj('tendon', [ANKLE + V((0, 0.0060, -0.002)), ANKLE + V((0, 0.0058, -0.016)), FOOT + V((0, 0.0052, 0.006))], 0.0008, m='cable', nseg=6))
    # full ceramic sleeve over strut and tendon (feathered-tarsus analogue)
    p.append(leg_shell('tsleeve', ANKLE.z - 0.005, FOOT.z + 0.0065, lambda s_: 0.0088 - 0.0016 * s_, center_y=0.0006, ovality=1.15))
    o = join(p, S('P_tarsus_{s}'))
    return finalize(o, origin=ANKLE, uv_scale=60)


TOES = {  # direction (left foot, body frame), segment lengths -- zygodactyl 2+2 perching grip
    1: ((-0.38, 1.0), [0.0130, 0.0100]),           # hallux (back, inner)
    2: ((-0.26, -1.0), [0.0140, 0.0110, 0.0090]),  # inner forward
    3: ((0.26, -1.0), [0.0160, 0.0120, 0.0100]),   # middle forward
    4: ((0.42, 1.0), [0.0130, 0.0100, 0.0080]),    # outer toe reversed (back, outer)
}


def toe_segment(name, p0, d, L, r0, r1, last):
    R = frame_from_x(d)
    p = []
    p.append(sweep_obj(name + 'b', [p0 + R @ V((0.0025, 0, 0)), p0 + R @ V((L * 0.5, 0, 0.0002)), p0 + R @ V((L - 0.0012, 0, 0))],
                       [(r0, r0 * 1.05), (r0 * 0.95, r0), (r1, r1 * 1.05)], m='graphite', nseg=16))
    p.append(cyl_obj(name + 'k', p0 + R @ V((0, -0.0030, 0)), p0 + R @ V((0, 0.0030, 0)), 0.0026, m='alu_dark', nseg=16))
    p.append(sweep_obj(name + 'pad', [p0 + R @ V((0.003, 0, -0.0022)), p0 + R @ V((L * 0.55, 0, -0.0026)), p0 + R @ V((L - 0.002, 0, -0.0022))],
                       [(0.0011, 0.0024), (0.0013, 0.0026), (0.0010, 0.0020)], m='rubber', nseg=12))
    if last:
        pts = []; rads = []
        for k in range(12):
            a = math.radians(95 * k / 11)
            rr = 0.0105
            pts.append(p0 + R @ V((L - 0.002 + rr * math.sin(a), 0, -rr * (1 - math.cos(a)) + 0.0006)))
            w = 0.0026 * (1 - k / 11) ** 0.9 + 0.0002
            rads.append((w * 1.15, w * 0.85))
        p.append(sweep_obj(name + 't', pts, rads, m='ceramic_dark', nseg=14))
    o = join(p, name)
    return finalize(o, origin=p0, R=R)


def build_foot():
    p = []
    p.append(rbox('fhub', (0.013, 0.014, 0.0085), FOOT + V((0, 0, -0.001)), m='alu', bev=0.0022, seg=3))
    p.append(cyl_obj('fpul', FOOT + V((-0.0045, 0.0, 0.0045)), FOOT + V((0.0045, 0.0, 0.0045)), 0.0040, m='steel', nseg=20))
    hub = join(p, S('P_foot_{s}'))
    hub = finalize(hub, origin=FOOT, uv_scale=60)
    joints = []; segs = []
    for tk, ((dx, dy), lens) in TOES.items():
        d = V((dx, dy, 0)).normalized()
        R = frame_from_x(d)
        p0 = FOOT + d * 0.0062 + V((0, 0, -0.0022))
        parent = hub.name
        for j, L in enumerate(lens):
            jn = S(f'J_toe_{{s}}_{tk}_{j}')
            empty(jn, p0, R, 0.004)
            link_parent(jn, parent)
            sn = S(f'K_toe_{{s}}_{tk}_{j}')
            r0 = 0.0031 - 0.0003 * j; r1 = r0 - 0.0004
            sg = toe_segment(sn, p0, d, L, r0, r1, j == len(lens) - 1)
            link_parent(sn, jn)
            parent = jn
            p0 = p0 + d * L
    g = S('G_grip_{s}')
    empty(g, FOOT + V((0, 0, -0.0215)), None, 0.004)
    link_parent(g, hub.name)
    return hub


# ---------------------------------------------------------------- perch & templates
def build_perch():
    """Weighted plinth, slender graphite column and a matte anodised perch bar with a rubber sleeve."""
    p = []
    p.append(lathe_obj('base', [(0, 0), (0.158, 0), (0.164, 0.004), (0.165, 0.015), (0.159, 0.021), (0.045, 0.024), (0.022, 0.036), (0, 0.037)],
                       112, V((0, 0, 0)), m='graphite', sharp=30))
    p.append(lathe_obj('bring', [(0.1645, 0.0065), (0.1662, 0.0065), (0.1662, 0.0115), (0.1645, 0.0115), (0.1645, 0.0065)], 112, V((0, 0, 0)), m='ti_brushed'))
    # tapered launch mast with a mid-height titanium collar and the charging cable run
    p.append(lathe_obj('post', [(0, 0.034), (0.0175, 0.034), (0.0150, 0.45), (0.0105, PERCH_Z - 0.040), (0, PERCH_Z - 0.040)], 56, V((0, 0, 0)), m='graphite', sharp=60))
    p.append(lathe_obj('mcollar', [(0.0150, 0.70), (0.0162, 0.70), (0.0162, 0.735), (0.0150, 0.735), (0.0150, 0.70)], 48, V((0, 0, 0)), m='ti_brushed'))
    cab = [V((0.0, 0.0205, 0.040))] + [V((0.0, 0.0160 + 0.0045 * (1 - z / PERCH_Z), z)) for z in np.linspace(0.08, PERCH_Z - 0.06, 12)] + [V((0.0, 0.0125, PERCH_Z - 0.035))]
    p.append(sweep_obj('pcable', cab, 0.0022, m='rubber', nseg=10))
    for zc in (0.30, 0.95):
        p.append(rbox(f'clip{zc}', (0.008, 0.008, 0.010), V((0.0, 0.0165 + 0.0045 * (1 - zc / PERCH_Z), zc)), m='alu_dark', bev=0.0012, seg=2))
    p.append(lathe_obj('collar', [(0.0089, PERCH_Z - 0.046), (0.0110, PERCH_Z - 0.046), (0.0110, PERCH_Z - 0.040), (0.0089, PERCH_Z - 0.040), (0.0089, PERCH_Z - 0.046)],
                       40, V((0, 0, 0)), m='accent'))
    # saddle block carrying the bar
    sad = [V((0, 0, PERCH_Z - 0.042)), V((0, 0, PERCH_Z - 0.026)), V((0, 0, PERCH_Z - 0.012))]
    p.append(sweep_obj('saddle', sad, [(0.0092, 0.0092), (0.0135, 0.0175), (0.0150, 0.0200)], m='graphite', nseg=32))
    # short machined titanium perch bar (fits between the folded wings): knurled grip zones under the
    # feet, a gold charging-contact strip along the top and a thin amber status ring at the centre
    from parts_head import knurl_ring
    HB = 0.050
    Rx = rot_to_z((1, 0, 0))
    p.append(cyl_obj('bar', V((-HB, 0, PERCH_Z)), V((HB, 0, PERCH_Z)), 0.0135, m='ti_brushed', nseg=56))
    for s_ in (1, -1):
        p.append(knurl_ring(f'knurl{s_}', 0.0141, -0.012, 0.012, V((0.032 * s_, 0, PERCH_Z)), Rx, n=144, depth=0.0003, m='alu'))
        p.append(lathe_obj(f'endcap{s_}', [(0, 0), (0.0138, 0), (0.0140, 0.0010), (0.0128, 0.0040), (0.0090, 0.0058), (0, 0.0062)], 48,
                           V((HB * s_, 0, PERCH_Z)), rot_to_z((s_, 0, 0)), m='graphite'))
    p.append(rbox('contacts', (0.082, 0.0040, 0.0010), V((0, 0, PERCH_Z + 0.0137)), m='gold', bev=0.0003, seg=1))
    p.append(cyl_obj('statring', V((-0.0012, 0, PERCH_Z)), V((0.0012, 0, PERCH_Z)), 0.0142, m='amber', nseg=48))
    for k in range(3):
        a = TAU * k / 3 + 0.5
        p.append(lathe_obj(f'foot{k}', [(0, -0.004), (0.010, -0.004), (0.012, -0.002), (0.012, 0.001), (0, 0.001)], 24,
                           V((0.128 * math.cos(a), 0.128 * math.sin(a), 0.0)), m='rubber'))
    o = join(p, 'PERCH')
    return finalize(o, origin=(0, 0, 0), uv_scale=40)


def text_obj(name, text, size, center, R, m='alu_dark', depth=0.0002):
    cu = bpy.data.curves.new(name + '_t', 'FONT')
    cu.body = text
    cu.size = size
    cu.extrude = depth
    cu.align_x = 'CENTER'
    cu.align_y = 'CENTER'
    co = bpy.data.objects.new(name + '_tmp', cu)
    bpy.context.scene.collection.objects.link(co)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(co.evaluated_get(dg))
    bpy.data.objects.remove(co); bpy.data.curves.remove(cu)
    verts = [V(center) + R @ v.co for v in me.vertices]
    faces = [pp.vertices[:] for pp in me.polygons]
    bpy.data.meshes.remove(me)
    o = mesh_obj(name, verts, faces, mat_names=(m,), smooth=False)
    return o


def build_screw_template():
    prof = [(0, 0), (0.0019, 0), (0.0019, 0.0011), (0.0016, 0.0015), (0, 0.0015)]
    o = lathe_obj('TPL_screw_head', prof, 24, V((0, 0, -0.0013)), m='ti_brushed', sharp=35)
    hexp = [(0.0009 * math.cos(TAU * k / 6), 0.0009 * math.sin(TAU * k / 6)) for k in range(6)]
    h = plate('TPL_screw_hex', hexp, thick=0.0008, m='alu_dark', center=V((0, 0, 0.0000)), bev=0.0)
    o = join([o, h], 'TPL_screw')
    return finalize(o, origin=(0, 0, 0))
