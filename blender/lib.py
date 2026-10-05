"""Geometry helpers for the STRIX-01 Blender build.

Conventions (Blender space): +X = owl's left, -Y = forward (owl faces -Y), +Z = up.
glTF export converts to +X left, +Y up, +Z forward.

All builders emit LEFT-side world coordinates. Wrapping a build in `with side(-1)`
mirrors every emitted vertex / frame across X so right-side parts are generated
with correct winding and proper (det=+1) pivot frames.
"""
import bpy, bmesh, math
import numpy as np
from mathutils import Vector as V, Matrix as M, Quaternion as Q

TAU = math.tau
COLL = None
MATS = {}
SIDE = [1]
WORLD = {}        # object name -> intended world matrix (Blender space)
PARENT = {}       # object name -> parent name


def init_scene():
    global COLL
    bpy.ops.wm.read_factory_settings(use_empty=True)
    COLL = bpy.context.scene.collection
    MATS.clear(); WORLD.clear(); PARENT.clear()


class side:
    def __init__(self, s): self.s = s
    def __enter__(self): SIDE.append(self.s)
    def __exit__(self, *a): SIDE.pop()


def sx():
    return SIDE[-1]


def S(name):
    """Resolve '{s}' in a part name to L or R for the current build side."""
    return name.replace('{s}', 'R' if sx() < 0 else 'L')


MIR = M(((-1, 0, 0), (0, 1, 0), (0, 0, 1)))


def mframe(R):
    """Mirror a 3x3 rotation for the current side (proper rotation M R M)."""
    if sx() < 0:
        return MIR @ R @ MIR
    return R


def mvec(v):
    v = V(v)
    if sx() < 0:
        v = V((-v.x, v.y, v.z))
    return v


# ---------------------------------------------------------------- materials

def mat(name):
    m = MATS.get(name)
    if m is None:
        m = bpy.data.materials.new(name)
        MATS[name] = m
    return m


# ---------------------------------------------------------------- math

def pchip(xs, ys):
    xs = np.asarray(xs, float); ys = np.asarray(ys, float)
    h = np.diff(xs); d = np.diff(ys) / h
    n = len(xs); m = np.zeros(n)
    m[0] = d[0]; m[-1] = d[-1]
    for k in range(1, n - 1):
        if d[k - 1] * d[k] <= 0:
            m[k] = 0
        else:
            w1 = 2 * h[k] + h[k - 1]; w2 = h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])

    def f(x):
        x = np.clip(x, xs[0], xs[-1])
        k = np.clip(np.searchsorted(xs, x) - 1, 0, n - 2)
        t = (x - xs[k]) / h[k]
        h00 = 2 * t**3 - 3 * t**2 + 1; h10 = t**3 - 2 * t**2 + t
        h01 = -2 * t**3 + 3 * t**2; h11 = t**3 - t**2
        return h00 * ys[k] + h10 * h[k] * m[k] + h01 * ys[k + 1] + h11 * h[k] * m[k + 1]
    return f


def smoothstep(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def bump(x, a, b, ramp):
    """1 inside [a,b], smooth ramps of width `ramp` outside."""
    return smoothstep(a - ramp, a, x) * (1 - smoothstep(b, b + ramp, x))


def spow(x, p):
    return math.copysign(abs(x) ** p, x)


def frame_from_x(xdir, up=(0, 0, 1)):
    """Rotation whose local X = xdir, local Z close to `up`."""
    x = V(xdir).normalized()
    u = V(up)
    y = u.cross(x)
    if y.length < 1e-6:
        y = V((0, 1, 0)).cross(x)
    y.normalize()
    z = x.cross(y).normalized()
    return M((x, y, z)).transposed()


def rot_axis(axis, ang):
    return M.Rotation(ang, 3, V(axis).normalized())


# ---------------------------------------------------------------- mesh creation

def make_mesh(name, verts, faces, uvs=None, smooth=True, mat_names=('graphite',), face_mat=None):
    verts = np.asarray(verts, dtype=float).reshape(-1, 3).copy()
    faces = [list(f) for f in faces]
    if sx() < 0:
        verts[:, 0] *= -1
        faces = [f[::-1] for f in faces]
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts.tolist(), [], faces)
    if uvs is not None:
        uvs = np.asarray(uvs, dtype=float).reshape(-1, 2)
        uvl = me.uv_layers.new(name='UVMap')
        vi = np.zeros(len(me.loops), dtype=np.int32)
        me.loops.foreach_get('vertex_index', vi)
        uvl.data.foreach_set('uv', uvs[vi].ravel())
    for m in mat_names:
        me.materials.append(mat(m))
    if face_mat is not None:
        me.polygons.foreach_set('material_index', [int(i) for i in face_mat])
    me.validate(clean_customdata=False)
    if smooth:
        me.shade_smooth()
    else:
        me.shade_flat()
    return me


def obj(name, me):
    o = bpy.data.objects.new(name, me)
    COLL.objects.link(o)
    return o


def empty(name, origin=(0, 0, 0), R=None, size=0.01):
    """Empty at a LEFT-space origin/rotation (mirrored automatically)."""
    o = bpy.data.objects.new(name, None)
    o.empty_display_size = size
    COLL.objects.link(o)
    R = M.Identity(3) if R is None else R
    W = M.Translation(mvec(origin)) @ mframe(R).to_4x4()
    o.matrix_world = W
    WORLD[name] = W
    return o


def mesh_obj(name, verts, faces, **kw):
    return obj(name, make_mesh(name, verts, faces, **kw))


def bake(o):
    if not o.modifiers and o.type == 'MESH':
        return o
    dg = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(ev, preserve_all_data_layers=True, depsgraph=dg)
    o.modifiers.clear()
    if o.type == 'MESH':
        old = o.data
        o.data = me
        bpy.data.meshes.remove(old)
        return o
    # curve -> new mesh object
    name = o.name
    mats = [s.material for s in o.material_slots]
    bpy.data.objects.remove(o)
    n = obj(name, me)
    return n


def solidify(o, t, offset=-1.0, rim=True):
    m = o.modifiers.new('sol', 'SOLIDIFY')
    m.thickness = t; m.offset = offset; m.use_rim = rim
    m.use_even_offset = True; m.use_quality_normals = True
    return m


def bevel(o, w, seg=3, angle=30, harden=True, limit='ANGLE', clamp=True):
    m = o.modifiers.new('bev', 'BEVEL')
    m.width = w; m.segments = seg; m.limit_method = limit
    m.angle_limit = math.radians(angle)
    m.harden_normals = harden
    m.use_clamp_overlap = clamp
    m.miter_outer = 'MITER_ARC'
    return m


def weld(o, dist=1e-6):
    bm = bmesh.new(); bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    bm.to_mesh(o.data); bm.free()
    return o


def recalc_outward(o):
    bm = bmesh.new(); bm.from_mesh(o.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(o.data); bm.free()


def join(objs, name):
    objs = [bake(o) for o in objs]
    objs = [o for o in objs if o is not None]
    for o in objs:
        if o.type == 'MESH' and not o.data.uv_layers:
            box_uv(o, 40.0)
    if len(objs) > 1:
        for o in objs:
            o.select_set(True)
        with bpy.context.temp_override(active_object=objs[0], selected_editable_objects=objs,
                                       selected_objects=objs, object=objs[0]):
            bpy.ops.object.join()
    o = objs[0]
    o.name = name
    o.data.name = name
    return o


def box_uv(o, scale=30.0):
    me = o.data
    if not me.uv_layers:
        me.uv_layers.new(name='UVMap')
    uvl = me.uv_layers[0]
    co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get('co', co); co = co.reshape(-1, 3)
    vi = np.zeros(len(me.loops), dtype=np.int32); me.loops.foreach_get('vertex_index', vi)
    pn = np.zeros(len(me.polygons) * 3); me.polygons.foreach_get('normal', pn); pn = pn.reshape(-1, 3)
    ls = np.zeros(len(me.polygons), dtype=np.int32); me.polygons.foreach_get('loop_start', ls)
    lt = np.zeros(len(me.polygons), dtype=np.int32); me.polygons.foreach_get('loop_total', lt)
    loop_face = np.repeat(np.arange(len(me.polygons)), lt)
    ax = np.argmax(np.abs(pn), axis=1)[loop_face]
    p = co[vi]
    u = np.where(ax == 0, p[:, 1], p[:, 0])
    v = np.where(ax == 2, p[:, 1], p[:, 2])
    uvl.data.foreach_set('uv', (np.stack([u, v], 1) * scale).ravel())


def set_origin(o, origin, R=None):
    """Move object pivot to LEFT-space origin/rotation, keeping geometry in place."""
    R = M.Identity(3) if R is None else R
    W = M.Translation(mvec(origin)) @ mframe(R).to_4x4()
    o.data.transform(W.inverted() @ o.matrix_world)
    o.matrix_world = W
    WORLD[o.name] = W
    return o


def finalize(o, origin=(0, 0, 0), R=None, uv_scale=None):
    o = bake(o)
    if not o.data.uv_layers:
        box_uv(o, uv_scale or 40.0)
    set_origin(o, origin, R)
    return o


def link_parent(child, parent):
    PARENT[child if isinstance(child, str) else child.name] = parent if isinstance(parent, str) else parent.name


def apply_hierarchy():
    objs = bpy.data.objects
    for c, p in PARENT.items():
        co = objs[c]; po = objs[p]
        Wc = WORLD.get(c, co.matrix_world.copy())
        Wp = WORLD.get(p, po.matrix_world.copy())
        co.parent = po
        co.matrix_parent_inverse = M.Identity(4)
        co.matrix_basis = Wp.inverted() @ Wc


# ---------------------------------------------------------------- primitives (left-space)

def grid_surface(fn, nu, nv, u_closed=False, v_closed=False):
    """fn(u,v)->(x,y,z), u,v in [0,1]. Returns verts, faces, uvs."""
    us = [i / (nu if u_closed else nu - 1) for i in range(nu)]
    vs = [j / (nv if v_closed else nv - 1) for j in range(nv)]
    verts = []; uvs = []
    for u in us:
        for v in vs:
            verts.append(fn(u, v)); uvs.append((v, u))
    faces = []
    iu = nu if u_closed else nu - 1
    iv = nv if v_closed else nv - 1
    for i in range(iu):
        for j in range(iv):
            a = i * nv + j; b = ((i + 1) % nu) * nv + j
            c = ((i + 1) % nu) * nv + (j + 1) % nv; d = i * nv + (j + 1) % nv
            faces.append((a, b, c, d))
    return verts, faces, uvs


def lathe(profile, nseg=48, center=(0, 0, 0), R=None, a0=0.0, a1=TAU):
    """profile: list of (r, h) along local Z. Rotated by R, placed at center."""
    R = M.Identity(3) if R is None else R
    c = V(center)
    full = abs(a1 - a0 - TAU) < 1e-9
    n = nseg if full else nseg + 1
    verts = []; uvs = []
    for i, (r, h) in enumerate(profile):
        for j in range(n):
            a = a0 + (a1 - a0) * j / nseg
            p = V((r * math.cos(a), r * math.sin(a), h))
            verts.append(c + R @ p); uvs.append((j / nseg * 4, h * 30))
    faces = []
    for i in range(len(profile) - 1):
        for j in range(nseg):
            a = i * n + j; b = i * n + (j + 1) % n
            cc = (i + 1) * n + (j + 1) % n; d = (i + 1) * n + j
            faces.append((a, b, cc, d))
    return verts, faces, uvs


def lathe_obj(name, profile, nseg=48, center=(0, 0, 0), R=None, m='alu', smooth=True, bev=None, sharp=40):
    v, f, uv = lathe(profile, nseg, center, R)
    o = mesh_obj(name, v, f, uvs=uv, mat_names=(m,), smooth=smooth)
    weld(o, 1e-7)
    if bev:
        bevel(o, bev, seg=2, angle=40)
    elif sharp:
        o.data.set_sharp_from_angle(angle=math.radians(sharp))
    return o


def cyl_profile(r, h0, h1, ch=0.0004, r_in=0.0):
    """Closed cylinder (optionally tube) profile with small chamfers."""
    if r_in <= 0:
        return [(0, h0), (r - ch, h0), (r, h0 + ch), (r, h1 - ch), (r - ch, h1), (0, h1)]
    return [(r_in, h0), (r - ch, h0), (r, h0 + ch), (r, h1 - ch), (r - ch, h1), (r_in, h1), (r_in, h0)]


def cyl_obj(name, p0, p1, r, m='alu', nseg=32, ch=0.0004, r_in=0.0):
    p0 = V(p0); p1 = V(p1); d = p1 - p0
    Rz = rot_to_z(d)
    v, f, uv = lathe(cyl_profile(r, 0, d.length, ch, r_in), nseg, p0, Rz)
    o = mesh_obj(name, v, f, uvs=uv, mat_names=(m,))
    weld(o, 1e-7)
    o.data.set_sharp_from_angle(angle=math.radians(40))
    return o


def rot_to_z(d):
    """Rotation mapping local +Z onto direction d."""
    d = V(d).normalized()
    z = V((0, 0, 1))
    if (z - d).length < 1e-9:
        return M.Identity(3)
    if (z + d).length < 1e-9:
        return M.Rotation(math.pi, 3, 'X')
    return z.rotation_difference(d).to_matrix()


def sweep(path, radius, nseg=16, profile=None, cap=True, twist_up=(0, 0, 1)):
    """Tube along polyline `path` (list of points). radius: float, list, or (rx,ry) pairs.
    profile(theta)->(px,py) unit cross-section (default circle)."""
    path = [V(p) for p in path]
    n = len(path)
    if not isinstance(radius, list):
        radius = [radius] * n
    T = []
    for i in range(n):
        if i == 0:
            t = path[1] - path[0]
        elif i == n - 1:
            t = path[-1] - path[-2]
        else:
            t = path[i + 1] - path[i - 1]
        T.append(t.normalized())
    up = V(twist_up)
    N0 = up - T[0] * up.dot(T[0])
    if N0.length < 1e-6:
        N0 = V((1, 0, 0)) - T[0] * T[0].x
    N0.normalize()
    Ns = [N0]
    for i in range(1, n):
        nprev = Ns[-1]
        nn = nprev - T[i] * nprev.dot(T[i])
        if nn.length < 1e-9:
            nn = nprev
        Ns.append(nn.normalized())
    verts = []; uvs = []
    for i in range(n):
        B = T[i].cross(Ns[i]).normalized()
        r = radius[i]
        rx, ry = (r, r) if not isinstance(r, (list, tuple)) else r
        for j in range(nseg):
            th = TAU * j / nseg
            if profile:
                px, py = profile(th)
            else:
                px, py = math.cos(th), math.sin(th)
            verts.append(path[i] + Ns[i] * (px * rx) + B * (py * ry))
            uvs.append((j / nseg, i / max(1, n - 1)))
    faces = []
    for i in range(n - 1):
        for j in range(nseg):
            a = i * nseg + j; b = i * nseg + (j + 1) % nseg
            c = (i + 1) * nseg + (j + 1) % nseg; d = (i + 1) * nseg + j
            faces.append((a, d, c, b))
    if cap:
        c0 = len(verts); verts.append(path[0]); uvs.append((0.5, 0))
        c1 = len(verts); verts.append(path[-1]); uvs.append((0.5, 1))
        for j in range(nseg):
            faces.append((c0, j, (j + 1) % nseg))
            faces.append((c1, (n - 1) * nseg + (j + 1) % nseg, (n - 1) * nseg + j))
    return verts, faces, uvs


def sweep_obj(name, path, radius, m='carbon', nseg=16, **kw):
    v, f, uv = sweep(path, radius, nseg, **kw)
    o = mesh_obj(name, v, f, uvs=uv, mat_names=(m,))
    return o


def rbox(name, size, center=(0, 0, 0), R=None, m='graphite', bev=0.0015, seg=3, harden=True):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    sx_, sy_, sz_ = size
    for v in bm.verts:
        v.co = V((v.co.x * sx_, v.co.y * sy_, v.co.z * sz_))
    R = M.Identity(3) if R is None else R
    for v in bm.verts:
        v.co = V(center) + R @ v.co
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me); bm.free()
    verts = [v.co[:] for v in me.vertices]
    faces = [p.vertices[:] for p in me.polygons]
    bpy.data.meshes.remove(me)
    o = mesh_obj(name, verts, faces, mat_names=(m,), smooth=True)
    if bev > 0:
        bevel(o, bev, seg=seg, angle=30, harden=harden)
    else:
        o.data.set_sharp_from_angle(angle=math.radians(40))
    return o


def plate(name, outer, holes=(), thick=0.002, m='carbon', center=(0, 0, 0), R=None, bev=0.0004, seg=2):
    """2D outline (XY, list of (x,y)) with holes -> extruded plate along local Z, centered."""
    cu = bpy.data.curves.new(name + '_cu', 'CURVE')
    cu.dimensions = '2D'; cu.fill_mode = 'BOTH'
    cu.extrude = thick / 2
    for loop in [outer] + list(holes):
        sp = cu.splines.new('POLY')
        sp.points.add(len(loop) - 1)
        for i, (x, y) in enumerate(loop):
            sp.points[i].co = (x, y, 0, 1)
        sp.use_cyclic_u = True
    co = bpy.data.objects.new(name + '_tmp', cu)
    COLL.objects.link(co)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(co.evaluated_get(dg))
    bpy.data.objects.remove(co); bpy.data.curves.remove(cu)
    R = M.Identity(3) if R is None else R
    c = V(center)
    verts = [c + R @ v.co for v in me.vertices]
    faces = [p.vertices[:] for p in me.polygons]
    bpy.data.meshes.remove(me)
    o = mesh_obj(name, verts, faces, mat_names=(m,), smooth=True)
    weld(o, 1e-7)
    if bev > 0:
        bevel(o, bev, seg=seg, angle=30, harden=True)
    else:
        o.data.set_sharp_from_angle(angle=math.radians(40))
    return o


def circle_pts(r, n=48, cx=0.0, cy=0.0, a0=0.0, a1=TAU, closed=True):
    k = n if closed else n + 1
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(k)]


def rounded_rect(w, h, r, n=6, cx=0.0, cy=0.0):
    pts = []
    for (qx, qy, a0) in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, math.pi / 2),
                         (-w / 2 + r, -h / 2 + r, math.pi), (w / 2 - r, -h / 2 + r, 1.5 * math.pi)):
        for i in range(n + 1):
            a = a0 + (math.pi / 2) * i / n
            pts.append((cx + qx + r * math.cos(a), cy + qy + r * math.sin(a)))
    return pts


def gear_outline(r_pitch, n_teeth, h=0.0015, a0=0.0, a1=TAU, r_hub=None, samples=6):
    """Involute-ish gear outline (trapezoid teeth). Partial sector if a1-a0 < TAU (closed via hub)."""
    pts = []
    full = abs(a1 - a0 - TAU) < 1e-9
    pitch_ang = TAU / n_teeth
    nt = int(round((a1 - a0) / pitch_ang))
    ra = r_pitch + h * 0.5; rd = r_pitch - h * 0.6
    for k in range(nt):
        base = a0 + k * pitch_ang
        prof = [(0.00, rd), (0.18, rd), (0.32, ra), (0.50, ra), (0.64, rd), (1.0, rd)]
        for (f, r) in prof:
            a = base + f * pitch_ang
            pts.append((r * math.cos(a), r * math.sin(a)))
    if not full:
        rh = r_hub if r_hub else r_pitch * 0.35
        pts.append((rd * math.cos(a1), rd * math.sin(a1)))
        for i in range(samples + 1):
            a = a1 + (a0 - a1) * i / samples
            pts.append((rh * math.cos(a), rh * math.sin(a)))
    return pts


def helix(center, axis_R, r, pitch, turns, n_per=24):
    pts = []
    N = int(turns * n_per)
    for i in range(N + 1):
        a = TAU * i / n_per
        p = V((r * math.cos(a), r * math.sin(a), pitch * a / TAU))
        pts.append(V(center) + axis_R @ p)
    return pts
