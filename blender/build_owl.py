"""STRIX-01 — builds the complete hierarchical owl-robot assembly and exports GLB.

Run:  python3 build_owl.py -- <out.glb> <report.json>
"""
import sys, os, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
import numpy as np
from mathutils import Vector as V, Matrix as M
from lib import *
import lib
import parts_body as PB
import parts_head as PH
import parts_wing as PW
import parts_tail_legs as PT

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
OUT = argv[0] if argv else 'owl.glb'
REPORT = argv[1] if len(argv) > 1 else 'build_report.json'

t0 = time.time()
init_scene()


def E(name, origin=(0, 0, 0), R=None, parent=None):
    o = empty(name, origin, R)
    if parent:
        link_parent(name, parent)
    return o


def P(obj, parent):
    link_parent(obj.name if not isinstance(obj, str) else obj, parent)


# ------------------------------------------------------------------ root
E('OWL')
# ------------------------------------------------------------------ core
E('A_core', (0, 0, 0), parent='OWL')
for o in [PB.build_frame(), PB.build_battery(), PB.build_fc(), PB.build_vision(), PB.build_pdu()] + PB.build_shells():
    P(o, 'A_core')
# surveillance payload: the cameras are the head's own eyes; only the comms antenna is a separate part
P(PB.build_antenna(), 'A_core')
for s in (1, -1):
    with side(s):
        sc, hinge = PB.build_scapular()
        E(S('J_scapular_{s}'), hinge, parent='A_core')
        P(sc, S('J_scapular_{s}'))
print('core', time.time() - t0)

# ------------------------------------------------------------------ head
H = PH.H
E('A_head', H + V((0, 0.02, -0.02)), parent='OWL')
P(PH.build_neck(), 'A_head')
E('J_neckPitch', H, parent='A_head')
P(PH.build_cradle(), 'J_neckPitch')
E('J_neckYaw', H, parent='J_neckPitch')
E('A_headUnit', H, parent='J_neckYaw')
crown = PH.build_crown(); P(crown, 'A_headUnit')
E_left = None
for s in (1, -1):
    with side(s):
        face = PH.build_face(); P(face, 'A_headUnit')
        part, blades, lids, Ec_eye, Ec = PH.build_optic()
        if s > 0:
            E_left = Ec_eye
        jn = S('J_eye_{s}')
        E(jn, Ec_eye, parent='A_headUnit')
        P(part, jn)
        for b in blades:
            P(b, part.name)
        for nm, lid in zip(('U', 'L'), lids):
            jl = S(f'J_lid{nm}_{{s}}')
            E(jl, Ec, parent=part.name)
            P(lid, jl)
P(PH.build_beak(), 'A_headUnit')
P(PH.build_head_frame(E_left), 'A_headUnit')
print('head', time.time() - t0)

# ------------------------------------------------------------------ wings
for s in (1, -1):
    with side(s):
        aw = S('A_wing_{s}')
        E(aw, PW.AXLE, parent='OWL')
        P(PW.build_root_motor(), aw)
        P(PW.build_capstan(), aw)
        js = S('J_shoulder_{s}')
        E(js, PW.AXLE, parent=aw)
        P(PW.build_shoulder_hub(), js)
        P(PW.build_sector_drum(), js)
        P(PW.build_torsion_spring(), js)
        P(PW.build_pitch_servo(), js)
        jsw = S('J_sweep_{s}')
        E(jsw, PW.SP, None, parent=js)
        jp = S('J_armPitch_{s}')
        E(jp, PW.SP, PW.ARM_R, parent=jsw)
        aa = S('A_arm_{s}')
        E(aa, PW.SP, PW.ARM_R, parent=jp)
        P(PW.build_humerus(), aa)
        P(PW.build_hum_fairing(), aa)
        P(PW.build_fold_actuator(), aa)
        P(PW.build_membrane(), aa)
        for f in PW.build_tertials():
            P(f, aa)
        P(PW.build_elbow_cover(), aa)
        je = S('J_elbow_{s}')
        E(je, PW.ELB, PW.ARM_R, parent=aa)
        af = S('A_forearm_{s}')
        E(af, PW.ELB, PW.ARM_R, parent=je)
        P(PW.build_spar_primary(), af)
        P(PW.build_le_fairing(), af)
        rod_start = PW.SP + PW.ARM_R @ (V((PW.ELB_X + 0.016, 0, 0)) + PW.FOLD_ROD_OFF)
        jr = S('J_foldRod_{s}')
        E(jr, rod_start, PW.ARM_R, parent=af)
        P(PW.build_fold_rod(), jr)
        asec = S('A_secondaries_{s}')
        E(asec, PW.ELB + PW.ARM_R @ V((0.06, 0.08, 0)), PW.ARM_R, parent=af)
        for f in PW.build_secondaries():
            P(f, asec)
        jw = S('J_wrist_{s}')
        E(jw, PW.W, PW.ARM_R, parent=af)
        P(PW.build_wrist_joint(), jw)
        P(PW.build_carpal_cover(), af)
        jt = S('J_handTwist_{s}')
        E(jt, PW.W, PW.HAND_R, parent=jw)
        ah = S('A_hand_{s}')
        E(ah, PW.W, PW.HAND_R, parent=jt)
        P(PW.build_spar_secondary(), ah)
        P(PW.build_hand_fairing(), ah)
        al, piv = PW.build_alula()
        ja = S('J_alula_{s}')
        E(ja, piv, PW.HAND_R, parent=ah)
        P(al, ja)
        apr = S('A_primaries_{s}')
        E(apr, PW.W + PW.HAND_R @ V((0.06, 0.10, 0)), PW.HAND_R, parent=ah)
        for f in PW.build_primaries():
            P(f, apr)
print('wings', time.time() - t0)

# ------------------------------------------------------------------ tail
E('A_tail', PT.TAIL_PIV, parent='OWL')
P(PT.build_tail_mount(), 'A_tail')
E('J_tailPitch', PT.TAIL_PIV, parent='A_tail')
E('J_tailTilt', PT.TAIL_PIV, parent='J_tailPitch')
P(PT.build_tail_actuator(), 'J_tailTilt')
E('A_tailFan', PT.TAIL_PIV + V((0, 0.06, 0)), parent='J_tailTilt')
for f in PT.build_rectrices():
    P(f, 'A_tailFan')
print('tail', time.time() - t0)

# ------------------------------------------------------------------ landing gear
E('A_landing', (0, -0.002, -0.07), parent='OWL')
for s in (1, -1):
    with side(s):
        al = S('A_leg_{s}')
        E(al, PT.HIP, parent='A_landing')
        P(PT.build_hip_mount(), al)
        jh = S('J_hip_{s}')
        E(jh, PT.HIP, parent=al)
        P(PT.build_leg_upper(), jh)
        ja = S('J_ankle_{s}')
        E(ja, PT.ANKLE, parent=jh)
        P(PT.build_tarsus(), ja)
        foot = PT.build_foot()
        P(foot, ja)
print('legs', time.time() - t0)

# ------------------------------------------------------------------ perch & templates
PT.build_perch()
PT.build_screw_template()

apply_hierarchy()
bpy.context.view_layer.update()

# ------------------------------------------------------------------ body scale
# The fuselage, head, tail and legs are scaled up about the origin (owl proportions: a body that
# is large relative to the wing). The wing roots move out with the body; the wing structure itself
# (everything below J_sweep) keeps its size through a counter-scale on the sweep joint.
BS = 1.18
for n in ('A_core', 'A_head', 'A_tail', 'A_landing', 'A_wing_L', 'A_wing_R'):
    o = bpy.data.objects[n]
    o.location = o.location * BS
    o.scale = (BS, BS, BS)
for n in ('J_sweep_L', 'J_sweep_R'):
    bpy.data.objects[n].scale = (1 / BS, 1 / BS, 1 / BS)
bpy.context.view_layer.update()

# ------------------------------------------------------------------ report: planform area, span, CG estimate
def planform():
    """Rasterise the top-down projection of all wing + body surfaces (union area)."""
    res = 0.0015
    xs = np.arange(-0.90, 0.90, res); ys = np.arange(-0.30, 0.40, res)
    grid = np.zeros((len(ys), len(xs)), dtype=bool)
    dg = bpy.context.evaluated_depsgraph_get()
    span = 0.0
    for o in bpy.data.objects:
        if o.type != 'MESH':
            continue
        n = o.name
        if WING_ONLY and n.startswith(('P_shell', 'P_flank')):
            continue
        if not (n.startswith(('P_pri', 'P_sec', 'P_ter', 'P_leFairing', 'P_handFairing', 'P_coverts', 'P_primaryCoverts', 'P_membrane', 'P_shell', 'P_flank', 'P_alula'))):
            continue
        me = o.evaluated_get(dg).to_mesh()
        me.calc_loop_triangles()
        mw = o.matrix_world
        co = np.array([(mw @ v.co)[:2] for v in me.vertices])
        span = max(span, np.abs(co[:, 0]).max())
        for tri in me.loop_triangles:
            a, b, c = co[list(tri.vertices)]
            x0, x1 = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
            y0, y1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
            i0 = max(0, int((x0 + 0.90) / res)); i1 = min(len(xs) - 1, int((x1 + 0.90) / res) + 1)
            j0 = max(0, int((y0 + 0.30) / res)); j1 = min(len(ys) - 1, int((y1 + 0.30) / res) + 1)
            if i1 <= i0 or j1 <= j0:
                continue
            X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-14:
                continue
            l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
            l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
            l3 = 1 - l1 - l2
            grid[j0:j1, i0:i1] |= (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
        o.evaluated_get(dg).to_mesh_clear()
    return float(grid.sum() * res * res), float(span * 2)


WING_ONLY = False
area, span = planform()
WING_ONLY = True
wing_area, _ = planform()

# mass budget (grams) assigned to modelled components (concept-level estimate, not weighed)
MASS = {
    'P_frame': 200, 'P_battery': 220, 'P_fc': 16, 'P_vision': 26, 'P_pdu': 34,
    'P_shell_front': 44, 'P_shell_rear': 52, 'P_flank_L': 18, 'P_flank_R': 18, 'P_ventral': 18,
    'P_scapular_L': 9, 'P_scapular_R': 9,
    'P_neck': 20, 'P_neckCradle': 22, 'P_headFrame': 22, 'P_crown': 30, 'P_face_L': 14, 'P_face_R': 14,
    'P_optic_L': 18, 'P_optic_R': 18, 'P_beak': 12,
    'P_tailMount': 12, 'P_tailActuator': 24, 'P_rect': 2.0,
    'P_antenna': 8,
}
for s in 'LR':
    MASS.update({f'P_rootMotor_{s}': 150, f'P_capstan_{s}': 28, f'P_shoulderHub_{s}': 40, f'P_sectorDrum_{s}': 14,
                 f'P_torsionSpring_{s}': 62, f'P_pitchServo_{s}': 30, f'P_humerus_{s}': 15, f'P_humFairing_{s}': 9,
                 f'P_foldActuator_{s}': 35, f'P_foldRod_{s}': 3, f'P_membrane_{s}': 3, f'P_sparPrimary_{s}': 22,
                 f'P_leFairing_{s}': 12, f'P_wristJoint_{s}': 7, f'P_carpalCover_{s}': 3, f'P_elbowCover_{s}': 3, f'P_sparSecondary_{s}': 14, f'P_handFairing_{s}': 10,
                 f'P_alula_{s}': 2, f'P_pri_{s}': 4.2, f'P_sec_{s}': 4.0, f'P_ter_{s}': 2.8,
                 f'P_hipMount_{s}': 40, f'P_legUpper_{s}': 22, f'P_tarsus_{s}': 12, f'P_foot_{s}': 18})
MASS['wiring_misc'] = 90
tot = 0.0; mom = V((0, 0, 0)); rows = []
for o in bpy.data.objects:
    n = o.name
    key = None
    if n in MASS:
        key = n
    else:
        for pre in ('P_rect', 'P_pri_L', 'P_pri_R', 'P_sec_L', 'P_sec_R', 'P_ter_L', 'P_ter_R'):
            if n.startswith(pre + '_'):
                key = pre
    if key is None:
        continue
    m = MASS[key]
    if o.type == 'MESH':
        dg = bpy.context.evaluated_depsgraph_get()
        me = o.data
        co = np.array([(o.matrix_world @ v.co)[:] for v in me.vertices])
        c = V(co.mean(axis=0))
    else:
        c = o.matrix_world.translation
    tot += m; mom += c * m
    rows.append((n, m))
groups = {}
for n, m in rows:
    g = ('wings' if any(k in n for k in ('_pri_', '_sec_', '_ter_', 'Fairing', 'humerus', 'spar', 'fold', 'membrane', 'alula', 'wrist', 'carpal', 'elbowCover'))
         else 'wing drive' if any(k in n for k in ('rootMotor', 'capstan', 'shoulderHub', 'sectorDrum', 'torsionSpring', 'pitchServo'))
         else 'head' if any(k in n for k in ('neck', 'head', 'crown', 'face', 'optic', 'beak'))
         else 'shells' if any(k in n for k in ('shell', 'flank', 'ventral', 'scapular'))
         else 'landing gear' if any(k in n for k in ('hip', 'leg', 'tarsus', 'foot'))
         else 'tail' if ('tail' in n or 'rect' in n)
         else 'payload' if 'antenna' in n else 'core')
    groups[g] = groups.get(g, 0) + m
groups['wiring & misc'] = MASS['wiring_misc']
MOVING = ('P_shoulderHub_L', 'P_sectorDrum_L', 'P_pitchServo_L', 'P_humerus_L', 'P_humFairing_L', 'P_foldActuator_L', 'P_foldRod_L',
          'P_membrane_L', 'P_sparPrimary_L', 'P_leFairing_L', 'P_wristJoint_L', 'P_carpalCover_L', 'P_elbowCover_L',
          'P_sparSecondary_L', 'P_handFairing_L', 'P_alula_L', 'P_pri_L_', 'P_sec_L_', 'P_ter_L_')
I_stroke = 0.0; m_moving = 0.0; I_feather = 0.0; DBG_I = []
for o in bpy.data.objects:
    n = o.name
    if o.type != 'MESH' or not n.startswith(MOVING):
        continue
    key = n if n in MASS else next((k for k in ('P_pri_L', 'P_sec_L', 'P_ter_L') if n.startswith(k + '_')), None)
    if key is None:
        continue
    m = MASS[key] / 1000.0
    co = np.array([(o.matrix_world @ v.co)[:] for v in o.data.vertices])
    r2 = (co[:, 0] - PW.AXLE.x * BS) ** 2 + (co[:, 2] - PW.AXLE.z * BS) ** 2   # distance from the stroke axis (along Y)
    I_stroke += m * float(r2.mean()); m_moving += m
    DBG_I.append((n, round(m * 1000, 1), round(float(np.sqrt(r2.mean())), 3), round(m * float(r2.mean()) * 1e3, 3)))
    if n.startswith(('P_pri', 'P_sec', 'P_ter')):
        I_feather += m * float(r2.mean())
tot += MASS['wiring_misc']; mom += V((0, -0.02, 0.0)) * MASS['wiring_misc']
cg = mom / tot
mass_kg = tot / 1000
WL = mass_kg * 9.81 / area
report = {
    'mass_g': round(tot, 1), 'cg_blender': [round(c, 4) for c in cg], 'planform_area_m2': round(area, 4), 'span_m': round(span, 3),
    'wing_loading_N_m2': round(WL, 1), 'aspect_ratio': round(span * span / area, 2),
    'wing_only_area_m2': round(wing_area, 4), 'wing_loading_wing_only_N_m2': round(mass_kg * 9.81 / wing_area, 1),
    'cg_below_stroke_axis_mm': round((PW.AXLE.z * BS - cg.z) * 1000, 1), 'cg_behind_stroke_axis_mm': round((cg.y - PW.AXLE.y * BS) * 1000, 1),
    'shoulder_axis_blender': [round(c * BS, 4) for c in PW.AXLE],
    'body_scale': BS,
    'perch_bar_height_m': PT.PERCH_Z,
    'wing_stroke_inertia_kgm2': round(I_stroke, 5), 'wing_moving_mass_g': round(m_moving * 1000, 1),
    'wing_stroke_radius_of_gyration_m': round(math.sqrt(I_stroke / m_moving), 3), 'feather_stroke_inertia_kgm2': round(I_feather, 5),
    'mass_groups_g': {k: round(v, 1) for k, v in sorted(groups.items(), key=lambda kv: -kv[1])},
    'objects': len(bpy.data.objects), 'build_s': round(time.time() - t0, 1),
}
# normal audit: every closed shell island must face outward (inverted islands render inside-out
# with back-face culling). Islands with negative signed volume are flipped.
import bmesh
flipped = 0
for o in bpy.data.objects:
    if o.type != 'MESH':
        continue
    bm = bmesh.new(); bm.from_mesh(o.data)
    bm.faces.ensure_lookup_table()
    seen = set(); changed = False
    for f0 in bm.faces:
        if f0.index in seen:
            continue
        isl = []; stack = [f0]; seen.add(f0.index)
        while stack:
            f = stack.pop(); isl.append(f)
            for e in f.edges:
                for g in e.link_faces:
                    if g.index not in seen:
                        seen.add(g.index); stack.append(g)
        closed = all(len(e.link_faces) == 2 for f in isl for e in f.edges)
        if not closed:
            continue
        vol = 0.0
        for f in isl:
            vs = [v.co for v in f.verts]
            for i in range(1, len(vs) - 1):
                vol += vs[0].dot(vs[i].cross(vs[i + 1])) / 6.0
        if vol < -1e-12:
            bmesh.ops.reverse_faces(bm, faces=isl)
            flipped += 1; changed = True
    if changed:
        bm.to_mesh(o.data)
    bm.free()
report['inverted_islands_fixed'] = flipped
print('normal audit: flipped', flipped, 'inverted closed islands')

# triangle count
tris = 0
for o in bpy.data.objects:
    if o.type == 'MESH':
        o.data.calc_loop_triangles(); tris += len(o.data.loop_triangles)
report['triangles'] = tris
for row in sorted(DBG_I, key=lambda r: -r[3])[:14]:
    print('I-part', row)
print(json.dumps(report, indent=1))
json.dump(report, open(REPORT, 'w'), indent=1)

bpy.ops.export_scene.gltf(filepath=OUT, export_format='GLB', export_apply=True, export_yup=True,
                          export_texcoords=True, export_normals=True, export_materials='EXPORT',
                          export_cameras=False, export_lights=False, export_extras=False)
print('exported', OUT, os.path.getsize(OUT) / 1e6, 'MB', time.time() - t0, 's')
