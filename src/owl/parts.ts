/**
 * Part catalogue: human-readable metadata and the engineering exploded-view layout.
 *
 * Coordinates are owl space (glTF): +X = owl's left, +Y = up, +Z = forward. Metres.
 * The layout explodes along the airframe's principal axes:
 *   core  -> vertical stack (Y)      head -> forward along the neck (+Z)
 *   wings -> outward along the span  tail -> aft (−Z)      legs -> down along the leg axis
 *
 *  d   — absolute displacement of the node origin at full explode (composes with ancestors)
 *  dl  — extra displacement in the node's own local frame (feathers slide out along the rachis)
 *  rot — small purposeful rotation at full explode (degrees, local XYZ)
 *  release — short "unlatch" motion along the assembly axis, applied before the main move
 *  detach / assemble — start delay (s) inside each sequence; the groups follow the brief's order
 *  lead — draw a trace line back to the attachment point
 */
export type V3 = [number, number, number]

export interface ExplodeSpec {
  d?: V3
  dl?: V3
  rel?: V3
  rot?: V3
  release?: V3
  detach: number
  assemble: number
  dur?: number
  lead?: boolean
}

export interface PartInfo {
  title: string
  fn: string
  group: string
}

const R = (v: V3): V3 => [-v[0], v[1], v[2]]

export const EXPLODE: Record<string, ExplodeSpec> = {}
export const INFO: Record<string, PartInfo> = {}

function both(name: string, spec: ExplodeSpec | null, info?: PartInfo) {
  for (const s of ['L', 'R'] as const) {
    const n = name.replace('{s}', s)
    if (spec) {
      EXPLODE[n] = {
        ...spec,
        d: spec.d ? (s === 'L' ? spec.d : R(spec.d)) : undefined,
        rel: spec.rel ? (s === 'L' ? spec.rel : R(spec.rel)) : undefined,
        release: spec.release ? (s === 'L' ? spec.release : R(spec.release)) : undefined,
        rot: spec.rot ? (s === 'L' ? spec.rot : [spec.rot[0], -spec.rot[1], -spec.rot[2]]) : undefined,
      }
    }
    if (info) INFO[n] = { ...info, title: info.title.replace('{side}', s === 'L' ? 'Left' : 'Right') }
  }
}

function one(name: string, spec: ExplodeSpec | null, info?: PartInfo) {
  if (spec) EXPLODE[name] = spec
  if (info) INFO[name] = info
}

// ----------------------------------------------------------------- core (vertical stack)
one('A_core', null, { title: 'Core Assembly', fn: 'Load-bearing fuselage: frame, energy storage and avionics.', group: 'Core' })
one('P_frame', null, {
  title: 'Central Frame & Shoulder Girdle', group: 'Core',
  fn: 'Truss-cut carbon keel plus a box girdle of three CFRP bulkheads with bonded titanium bearing inserts, carrying both stroke axles and capstan shafts on bearing pairs. Wing root torque goes straight into this box.',
})
one('P_shell_rear', { d: [0, 0.2, -0.01], release: [0, 0.012, 0], detach: 0.0, assemble: 2.55, lead: true }, {
  title: 'Rear Fuselage Shell', group: 'Core',
  fn: 'Dorsal shell of three overlapping chevron plates. Narrow slots at each shoulder clear the yoke through its stroke.',
})
both('P_flank_{s}', { d: [0.17, 0.0, -0.01], release: [0.012, 0, 0], detach: 0.1, assemble: 2.45, lead: true }, {
  title: '{side} Flank Panel', group: 'Core', fn: 'Side panel with a sculpted shoulder cowl over the stroke axle and torsion spring.',
})
both('P_scapular_{s}', { d: [0.2, 0.05, 0.03], release: [0.01, 0, 0.004], rot: [0, 0, 72], detach: 0.06, assemble: 2.5 }, {
  title: '{side} Scapular Stow Cover', group: 'Core',
  fn: 'Feathered shoulder plate. It swings in under the dorsal shell for flight and drops over the folded wing root when the wing is stowed, like the scapular feathers of a perched owl.',
})
one('P_shell_front', { d: [0, -0.2, 0.06], release: [0, -0.01, 0.006], detach: 0.18, assemble: 2.35, lead: true }, {
  title: 'Front Fuselage Shell', group: 'Core',
  fn: 'Breast shell of three overlapping chevron plates (contour-feather analogue). Protects the battery bay.',
})
one('P_ventral', { d: [0, -0.18, -0.09], release: [0, -0.01, 0], detach: 0.24, assemble: 2.4, lead: true }, {
  title: 'Ventral Hatch', group: 'Core', fn: 'Belly hatch with the leg slots and the cooling-air exhaust louvres. Air enters through the annular gap around the neck gimbal, passes the motors, torsion springs and power electronics, and leaves here.',
})
one('P_battery', { d: [0, -0.11, 0.02], detach: 1.95, assemble: 0.15, lead: true }, {
  title: 'Battery Pack', group: 'Core',
  fn: '6S 1300 mAh, 45C LiPo (≈29 Wh, assumed) with XT30 main lead and balance lead. Sized for the take-off current bursts; low and forward so the centre of mass sits below and just behind the stroke axes.',
})
one('P_fc', { d: [0, 0.105, -0.01], detach: 2.05, assemble: 0.1, lead: true }, {
  title: 'Flight Controller', group: 'Core',
  fn: 'Two-board stack: flight-control MCU, barometer and actuator bus. Closes the wingbeat, attitude and gait loops.',
})
one('P_vision', { d: [0, 0.0, 0.13], detach: 2.1, assemble: 0.2, lead: true }, {
  title: 'Sensor Processing Module', group: 'Core',
  fn: 'Vision/acoustic compute with a vibration-isolated IMU near the centre of mass; neck encoders align the head cameras to it.',
})
one('P_pdu', { d: [0, -0.07, -0.08], detach: 2.15, assemble: 0.25, lead: true }, {
  title: 'Power Distribution', group: 'Core',
  fn: 'Copper bus bars, bulk capacitors and the servo/linear-actuator drivers. The wing motors carry their own FOC drives.',
})

// ----------------------------------------------------------------- surveillance payload
one('P_sensorPod', { dl: [0, -0.2, 0.02], detach: 1.95, assemble: 0.28, dur: 1.0, lead: true }, {
  title: 'Ventral Sensor Turret', group: 'Surveillance',
  fn: 'Two-axis gimballed day (EO) and thermal (IR) camera under the forward belly, with onboard recording. The head cameras are for navigation; this turret is the dedicated surveillance payload — steered independently, stabilised against the wingbeat, and aimed forward and down.',
})
one('P_antenna', { dl: [0, 0.22, -0.02], detach: 1.95, assemble: 0.28, dur: 1.0, lead: true }, {
  title: 'Comms / Downlink Antenna', group: 'Surveillance',
  fn: 'Dorsal blade antenna for the command uplink and the video/telemetry downlink. The robot records and returns without it; with a link it streams a live or burst preview.',
})

// ----------------------------------------------------------------- head (forward along the neck)
one('A_head', { d: [0, 0.04, 0.16], detach: 0.45, assemble: 2.0 }, { title: 'Head Assembly', fn: 'Sensor head on a two-axis neck gimbal.', group: 'Head' })
one('P_neck', { d: [0, 0.02, 0.07], detach: 0.5, assemble: 1.98, lead: true }, {
  title: 'Neck Articulation Assembly', group: 'Head',
  fn: 'Carbon neck post and titanium pitch fork. The gimbal centre sits at the head’s centre so the head can turn without changing its silhouette.',
})
one('P_neckCradle', { d: [0, 0.05, 0.21], detach: 0.55, assemble: 1.96 }, {
  title: 'Neck Pitch Cradle & Yaw Drive', group: 'Head', fn: 'Pitch trunnion cradle carrying the pancake yaw motor, bearing ring and encoders.',
})
one('A_headUnit', { d: [0, 0.06, 0.28], detach: 0.6, assemble: 1.9 }, { title: 'Sensor Head', fn: 'Rotating sensor head.', group: 'Head' })
one('P_headFrame', null, {
  title: 'Head Structural Frame', group: 'Head',
  fn: 'Titanium cage: facial perimeter ring, optic mounts, midline keel and asymmetric acoustic pods.',
})
one('P_crown', { d: [0, 0.16, 0.25], release: [0, 0.008, -0.004], detach: 0.75, assemble: 2.05, lead: true }, {
  title: 'Cranial Shell', group: 'Head', fn: 'Superellipsoidal satin shell closing the back of the head; pad-printed streak markings.',
})
both('P_face_{s}', { d: [0.055, 0.06, 0.36], release: [0.004, 0, 0.008], detach: 0.8, assemble: 2.0, lead: true }, {
  title: '{side} Facial Shell', group: 'Head',
  fn: 'Concave facial disc of radiating composite blades. Like an owl’s ruff, the dish also shapes the acoustic aperture of the microphone array.',
})
both('P_optic_{s}', { d: [0.03, 0.065, 0.46], detach: 0.9, assemble: 1.95, lead: true }, {
  title: '{side} Optical Assembly', group: 'Head',
  fn: 'Forward-facing camera: domed front element, two internal lens groups, 9-blade iris and protective shutter lids.',
})
one('P_beak', { d: [0, 0.02, 0.42], detach: 0.85, assemble: 1.95, lead: true }, {
  title: 'Beak Sensor Housing', group: 'Head', fn: 'Hooked ceramic housing for a short-range time-of-flight sensor used for landing.',
})

// ----------------------------------------------------------------- wings (outward along the span)
both('A_wing_{s}', { d: [0.06, 0, 0], detach: 1.1, assemble: 0.6 }, { title: '{side} Wing Assembly', fn: 'Independently driven flapping wing.', group: 'Wing' })
both('P_rootMotor_{s}', { d: [0.06, 0, -0.1], release: [0, 0, -0.006], detach: 1.55, assemble: 0.35, lead: true }, {
  title: '{side} Wing Root Actuator', group: 'Wing',
  fn: 'Provides the primary oscillating motion that drives the wing flapping mechanism: a 58 mm low-Kv pancake motor (≈1.3 N·m peak) with integrated FOC drive and absolute encoder, reversing every half-stroke.',
})
both('P_capstan_{s}', { d: [0.06, 0, -0.055], detach: 1.5, assemble: 0.4, lead: true }, {
  title: '{side} Capstan Cable Transmission', group: 'Wing',
  fn: 'Two-stage reduction (≈13.8:1): a 3:1 toothed belt from the motor, then a zero-backlash 4.6:1 capstan stage — Dyneema cable on a 13 mm capstan driving the 60 mm sector drum. The capstan shaft runs in its own bearing pair in the girdle.',
})
both('P_shoulderHub_{s}', { d: [0.12, 0, 0.0], detach: 1.45, assemble: 0.5, lead: true }, {
  title: '{side} Shoulder Hub & Stroke Axle', group: 'Wing',
  fn: 'Stroke axle on a bearing pair in the girdle, hub and yoke, sweep clevis with its worm-geared stow/trim servo, axle encoder, and the stroke brake: a spring-applied, solenoid-released pin that drops into detents in the drum at the glide (+5°) and stow (−90°) positions.',
})
both('P_sectorDrum_{s}', { d: [0.1, 0.0, -0.035], detach: 1.48, assemble: 0.45, lead: true }, {
  title: '{side} Sector Drum', group: 'Wing', fn: 'Asymmetric 30 mm-radius sector on the stroke axle, long enough for the full flapping range and the −90° stow, with the cable wrap and terminations.',
})
both('P_torsionSpring_{s}', { d: [0.115, 0, 0.05], detach: 1.5, assemble: 0.48, lead: true }, {
  title: '{side} Resonant Torsion Spring', group: 'Wing',
  fn: 'Elastomer torsion spring (five bonded natural-rubber shear rings in series, ≈12 N·m/rad) tuned to the wing + reflected rotor inertia near the flapping frequency. A latching solenoid dog clutch, shifted only at stroke neutral, declutches it for stowing.',
})
both('P_pitchServo_{s}', { d: [0.13, 0.045, 0.0], detach: 1.52, assemble: 0.46, lead: true }, {
  title: '{side} Pitch Servo', group: 'Wing', fn: 'High-speed servo (≤0.05 s/60°, sized for ≈1.2–1.5 N·m peak with a preloaded pitch spring carrying the mean aerodynamic moment) setting arm pitch (±8° in cruise) through the stroke; the hand follows at 2:1 through the wrist tendons.',
})
both('A_arm_{s}', { d: [0.17, 0, 0], detach: 1.15, assemble: 0.75 }, { title: '{side} Humeral Section', fn: 'Inner wing between shoulder and elbow.', group: 'Wing' })
both('P_humerus_{s}', null, {
  title: '{side} Humeral Spar', group: 'Wing', fn: 'Short carbon spar from the sweep hinge to the elbow clevis; carries the pitch bearing, the fold actuator, the elbow extension stop and its preloaded return spring.',
})
both('P_humFairing_{s}', { d: [0.17, 0.02, 0.065], release: [0, 0, 0.005], detach: 1.45, assemble: 1.25, lead: true }, {
  title: '{side} Humeral Fairing', group: 'Wing', fn: 'Leading-edge fairing over the humeral spar and fold actuator.',
})
both('P_foldActuator_{s}', { d: [0.17, -0.012, 0.04], detach: 1.5, assemble: 1.2, lead: true }, {
  title: '{side} Wing-Fold Actuator', group: 'Wing',
  fn: 'High-speed ball-screw actuator (≈60 N, ≈25 W peak): stows and deploys the wing, and actively flexes the hand by ≈0.3 of full fold on every upstroke (against the centrifugal load). The parallel link folds the wrist with the elbow; over-centre locks engage only for gliding and holding.',
})
both('P_membrane_{s}', { d: [0.15, -0.02, -0.06], detach: 1.55, assemble: 1.35, lead: true }, {
  title: '{side} Root Membrane', group: 'Wing', fn: 'Flexible elastomer membrane closing the gap to the fuselage; it pleats chordwise as the wing folds.',
})
both('A_forearm_{s}', { d: [0.23, 0, 0], detach: 1.18, assemble: 0.78 }, { title: '{side} Forearm Section', fn: 'Mid wing carrying the secondaries.', group: 'Wing' })
both('P_sparPrimary_{s}', null, {
  title: '{side} Primary Wing Spar (forearm)', group: 'Wing',
  fn: 'Main load-bearing spar from elbow to wrist: elbow bellcrank, over-centre lock and compliant secondary-feather sockets.',
})
both('P_leFairing_{s}', { d: [0.23, 0.02, 0.07], release: [0, 0, 0.006], detach: 1.45, assemble: 1.25, lead: true }, {
  title: '{side} Leading-Edge Fairing', group: 'Wing', fn: 'Thick rounded leading edge (owl-like) enclosing the spar and the parallel fold link.',
})
both('P_foldRod_{s}', { d: [0.23, -0.014, 0.05], detach: 1.52, assemble: 1.18, lead: true }, {
  title: '{side} Parallel Fold Link', group: 'Wing', fn: 'Rod inside the forearm fairing coupling elbow and wrist flexion — the robot’s “drawing parallels” mechanism.',
})
both('A_secondaries_{s}', { d: [0.23, -0.01, -0.05], detach: 1.6, assemble: 1.22 }, {
  title: '{side} Secondary Feather Group', group: 'Wing', fn: 'Eleven secondaries, each carrying its covert.',
})
both('P_wristJoint_{s}', { d: [0.285, 0, 0.03], detach: 1.5, assemble: 1.1, lead: true }, {
  title: '{side} Wrist Articulation', group: 'Wing', fn: 'Fold bearing, bellcrank and the 2:1 tendon pulley pair that slaves hand twist to arm pitch at any fold angle (no separate twist actuator).',
})
both('P_elbowCover_{s}', { d: [0.17, 0.055, 0.0], release: [0, 0.006, 0], detach: 1.42, assemble: 1.2, lead: true }, {
  title: '{side} Elbow Cover', group: 'Wing', fn: 'Rounded elbow pad, symmetric about the fold axis; hides the elbow clevis, extension stop and return spring. The amber pip is a joint-health indicator.',
})
both('P_carpalCover_{s}', { d: [0.285, 0.055, 0.03], release: [0, 0.006, 0], detach: 1.42, assemble: 1.2, lead: true }, {
  title: '{side} Carpal Cover', group: 'Wing', fn: 'Rounded wrist pad (carpal-covert analogue). It is symmetric about the fold axis, so it closes the gap between the forearm and hand fairings at any fold angle.',
})
both('A_hand_{s}', { d: [0.32, 0, 0], detach: 1.2, assemble: 0.8 }, { title: '{side} Hand Wing', fn: 'Outer wing.', group: 'Wing' })
both('P_sparSecondary_{s}', null, {
  title: '{side} Secondary Articulated Spar (hand)', group: 'Wing', fn: 'Hand-wing spar. Folds at the wrist and twists for active outer-wing pitch.',
})
both('P_handFairing_{s}', { d: [0.32, 0.02, 0.07], release: [0, 0, 0.005], detach: 1.5, assemble: 1.25, lead: true }, {
  title: '{side} Hand Fairing', group: 'Wing', fn: 'Leading-edge fairing of the hand wing.',
})
both('P_alula_{s}', { d: [0.33, 0.012, 0.1], detach: 1.55, assemble: 1.3, lead: true }, {
  title: '{side} Alula (Leading-Edge Slat)', group: 'Wing', fn: 'Spring-loaded slat; suction at high angle of attack lifts it during take-off and flare, as in birds.',
})
both('A_primaries_{s}', { d: [0.35, -0.01, -0.05], detach: 1.65, assemble: 1.22 }, {
  title: '{side} Primary Feather Group', group: 'Wing', fn: 'Ten primaries with their coverts. The outermost carries an owl-style leading-edge comb.',
})

for (const s of ['L', 'R']) {
  const side = s === 'L' ? 'Left' : 'Right'
  for (let j = 1; j <= 11; j++) {
    const n = `P_sec_${s}_${String(j).padStart(2, '0')}`
    EXPLODE[n] = { dl: [0.016, 0.0012 * (12 - j), 0], detach: 1.78 + 0.02 * j, assemble: 1.3 + 0.014 * (12 - j), dur: 0.8 }
    INFO[n] = {
      title: `${side} Secondary S${j}`, group: 'Wing',
      fn: 'Thin composite vane on a titanium rachis with its covert. A compliant root lets it twist open against a stop on the upstroke; fringed trailing edge.',
    }
  }
  for (let k = 1; k <= 3; k++) {
    const n = `P_ter_${s}_${String(k).padStart(2, '0')}`
    EXPLODE[n] = { dl: [0.016, 0.0015 * (k + 1), 0], detach: 1.78, assemble: 1.32, dur: 0.8 }
    INFO[n] = {
      title: `${side} Tertial T${k}`, group: 'Wing',
      fn: 'Inner feather on the humeral spar. Covers the wing root in flight and swings flat against the spar when the wing folds.',
    }
  }
  for (let k = 1; k <= 10; k++) {
    const n = `P_pri_${s}_${String(k).padStart(2, '0')}`
    EXPLODE[n] = { dl: [0.02, 0.0012 * k, 0], detach: 1.8 + 0.02 * k, assemble: 1.3 + 0.014 * k, dur: 0.8 }
    INFO[n] = {
      title: `${side} Primary P${k}`, group: 'Wing',
      fn: k === 10
        ? 'Outermost primary with a titanium leading-edge comb (serrations break up the leading-edge vortex to reduce noise, as studied on owls).'
        : 'Primary flight feather with its covert. Compliant root: aerodynamic load rotates it open on the upstroke and seals it on the downstroke.',
    }
  }
}

// ----------------------------------------------------------------- tail (aft)
one('A_tail', { d: [0, 0, -0.1], detach: 2.3, assemble: 1.6 }, { title: 'Tail Assembly', fn: 'Two-axis actuated tail fan.', group: 'Tail' })
one('P_tailMount', { d: [0, -0.015, -0.065], detach: 2.35, assemble: 1.6, lead: true }, {
  title: 'Tail Support Structure', group: 'Tail', fn: 'Carbon boom and titanium yoke carrying the tail pitch servo (the fast pitch effector, ≥10 Hz bandwidth) and its bearings.',
})
one('P_tailActuator', { d: [0, 0.02, -0.16], detach: 2.4, assemble: 1.65, lead: true }, {
  title: 'Tail Control Actuator', group: 'Tail',
  fn: 'Pitch trunnions, a tilt servo (tilting the fan about the body axis yaws the robot) and a fan-spread servo.',
})
one('A_tailFan', { d: [0, 0.02, -0.235], detach: 2.45, assemble: 1.6 }, { title: 'Tail Fan', fn: 'Twelve rectrices.', group: 'Tail' })
for (let k = 1; k <= 12; k++) {
  const n = `P_rect_${String(k).padStart(2, '0')}`
  EXPLODE[n] = { dl: [0.016, 0.0008 * (6 - Math.abs(k - 6.5)), 0], detach: 2.55 + 0.012 * k, assemble: 1.65 + 0.01 * k, dur: 0.7 }
  INFO[n] = { title: `Tail Feather R${k}`, group: 'Tail', fn: 'Rectrix panel. Spreads for low-speed control and tilts for directional trim.' }
}

// ----------------------------------------------------------------- landing gear (down the leg axis)
one('A_landing', { d: [0, -0.05, 0], detach: 2.3, assemble: 1.7 }, { title: 'Landing System', fn: 'Retractable legs and grasping feet.', group: 'Landing' })
both('A_leg_{s}', { d: [0.025, -0.05, 0], detach: 2.32, assemble: 1.72 }, { title: '{side} Leg Assembly', fn: 'Retractable leg.', group: 'Landing' })
both('P_hipMount_{s}', { d: [0.035, -0.065, 0], detach: 2.4, assemble: 1.76, lead: true }, {
  title: '{side} Hip Retraction Actuator', group: 'Landing', fn: 'Geared hip actuator with a latch that holds the tucked and extended positions unpowered. It also cocks a glass-fibre catapult spring (≈1.9 J) that pushes the robot off the perch at launch; sized for ~4 g perch landings.',
})
both('P_legUpper_{s}', { d: [0.04, -0.11, 0], detach: 2.45, assemble: 1.73, lead: true }, {
  title: '{side} Upper Leg', group: 'Landing', fn: 'Titanium tibia in two overlapping ceramic cowls, with the toe-tendon actuator on the back. The ankle is a passive spring joint.',
})
both('P_tarsus_{s}', { d: [0.045, -0.16, 0], detach: 2.5, assemble: 1.71, lead: true }, {
  title: '{side} Tarsus Strut', group: 'Landing', fn: 'Lower leg strut with a ceramic front shield and the flexor tendon.',
})
both('P_foot_{s}', { d: [0.05, -0.215, 0.01], detach: 2.55, assemble: 1.69, lead: true }, {
  title: '{side} Foot & Gripping Mechanism', group: 'Landing',
  fn: 'Four toes (2 forward, 2 back when perching) with ceramic talons. One tendon closes all toes around a perch; a ratchet holds the grip unpowered.',
})

// ----------------------------------------------------------------- exploded layout
/**
 * Engineering exploded view, laid out automatically from the parts' real bounding boxes.
 * Each stack moves its tiers along ONE principal axis, in order, so that every tier clears the
 * previous one by the same gap. Parts keep their assembled orientation (no rotations).
 *   core      : upper shell group up, internals and lower shell group down (Y)
 *   wings     : drive train, then each wing section outward along the span (X); within a section
 *               the fairing lifts up and the actuator / feather group drops down (Y)
 *   head      : forward along the neck (Z); crown up; optics, facial discs and beak forward
 *   tail      : aft (−Z)       legs: down their own axis (−Y)
 * `base` is the node whose (already exploded) box the first tier must clear.
 */
export interface Stack {
  base: string
  axis: 0 | 1 | 2
  sign: 1 | -1
  gap: number
  tiers: string[][]
}

const G = 0.024, g = 0.016
const side = (st: Stack[], s: 'L' | 'R'): Stack[] =>
  st.map((k) => ({
    ...k,
    base: k.base.replace('{s}', s),
    sign: (k.axis === 0 && s === 'R' ? -k.sign : k.sign) as 1 | -1,
    tiers: k.tiers.map((t) => t.map((n) => n.replace('{s}', s))),
  }))

const WING: Stack[] = [
  { base: 'P_frame', axis: 0, sign: 1, gap: G, tiers: [['P_capstan_{s}'], ['P_shoulderHub_{s}'], ['A_arm_{s}'], ['A_forearm_{s}'], ['P_wristJoint_{s}'], ['A_hand_{s}']] },
  { base: 'P_capstan_{s}', axis: 1, sign: -1, gap: g, tiers: [['P_rootMotor_{s}']] },
  { base: 'P_shoulderHub_{s}', axis: 1, sign: 1, gap: g, tiers: [['P_pitchServo_{s}']] },
  { base: 'P_shoulderHub_{s}', axis: 1, sign: -1, gap: g, tiers: [['P_sectorDrum_{s}'], ['P_torsionSpring_{s}']] },
  { base: 'A_arm_{s}', axis: 1, sign: 1, gap: g, tiers: [['P_humFairing_{s}'], ['P_elbowCover_{s}']] },
  { base: 'A_arm_{s}', axis: 1, sign: -1, gap: g, tiers: [['P_foldActuator_{s}']] },
  { base: 'A_forearm_{s}', axis: 1, sign: 1, gap: g, tiers: [['P_leFairing_{s}']] },
  { base: 'A_forearm_{s}', axis: 1, sign: -1, gap: g, tiers: [['P_foldRod_{s}'], ['A_secondaries_{s}']] },
  { base: 'P_wristJoint_{s}', axis: 1, sign: 1, gap: g, tiers: [['P_carpalCover_{s}']] },
  { base: 'A_hand_{s}', axis: 1, sign: 1, gap: g, tiers: [['P_handFairing_{s}'], ['P_alula_{s}']] },
  { base: 'A_hand_{s}', axis: 1, sign: -1, gap: g, tiers: [['A_primaries_{s}']] },
]
const LEG: Stack[] = [
  { base: 'A_leg_{s}', axis: 1, sign: -1, gap: G, tiers: [['P_legUpper_{s}'], ['P_tarsus_{s}'], ['P_foot_{s}']] },
]

export const LAYOUT: Stack[] = [
  { base: 'P_frame', axis: 1, sign: 1, gap: G, tiers: [['P_fc'], ['P_shell_rear', 'P_scapular_L', 'P_scapular_R']] },
  { base: 'P_frame', axis: 1, sign: -1, gap: G, tiers: [['P_battery', 'P_vision', 'P_pdu'], ['P_shell_front', 'P_ventral', 'P_flank_L', 'P_flank_R'], ['A_landing']] },
  ...side(LEG, 'L'), ...side(LEG, 'R'),
  ...side(WING, 'L'), ...side(WING, 'R'),
  { base: 'P_frame', axis: 2, sign: 1, gap: 0.08, tiers: [['A_head']] },
  { base: 'P_neck', axis: 2, sign: 1, gap: g, tiers: [['P_neckCradle'], ['A_headUnit']] },
  { base: 'P_headFrame', axis: 1, sign: 1, gap: g, tiers: [['P_crown']] },
  { base: 'P_headFrame', axis: 2, sign: 1, gap: g, tiers: [['P_optic_L', 'P_optic_R'], ['P_face_L', 'P_face_R'], ['P_beak']] },
  { base: 'P_frame', axis: 2, sign: -1, gap: G, tiers: [['A_tail']] },
  { base: 'P_tailMount', axis: 2, sign: -1, gap: g, tiers: [['P_tailActuator'], ['A_tailFan']] },
]

export const DETACH_TOTAL = 3.7
export const ASSEMBLE_TOTAL = 3.3

export function infoFor(name: string): PartInfo | undefined {
  return INFO[name]
}
