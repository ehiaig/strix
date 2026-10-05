# STRIX-01 — Engineering Concept

A concept-level engineering visualisation of an owl-shaped, electrically powered flapping-wing robot.
**Nothing here is aerodynamically or structurally validated.** Figures marked *measured* come from the 3D model
via the Blender build script (`blender/build_report.json`); everything else is an assumption or a hand estimate.
All performance numbers are carried at one mass, the **2.28 kg budget mass** of the 3D model. Actuator resizing
identified in review (+76 g, § 3) is booked against a **30 % growth margin (2.96 kg)**, quoted where it matters;
25–40 % is typical growth for a first prototype at this stage.

**Primary risk, stated up front:** wing loading of 60–70 N/m² (1.5–2× Robird, ≈ 6× SmartBird), which forces a
mean C_L ≈ 0.9 and a mid-downstroke sectional C_L ≈ 2 (§ 4), on a quasi-steady estimate that is probably
optimistic. No electric flapper has flown at this loading. **First development milestone:** a single wing and
drive on a whirl arm with a six-axis balance, to measure mean and peak lift, thrust and shaft power at the
cruise reduced frequency before any other part of the design is frozen.

## 1. Architecture and drive

| | Crank-rocker (one motor) | Independent wing-root actuators (chosen) |
|---|---|---|
| Precedent | Festo SmartBird, hobby ornithopters | Research flappers with servo-driven wings |
| Symmetry | Mechanically locked | Commanded; amplitude asymmetry gives roll authority |
| Glide / hold | Needs a wing lock | Motors hold position; a stroke brake locks the glide and stow positions |
| Stroke shaping | Fixed by linkage | Software-defined (amplitude, bias, down/up ratio) |
| Cost | Lighter, efficient | Two motors, reversing losses, a heavy drive (≈ 325 g per side) |

Each wing root has a **58 mm-class pancake motor** (Kv ≈ 180 rpm/V, Kt ≈ 0.053 N·m/A, Km ≈ 0.13 N·m/√W,
≈ 1.3 N·m peak for a few seconds, ≈ 150 g — assumed datasheet class) with an integrated FOC driver, and an
**absolute encoder on the stroke axle** itself. This is a **geared, back-drivable drive** (13.8 : 1), not
quasi-direct drive:

1. **3 : 1 toothed belt**: 3 mm-pitch, 12 mm-wide aramid-cord polyurethane belt on 12 / 36 mm pulleys, tensioned
   by an eccentric motor mount (fixed centres: a sprung idler would let teeth jump on every torque reversal).
   Peak tension ≈ 150 N at take-off.
2. **4.6 : 1 capstan stage**: **two parallel 0.8 mm Dyneema SK99 cables** (≈ 2 kN combined break; static safety
   factor ≈ 5 at the 370 N take-off peak, ≈ 3 once splice/termination efficiency is included; capstan/cable
   diameter ratio 16) on a 13 mm capstan driving a 60 mm sector drum, with screw tensioners at each cable end. The
   capstan shaft runs in its own bearing pair in the girdle. **Bending fatigue over D/d = 16 at 4 Hz limits cable
   life to the order of tens of flight hours**; the cables are a scheduled-replacement item, and creep needs
   periodic re-tensioning.

The sector is asymmetric (−95° … +65° of stroke) to cover the flapping range and the −90° stow. Belt and cable
stiffness against the reflected inertia put the first drive-train mode at an estimated 40–70 Hz. The belt is the
main tonal noise source (≈ 600 Hz mesh); a second capstan stage would remove it.

### Stroke inertia (per wing, computed from the model)
Each part's budgeted mass is spread over its own geometry in the extended pose and summed about the stroke axis:

| Group | Mass | Contribution |
|---|---|---|
| Primaries (10) | 42 g | 7.0×10⁻³ kg·m² |
| Spars (humeral, forearm, hand) | 51 g | 2.3×10⁻³ |
| Fairings, median coverts and joint covers | 37 g | 1.8×10⁻³ |
| Secondaries and tertials | 52 g | 1.6×10⁻³ |
| Wrist, fold actuator, rod, alula, membrane (50 g); hub, sector drum, pitch servo (84 g) | 134 g | 1.0×10⁻³ |
| **Wing, moving (316 g)** | | **0.0137 kg·m², r_g ≈ 0.21 m** |

The hand starts at the wrist (r ≈ 0.25 m) and the long primaries rake back, so the mass sits further inboard than
a uniform-rod estimate would suggest. Reflected rotor inertia ≈ 4×10⁻⁵ × 13.8² ≈ 7.6×10⁻³ kg·m², so the drive
sees **I ≈ 0.021 kg·m²**.

### Closing the lift–torque loop (per wing)
Lift is taken as a half-sine over the downstroke (56 % of the cycle) with 20 % of that on the upstroke, so peak
lift = mean lift / 0.41. The lift centre sits at ≈ 0.5 R from the stroke axis in cruise (advance ratio ≈ 1.7) and
≈ 0.6 R at take-off; R = 0.64 m. Stroke rates use the real 56/44 split (downstroke peak ≈ 13.4 rad/s, upstroke
≈ 17 rad/s in cruise).

| Quantity | Cruise (10.5 m/s, ±36°, 3.8 Hz) | Take-off (−46° … +58°, 4.3 Hz) — **sizing case** |
|---|---|---|
| Mean / peak force per wing | 11.2 N / 27.1 N | 11.7 N / 28.5 N (1.05 W: weight + 3 m/s² forward) |
| Aerodynamic torque at the axle (peak) | 8.7 N·m (r ≈ 0.32 m) | 10.8 N·m (r ≈ 0.38 m) |
| Spring | engaged, k ≈ 12.1 N·m/rad (tuned to 3.8 Hz) | **engaged** |
| Off-resonance inertial torque I·θ₀·(ω² − ωₙ²) | ≈ 0 | ≈ 3.1 N·m |
| Combined peak at the axle | 8.7 N·m | 11.2 N·m (quadrature) |
| Peak motor torque (÷ 13.8, η ≈ 0.88) | 0.72 N·m (RMS ≈ 0.38) | 0.92 N·m for ≈ 2.5 s |
| Motor speed peak / back-EMF | ≈ 2,000 rpm / 11 V | ≈ 3,240 rpm / 18 V (+ ≈ 2 V I·R) |
| Phase current peak | ≈ 14 A | ≈ 17 A |
| Copper loss per motor | ≈ 8.5 W (+ ≈ 3 W iron) | ≈ 30 W for 2.5 s |

* **Lift model caveat:** the table is quasi-steady. At the cruise reduced frequency k = πfc/U ≈ 0.28, Theodorsen's
  function attenuates attached-flow circulatory lift to ≈ 0.75 of its quasi-steady value and lags it ≈ 15°, while
  leading-edge-vortex (delayed-stall) effects may add lift on the outer wing. Neither is modelled here; the net is
  unknown, which is why the whirl-arm test is the first milestone.
* **Spring requirement:** k = I·ω² ≈ 0.021 × (2π·3.8)² ≈ 12 N·m/rad over ±58° (6.2 J at the take-off peak, 2.4 J
  at the ±36° cruise amplitude), ≈ 10⁶ cycles. A metal or GFRP **torsion tube was rejected**: at a fatigue-safe
  shear strain of ≈ 1 % it would need a length of ≈ 100 radii (≈ 0.9 m) to twist 58°.
* **Chosen spring: elastomer torsion unit** on the stroke axle: five bonded natural-rubber shear rings in series
  (12 / 17 mm radii, G ≈ 1.2 MPa, ≈ 30 cm³ of rubber), peak shear strain ≈ 0.5 in cruise and ≈ 0.8 at the
  take-off peak; ≈ 55 g with its sleeves. Costs: **hysteresis** (tan δ ≈ 0.05) dissipates ≈ 0.75 J per cycle,
  **≈ 3 W per wing in cruise**, which heats the rubber (≈ 2 K internal gradient; the housing runs ≈ 10 K above the
  cooling air), and the stiffness drifts ≈ ±10 % with temperature, moving the resonance ≈ ±5 % — the stroke
  controller tracks it, trimming frequency to the measured resonance. It stays **engaged for take-off** — the bias
  there is only ≈ +6° (≈ 1.3 N·m), whereas releasing it would leave the motor ≈ 14 N·m of inertial torque; the
  ≈ 11 take-off cycles per flight at the higher strain are negligible for fatigue.
* **Clutch:** bistable (latching) solenoid dog clutch (17 g), shifted only at the stroke neutral read from the axle
  encoder, during a glide hold or on the perch, where its halves have no relative speed. Released for stowing; no
  holding power in either state.
* **Stroke brake / glide lock:** a spring-applied, solenoid-released index pin (≈ 8 g) on the hub rides on the
  sector drum and drops into detents at **+5° (glide dihedral)** and **−90° (stow)**. It is fail-safe: on power
  loss the engaged elastomer spring returns the stroke towards neutral and the pin catches the glide detent. In
  normal flight it is held released.
* **Thermal:** ≈ 11.5 W per motor plus ≈ 3 W per spring in cruise. Cooling air enters through the annular gap
  around the neck gimbal, passes the motors, springs and power stage and leaves through the ventral louvres; the
  shoulder slots are sealed by bellows boots. Without that airflow the cruise case does not close — a key risk.
* **Voltage:** the critical case is the **landing flare at end of flight** (≈ take-off torque on a sagging pack,
  ≈ 20 V needed vs ≈ 21 V available). The controller reserves 15 % of the battery for it.
* **Unpowered perch:** at −90° the folded wing hangs below its stroke axle in a stable equilibrium and the stroke
  brake pin sits in the stow detent; the sweep is held by its worm drive (non-back-drivable); legs and toes latch
  mechanically. Quiescent draw ≈ 2 W.
* **Load path:** both stroke axles and capstan shafts run in bearing pairs in a box-section shoulder girdle: front
  and rear **CFRP bulkheads with bonded titanium bearing inserts**, carbon top and bottom plates, bolted to the keel
  and battery tray; the motors bolt to a third CFRP bulkhead with aluminium inserts. Primary structure is budgeted
  at 200 g (≈ 9 %), still lean for 11 N·m reversing at 4 Hz.

## 2. Wing mechanism (per wing)

| Joint | Axis | Actuation | Range used |
|---|---|---|---|
| Stroke | body longitudinal | pancake motor + belt + capstan | ±36° cruise, −46° … +58° take-off; stow −90° |
| Sweep | wing normal, at the yoke clevis | worm-geared servo (≈ 22 g, non-back-drivable): stow, landing and pitch trim | −10° … +96° |
| Arm pitch | humeral spar axis | high-speed servo on the hub (≤ 0.05 s/60°), sized for **≈ 1.2–1.5 N·m peak** (centre-of-pressure travel to ≈ 40 % chord under dynamic stall), with a preloaded pitch spring carrying the mean nose-up moment; its push-rod runs through the hollow sweep pin, so sweeping does not change pitch | ±8° cruise, ±15° max |
| Elbow + wrist | wing normal | high-speed ball-screw fold actuator (≈ 60 N, ≈ 25 W peak) via bellcrank; wrist slaved by a parallel rod | 0 … −145° / +150° |
| Hand twist | hand spar | slaved to arm pitch by a 2 : 1 tendon pulley pair centred on the wrist axis | ≈ ±16° |
| Primary pitch | each rachis | **passive**: asymmetric vane on a compliant root; nose-down under downstroke load (to ≈ 12° at P10), opens against a 13° stop on the upstroke | −12° … +13° |
| Alula | hand leading edge | **passive**, spring-loaded | take-off / flare |

The hand-twist tendons run over pulleys concentric with the wrist hinge, so wrist flexion does not change their
length — but routing them across a 150° hinge that also cycles at 4 Hz is a recognised wear risk.

### Spanwise pitch, cruise mid-downstroke
φ = atan(stroke velocity / airspeed) (downstroke peak 13.4 rad/s), θ = geometric pitch (arm pitch + hand twist +
passive feather pronation, leading edge down), α = φ − θ *before* ≈ 5° trim incidence:

| r/R | φ | θ | α |
|---|---|---|---|
| 0.3 (arm) | 14° | 8° | 6° |
| 0.5 (inner hand) | 22° | 20° | 2° |
| 0.7 | 30° | 24° | 6° |
| 0.9 | 36° | 28° | 8° |

With trim incidence ≈ 7–13°. A residual ≈ 8° pitch step remains at the wrist.
On the upstroke the arm supinates (−6°, hand −12°), the primaries open against their stops and the hand flexes.

### Folding and upstroke flex
* The fold actuator flexes the elbow; a rod in the forearm fairing couples elbow and wrist as the radius/ulna do,
  giving a Z-fold. In the perched stow the humerus and forearm lie on top of each other (as in birds), so the fold
  reads as one panel ending in the hand.
* **Upstroke flex is active** and identical on both wings: ≈ 0.3 of full fold (≈ 45° elbow, ≈ 48° wrist) at
  mid-upstroke. Centrifugal load (≈ 12 g, ≈ 9 N on the hand) plus the aerodynamic load on the hand tends to
  straighten the fold, and the flex must happen in ≈ 0.13 s, so the actuator peaks at **≈ 60 N at the bellcrank and
  ≈ 25 W** (≈ 8 W mean per wing); it is a life-limited part. In the
  downstroke the joints sit on elastomer-faced extension stops; **over-centre locks engage only for gliding and
  holding** (cam-driven at the end of the fold actuator's extension stroke).

### Fairings and seals
Each wing section carries a cambered **D-nose fairing** in 0.3 mm carbon/PEKK skin (≈ 0.45 kg/m²; titanium would
weigh about four times more): blunt super-elliptic nose, upper surface carried back over the feather roots,
finished with a row of median coverts that shingle onto the greater coverts. The spar sits at ≈ 20 % chord.
Rounded elbow and carpal covers, symmetric about their fold axes, close the joint gaps. Tertials and an elastomer
root membrane close the wing–body gap; bellows boots seal the yoke slots; tail coverts close the rump; the belly
shell has leg slots.

### Perched stow
Stroke −90°, sweep +96°, full fold, secondaries swung back alongside the hand; spring-hinged **scapular covers**
drop over the folded wing roots. The primaries end ≈ 10 cm below the perch bar and ≈ 5 cm short of the tail tip.

## 3. Mass, geometry and centre of mass

| Item | Value |
|---|---|
| Span (extended) | 1.44 m *(measured)* |
| Planform incl. body | 0.371 m² *(measured, rasterised top view)* |
| Wing-only area | 0.318 m² *(measured)* |
| Aspect ratio | 5.6 (broad, rounded, owl-like) |
| Budget mass | **2.28 kg** (nothing weighed) — 2.36 kg with the identified growth below — **2.96 kg with 30 % growth** |
| Wing loading | 60 N/m² total, 70 N/m² wing-only (78 / 92 at 2.96 kg) |
| CG | centreline, ≈ 22 mm behind and ≈ 27 mm below the stroke axes *(computed from the budget)* |

Mass by group (g): wing drive 648 · core 496 · wings 465 · landing gear 184 · head 170 · shells 168 ·
payload 8 (comms antenna; the surveillance cameras are the head eyes, § 8) ·
wiring & misc 90 · tail 60.

* Wing drive per side: motor 150, belt + capstan stage 28, hub + worm sweep servo + axle encoder 40, sector drum
  14, torsion spring 45 + clutch 17, pitch servo 30.
* Wing per side: feathers 94 (10 primaries × 4.2 g, 11 secondaries × 4.0 g incl. covert, 3 tertials × 2.8 g),
  spars 51, fairings with median coverts and joint covers 37, fold actuator 35, wrist, rod, alula and membrane 15
  (232 g).
* **Identified growth, not yet in the model's budget** (per side): elastomer spring +10 g (55 g vs 45 g), arm-pitch
  servo +10 g (1.2–1.5 N·m class), fold actuator +10 g (60 N class), stroke brake +8 g — **+76 g in total**, i.e.
  2.36 kg. All tables below stay at 2.28 kg; at 2.36 kg every load and power figure rises ≈ 3–5 %.
* Core: keel, girdle and three CFRP bulkheads with titanium/aluminium bearing inserts 200, battery 220, flight
  controller 16, vision/IMU 26, power 34.
* Head 170 g (7 %): two cameras, iris and lid actuators, a two-axis neck. It is the price of the owl.
* Legs 92 g each: hip actuator + latch + catapult leaf spring 40, upper leg 22, tarsus 12, foot 18.
* Wiring, connectors and fasteners 90 g (≈ 4 %) for ≈ 20 actuators and 50 A power runs.
* **Surveillance payload 8 g** (now in the model, § 8): the dorsal comms antenna. The surveillance cameras are the
  head's own forward eyes on the neck gimbal (§ 8.2), so they add no mass. The model's total is ≈ 2.29 kg; the
  performance tables are still carried at 2.28 kg for continuity.
* **Battery:** 6S 1300 mAh 45C LiPo, ≈ 29 Wh, ≈ 220 g (≈ 9.6 % of the mass). Moving ≈ 100 g from head or drive
  into battery would buy ≈ 40 % more endurance — the main trade still open.
* The 30 % growth margin covers the identified +76 g and ≈ 600 g of unidentified growth; at 2.96 kg the wing
  loading reaches 78 N/m², which would push cruise to ≈ 12 m/s at the same C_L.

**Perched balance:** the hip angle is solved at start-up so the CG sits directly above the perch bar.

## 4. Flight envelope, stability and power (estimates)

* **Speeds:** cruise **10.5 m/s**, mean C_L = W/(qS) ≈ **0.89** total planform (1.04 wing-only; 1.16 / 1.36 at
  2.96 kg). Stall at steady C_L,max ≈ 1.6: **≈ 7.8 m/s**. The minimum-power speed falls below stall, so this is a
  fast flier: minimum practical ≈ 8.5 m/s, best range ≈ 9.5 m/s; 10.5 m/s is chosen for C_L margin.
* **Peak lift, not only mean:** with the half-sine model the mid-downstroke force is ≈ 2.4× the mean, i.e. a
  sectional C_L ≈ 2.0–2.1 at a local q ≈ 80 Pa, above the steady 1.6. It relies on unsteady augmentation (reduced
  frequency ≈ 0.28) and the slotted tip — and the attached-flow (Theodorsen) attenuation at that reduced frequency
  works against it (§ 1). Alternatives if tests fall short: carry 35–50 % of the lift on the upstroke (more
  supination), add area, or cut mass.
* **Turns:** radius **18 m at ≈ 32° bank** (n ≈ 1.18, mean C_L ≈ 1.05) at cruise speed; the demo caps bank at 34°.
  A 12 m turn would need 43° and a mean C_L ≈ 1.22, beyond the lift margin. Roll from up to ±35 % stroke-amplitude
  asymmetry; the +5° stroke bias gives effective dihedral and flapping gives strong roll damping.
* **Wingbeat:** Pennycuick-type scaling for 2.28 kg / 1.44 m / 0.37 m² gives ≈ 3.9 Hz (demo 3.8 Hz cruise).
* **Pitch:** wing aerodynamic centre ≈ 25 % MAC (MAC ≈ 0.25 m), ≈ 13 mm behind the stroke axes; CG ≈ 22 mm behind.
  The tail (≈ 0.03 m² closed, ≈ 0.045 m² spread, arm ≈ 0.24 m; tail volume ≈ 0.09) moves the neutral point ≈ 3 %
  MAC aft, but the Munk moment of the head and body (≈ 0.007 m³) moves it ≈ 3–4 % forward: **static margin
  ≈ −7 … −2 % MAC, i.e. statically unstable**, as many birds are, with a **time-to-double of ≈ 0.15–0.3 s**.
  - **Fast pitch effector: the tail pitch servo** (≥ 10 Hz bandwidth; ≈ ±0.6 N·m with the fan spread at cruise q,
    several times the unstable moment of a 5° disturbance). It closes the stabilising inner loop.
  - **Stroke-plane bias** (shifting the mean stroke angle) is changed once per half-stroke and only acts through
    the wingbeat-averaged force, so its effective bandwidth is ≈ 1 Hz: it trims the outer loop.
  - **Sweep** is worm-geared and slow: **trim only** (C_m0 of the cambered section ≈ −0.06 is trimmed by ≈ 3° of
    forward sweep).
* **Yaw:** no fin, and the head ahead of the CG is destabilising (C_nβ ≈ 0 or slightly negative); yaw is
  controlled actively by **tail tilt**, **differential arm pitch** (thrust difference between the wings, through
  the fast pitch servos) and **differential upstroke flex** (fold actuators). There is no separate primary-spread
  actuator.
* **Wingbeat-induced motion:** ≈ ±2–3 cm body heave and a pitch couple at 3.8 Hz. The neck gimbal stabilises the
  head against them (head-stabilised vision, as in birds), using the IMU.
* **Sensing:** IMU, head cameras, axle encoders, and a pitot at the leading edge of the left humeral fairing
  (inboard, small stroke motion, corrected with the axle encoder), clear of the facial-disc wake.
* **Thrust:** cruise drag ≈ 3.4 N (≈ 15 % of weight), from the forward tilt of the outer-wing lift on the pronated
  downstroke.
* **Power, reconciled with the torque model:** aerodynamic ≈ 35 W (induced 14 W, flapping-augmented profile
  13.5 W, parasite 7.4 W with C_D ≈ 0.45 on the bluff head and body). Shaft power from the same lift and stroke
  histories: downstroke ≈ 33 W per wing, upstroke regeneration ≈ 6.5 W per wing, so 52–65 W at the axles
  depending on how much regeneration survives the drive — an implied flapping efficiency of ≈ 0.55–0.65 — plus
  ≈ 6 W of elastomer-spring hysteresis: **58–71 W at the axles**. Electrical: ÷ 0.92 transmission, + ≈ 17 W
  copper, ≈ 6 W iron, ≈ 4 W driver, ≈ 16 W fold actuators, ≈ 17 W avionics, servos and head →
  **≈ 130 W (120–155 W)**.
* **Endurance:** 23 Wh usable → **≈ 9–11 min** at budget mass, ≈ 7–8 min at the 30 % growth mass, less ≈ 0.2 Wh
  per take-off (≈ 300 W average, ≈ 15 A from the pack, for ≈ 2.5 s; the 17 A above are phase currents).
* **No sustained hover:** ≈ 170 W of mechanical power at a realistic figure of merit.

### Launch and landing
* **Launch:** the robot crouches ≈ 7 cm (hip and ankle), then a glass-fibre **catapult leaf spring in each hip**
  (≈ 1.9 J each, cocked by the hip actuator, latched) pushes it off at ≈ 1.5 m/s while the wings start take-off
  strokes. That is far below the ≈ 7.8 m/s needed for weight support: falling alone from 1.5 to 7.8 m/s costs
  3.0 m of height, and flapping thrust (≈ 3 m/s²) and partial lift recover only part of it. **Planning figure:
  2–4 m height loss.** A real launch therefore needs a perch **≥ 4–5 m above clear ground**, or a bungee/rail
  launcher that releases at ≥ 8 m/s. **The demo's 1.35 m stand and ≈ 0.55 m dip are a visual simplification**, not
  a feasible trajectory.
* **Sequence from the stowed perch:** unfold (≈ 0.8 s), stroke to neutral, engage clutch, crouch and cock, release
  on the first downstroke.
* **Landing:** an approach at ≈ 8.5 m/s, then a pitch-up flare with the tail spread, braking strokes, legs swung
  forward, feet closing on contact at ≈ 1 m/s. The flare is the voltage-limited sizing case. Perching manoeuvres
  have been shown by sub-kilogram flappers and gliders; at 2.3 kg it is a research-level risk.
* In cruise the legs trail straight back under the tail with the toes clenched.

### Failure handling
A cable or belt failure shows up as a mismatch between motor and axle encoders; the stroke brakes are then
released into their glide detents (+5°) on both wings and the robot spirals down on tail and sweep. A geofence and
a kill link cut the flapping drive; the springs return the strokes towards neutral and the fail-safe brake pins
catch the glide detents. A 2.3 kg, 1.44 m machine needs these before any free flight.

## 5. Actuator list

Wing (×2): stroke motor, arm-pitch servo (1.2–1.5 N·m class, with pitch spring), worm-geared sweep servo (trim and
stow), high-speed fold actuator (≈ 60 N), latching spring-clutch solenoid, **stroke-brake solenoid** (spring-applied,
solenoid-released glide/stow lock). Hand twist slaved; feather pitch and alula passive.
Shoulder (×2): scapular-cover micro-servo. Tail: pitch servo (in the mount), tilt and fan-spread servos.
Legs (×2): geared hip actuator that also cocks the catapult spring, with a latch; toe-tendon actuator with ratchet;
passive spring ankle. Head: neck pitch servo, pancake yaw motor, iris and shutter-lid actuators.

**Noise** (the reason for the owl features): elastomer-faced stops at the joints and feather roots, a capstan
final stage, a comb on P10 and fringed trailing edges. The belt stage remains the tonal source.

## 6. Component hierarchy (GLB node names)

```
OWL
├─ A_core: P_frame (keel + girdle), P_battery, P_fc, P_vision (+IMU), P_pdu,
│          P_shell_front, P_shell_rear, P_flank_L/R, P_ventral, J_scapular_L/R → P_scapular_L/R,
│          P_antenna (comms downlink)   ← surveillance variant; cameras are the head eyes (§ 8)
├─ A_head: P_neck, J_neckPitch → P_neckCradle, J_neckYaw → A_headUnit
│    └─ P_headFrame, P_crown, P_face_L/R, P_beak, J_eye_L/R → P_optic_L/R (iris blades K_iris_*, lids J_lid*)
├─ A_wing_L/R: P_rootMotor, P_capstan, J_shoulder → P_shoulderHub, P_sectorDrum, P_torsionSpring, P_pitchServo
│    └─ J_sweep → J_armPitch → A_arm: P_humerus, P_humFairing, P_foldActuator, P_membrane, P_elbowCover, P_ter_01..03
│         └─ J_elbow → A_forearm: P_sparPrimary, P_leFairing, P_carpalCover, J_foldRod → P_foldRod,
│              A_secondaries (P_sec_01..11)
│              └─ J_wrist → P_wristJoint, J_handTwist → A_hand: P_sparSecondary, P_handFairing,
│                   J_alula → P_alula, A_primaries (P_pri_01..10)
├─ A_tail: P_tailMount, J_tailPitch → J_tailTilt → P_tailActuator (+ tail coverts), A_tailFan (P_rect_01..12)
└─ A_landing: A_leg_L/R: P_hipMount, J_hip → P_legUpper, J_ankle → P_tarsus, P_foot (J_toe_* chains, G_grip)
```
`A_*` explodable assembly · `P_*` explodable part · `J_*` joint (animated, never exploded) ·
`FAST_*` fastener placements (rendered as instanced screws) · `G_*` grip reference.
The fuselage, head, tail and legs carry a 1.18 node scale relative to the wing structure (owl proportions);
the sweep joints carry the inverse, so the wings keep their size.

## 7. What remains unvalidated

The wing loading and the mean and peak lift coefficients it implies; the spanwise pitch schedule and passive
feather behaviour; the unsteady aerodynamics at k ≈ 0.28 (attenuation vs. leading-edge-vortex augmentation);
drive thermal balance (dependent on the cooling airflow) and voltage margin in the flare; structure, axle, drum,
belt and cable sizing and life; elastomer spring hysteresis, heating, stiffness drift and fatigue; clutch and
brake dynamics; endurance; the unstable pitch and weak yaw dynamics and their control laws (tail bandwidth and
authority); the launch (which needs height or a launcher) and the perching landing; landing loads (~4 g
assumed); noise; shell stiffness and impact resistance; failure handling; and the mass budget itself — no part
has been weighed, hence the 30 % growth margin.

## 8. Surveillance payload variant (concept extension)

A perch-and-observe variant, not a carrier: the robot flies in quietly, lands on a vantage point, records or
streams, and returns. **The surveillance cameras are the head's own forward eyes** (§ 8.2) — the forward-facing
owl eyes on the two-axis neck gimbal are already a steerable camera pair, so no separate turret is needed, and
nothing protrudes from the airframe. The one visible payload part the model carries is a dorsal comms blade
antenna (`P_antenna`, 8 g), in the exploded view. The rest of the variant (recorder, downlink radio, positioning
and command electronics, and the flat-perch foot) is still unbuilt and is sketched below. Everything here is
concept-level and unvalidated, and the hardest base-airframe risks (perching landing, unstable-flight control)
are the ones the surveillance mission leans on hardest.

### 8.1 Mission and why the owl form fits
The owl form earns its place only for **covert, quiet, perch-to-watch** observation — flying in without the
rotor signature of a multirotor, settling on a ledge, and watching from stillness. For a *fixed* vantage point a
concealed static camera or a hovering multirotor is simpler and more reliable; the flapping owl is justified only
where the quiet approach and the bird silhouette are the point.

### 8.2 Sensor and aiming — the eyes are the payload
* **Camera:** the two forward eye optics already modelled (`P_optic_L/R`) serve double duty — navigation vision
  and surveillance. Spec one as a stabilised EO (day) sensor and the other as low-light/thermal, giving a stereo
  EO/IR pair. No separate turret: the forward owl eyes are, by design, exactly where a perch-and-watch camera
  wants to be.
* **Aiming:** the existing **two-axis neck gimbal** (pitch + yaw) pans and tilts the head to aim the eyes while
  perched — the same gimbal the idle head-tracking already drives in the app. A roll axis or a small in-eye gimbal
  would be needed for a level horizon on a tilted perch.
* **The trade-off of eyes-only:** in flight the head also does navigation and wingbeat stabilisation, so it cannot
  independently lock onto a separate target while the body maneuvers — the capability a dedicated belly gimbal
  would add. For perch-and-watch this does not bite; it would only matter for persistent target-staring in flight,
  at which point a belly gimbal (and its weight and protrusion) could be reconsidered.

### 8.3 Getting the imagery back — two options
* **Record-and-return** (simplest, supported in principle today): add ≈ 3 g of onboard storage; the robot records
  while perched, flies home, and the card is read out. No radio range limit, lowest power, lowest detectability.
* **Live downlink** (much harder): add a video radio + antenna (≈ 25–40 g) and ≈ 10 g of video-encode compute.
  Range is limited, it raises power draw and radio-frequency detectability, and antenna placement on a feathered,
  folding airframe is awkward. A hybrid — record continuously, burst a low-rate preview on demand — is the
  sensible middle path.

### 8.4 Navigation and remote control (the biggest gap)
The app's "flight" is a scripted kinematic path, not a controllable system. Real go-to-a-location and remote
positioning need: a **positioning source** (GNSS ≈ 8 g, and/or visual-inertial odometry reusing the head
cameras for GNSS-denied indoor work), a **command/telemetry uplink** (≈ 15 g, can share the downlink radio), and
— the real blocker — **working flight-control laws for a statically unstable flapping aircraft**, which § 4 lists
as unvalidated. Precise station-keeping and a clean perch approach are research-level at 2.3 kg. Realistically the
operator designates a target and an approach, and the robot flies it semi-autonomously; continuous manual
"stick" control of a flapping platform is not a near-term option.

### 8.5 Perching on a ledge or sill
The base zygodactyl foot is built to clamp a **rounded perch bar** (toes wrap a cylinder, ratchet holds it
unpowered); a flat windowsill gives the toes nothing to close around. The variant needs an **alternative foot
module**: compliant flat pads with a high-friction or dry-adhesive contact, or a shallow spring clamp that hooks
the sill lip. A first-storey sill (≈ 3–4 m) is actually convenient — it is too low to take off from the ground,
but high enough to **drop-launch** from on the way out, which the base airframe needs anyway.

### 8.6 Mass, power and endurance
* **Added mass:** ≈ 60–90 g of sensor/comms/nav avionics plus the flat-perch foot change. This fits inside the
  30 % growth margin (§ 3) but consumes a meaningful slice of it; flight loads and power rise the few per cent
  that mass costs.
* **Flight power:** the avionics add ≈ 5–8 W continuous on top of ≈ 130 W cruise — a few per cent, trimming well
  under a minute off the 9–11 min flight endurance.
* **Perched-watch endurance is the useful number, and it is large.** Quiescent perch draw is ≈ 2 W; add ≈ 5–8 W
  for sensing and a live downlink and the robot can watch from a perch for **roughly 2–3 hours** on the 23 Wh
  usable pack (far longer if it records and only bursts data). Observation time is set by radio/sensor power on
  the perch, not by the flapping-flight budget — the opposite of a multirotor, which burns its whole battery
  staying airborne. This is the variant's one clear advantage.

### 8.7 Legal and deployment note
Recording from or toward windows and occupied buildings is regulated — in the UK, under data-protection law and
surveillance codes of practice, and elsewhere under local equivalents. Lawful deployment (consent, signage,
operator registration, airspace rules for the flight itself) will constrain where this can be pointed as much as
the engineering does, and should be scoped before any build.

### 8.8 What this variant adds to the unvalidated list
On top of § 7: the perch-approach and ledge-grip reliability on non-cylindrical surfaces; the semi-autonomous
navigation and station-keeping control laws (GNSS and GNSS-denied); downlink range, latency and detectability;
the sensor/gimbal stabilisation on a wind-loaded perch; antenna integration; thermal and power behaviour of the
added electronics; and the perched-endurance figures above, none of which have been bench-tested.
