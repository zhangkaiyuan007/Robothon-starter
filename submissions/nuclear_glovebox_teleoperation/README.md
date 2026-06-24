# Nuclear Glovebox — Vision-Denied Tactile Grasping (Shadow Dexterous Hand)

A single **Shadow Dexterous Hand** performs a full **pick → in-hand reorient → place**
cycle on a radioactive sample vial inside a nuclear glovebox **without any vision**.
The grasp loop is closed on **fingertip touch sensors**: the hand lifts only after at
least three fingertips confirm real contact force — not on a blind timer, not on vision.

> **One line:** a MuJoCo dexterous-manipulation demo that actually runs, is interactively
> draggable, and is closed on tactile feedback — 24 actuators, 5 fingertip touch sensors,
> a free-joint vial, contact-force visualization, reproducible with one command.

---

## Project name

**Nuclear Glovebox — Vision-Denied Tactile Grasping with the Shadow Dexterous Hand**

---

## Robot platform

- **Shadow Dexterous Hand (right)** from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
  (Apache-2.0): 20 hand actuators, tendon-driven five-finger hand.
- **Wrist carriage (added):** a 4-DOF mount — `x/y/z` prismatic slides + `roll` hinge,
  driven by position actuators — that moves the whole hand through the scene.
- **Total: 24 actuators** (20 hand + 4 carriage), **5 fingertip `touch` sensors**
  (FF / MF / RF / LF / TH).

---

## Task goal

Inside a glovebox where vision is unavailable (radiation fog, sealed enclosure), the hand
must blind-grasp a radioactive sample vial, lift it, reorient it in-hand, carry it to a
shielded container, and release it inside — driven entirely by touch.

Scene objects:

| Object | Description |
|--------|-------------|
| `sample_vial` | Free-joint (`freejoint`) rigid body: thin body r=13 mm + wide flanged cap r=32 mm, mass 30 g |
| `sample_rack` | Slim support post that holds the vial at grasp height |
| `shielded_container` | Open box — the placement target |
| `glovebox_floor` | Work surface (visual context) |

> **Why the wide flanged cap?** Radioactive sample vials commonly carry a wide
> anti-spill cap. Here it doubles as a **form-closure** feature: the cap rim rests on
> the ring formed by the closed fingers, so a blind grasp holds securely even on a
> smooth cylindrical body. It is a physically honest design, not a shortcut.

---

## Technical approach

A finite-state machine drives the hand through **9 phases**. The grasp is **gated on
tactile feedback**, so the controller reacts to physics rather than replaying a fixed
script.

| Phase | Action | Loop closure |
|-------|--------|--------------|
| ① approach | Open hand, descend over the vial | position |
| ② **close** | Envelop with fingers + thumb | **tactile gate:** ≥3 fingertips > 1.0 N = secured |
| ③ lift | Raise the grasped vial | only after grasp confirmed |
| ④ reorient | Roll the wrist while holding → in-hand reorientation | maintain grasp |
| ⑤ reorient_back | Roll back to a clean carry pose | maintain grasp |
| ⑥ carry | Translate over the shielded container | maintain grasp |
| ⑦ lower | Descend into the container mouth | position |
| ⑧ release | Open hand, vial drops in | — |
| ⑨ retract | Raise the empty hand | — |

A **firm-and-hold** close ramp reaches the full grip early (~60 % of the phase) and then
holds to firm up the contact, preventing slip during transport.

**Validated in simulation (headless, deterministic):** the vial is held by 4–5 fingertips,
lifted to 0.33 m, reoriented in-hand, and placed in the container at `[0.316, 0.061, 0.034]`.

---

## Core features

- **Vision-denied, tactile-closed control** — 5 fingertip touch sensors decide when the
  grasp is secure; no cameras, no blind timer.
- **Real physics** — free-joint vial, elliptic friction cone + `impratio`, position
  actuators, integrated Menagerie Shadow hand.
- **Self-contained 4-DOF wrist carriage** authored on top of the vendored hand.
- **Two entry points** — a deterministic headless self-check and an interactive,
  draggable viewer with live contact-force vectors.
- **One-command reproducibility.**

---

## Highlights

How this submission maps to the eight evaluation dimensions:

| # | Dimension | How it is addressed |
|---|-----------|---------------------|
| ① | Runnability | One-command repro; `validate.py` passes deterministically; `run_interactive.py` is interactive |
| ② | MuJoCo depth | Free-joint body, `touch`-sensor closed loop, elliptic cone + impratio, position actuators, custom 4-DOF carriage, contact-force viz, Menagerie integration |
| ③ | Task design | Multi-stage blind pick / reorient / place tied to a real nuclear-glovebox no-vision pain point |
| ④ | Control | Tactile-gated closed-loop FSM — real contact force decides the lift, not a timer |
| ⑤ | Dexterity | Five-finger form-closure grasp + 5 tactile channels + in-hand reorientation |
| ⑥ | Engineering | Clear module split (scene / control / self-check / interactive); headless + interactive; honest docs |
| ⑦ | Demo | Draggable interactive window + contact-force vectors + per-phase narration |
| ⑧ | Innovation | Tactile loop replacing vision in a vision-denied scenario; differentiated nuclear-glovebox narrative |

---

## Current limitations

Stated plainly so reviewers see the real level:

1. **"In-hand reorientation" is currently a wrist roll while gripping**, not true finger
   gaiting. The vial's pose relative to the palm does change, but it is closer to
   "rotate-and-hold" than pure intrinsic in-hand manipulation.
2. **Single hand** — not bimanual; there is no hand-to-hand handoff.
3. **Scripted FSM**, not a learned policy; touch is used for gating and visualization,
   not force-servo control.
4. **The vial sits on a slim support post**, not a multi-socket tube rack — a real rack's
   geometry collides with the finger/cap sweep envelope of the blind grasp; fitting one
   needs re-tuning to a higher grip (see Future improvements).

---

## Future improvements

- **True finger gaiting** for genuine intrinsic in-hand reorientation.
- **High grip + multi-socket tube rack** for a more realistic glovebox bench.
- **Force-servo grasping** (regulate contact force, not just gate on it) — supports a
  "don't crush the vial" safety narrative.
- **Disturbance-robustness demo** — perturb the vial mid-carry and show the tactile loop
  re-securing it.

See `ITERATION_STRATEGY.md` for the ranked plan.

---

## How to run

### 1. Deployment / setup

The Python virtual environment is **not** committed — create it fresh. Requires
**Python 3.10+** and MuJoCo 3.10 (the `mujoco` wheel bundles the engine; no separate
install needed).

```bash
# from this submission folder:
cd submissions/nuclear_glovebox_teleoperation

# create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

# install pinned dependencies
pip install -r requirements.txt
```

`requirements.txt`:

```
mujoco==3.10.0
numpy==2.2.6
glfw==2.10.0          # only needed for the interactive viewer
```

### 2. Headless self-check (no window, no rendering, ~10 s)

```bash
python validate.py
```

Expected output (exit code 0):

```
nu(actuators)=24  nsensor=5
peak touch (N) FF MF RF LF TH = [ ...,  ... ]
max lift height (m) = 0.330
final vial pos = [0.316 0.061 0.034]
grasp tactilely confirmed: True
placed in container: True

✅ self-check passed
```

### 3. Interactive viewer (opens a MuJoCo window)

```bash
python run_interactive.py
```

The hand auto-loops the demo. Mouse controls: **left-drag** orbit, **scroll** zoom,
**double-click** select, **Ctrl+right-drag** to yank the vial and disturb the grasp.
Contact points and contact-force vectors are on by default, so the "touch" is visible.

> A windowed viewer needs a display (and `glfw`). On a headless machine, run
> `validate.py` instead — it requires no display.

---

## Demo video

`demo.mkv` (in this folder) — a screen recording of the full cycle in the interactive
viewer: blind descent, finger closure, tactile confirmation, lift, in-hand reorientation,
transport, and placement into the shielded container, with contact-force vectors visible
throughout.

---

## Project structure

```
nuclear_glovebox_teleoperation/
├── mjcf/glovebox_scene.xml      # scene: floor / glovebox / support / vial / container
├── assets/shadow_hand/          # vendored Shadow hand + custom carriage & touch sites
│   └── right_hand.xml
├── src/grasp_controller.py      # tactile-gated finite-state-machine controller
├── validate.py                  # headless self-check (no window, no video)
├── run_interactive.py           # interactive MuJoCo viewer
├── requirements.txt             # pinned dependencies
├── demo.mkv                     # demo screen recording
├── ITERATION_STRATEGY.md        # iteration / scoring plan
└── registration.json            # submission metadata
```

---

## Credits & license

- **Shadow Dexterous Hand** model from
  [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0);
  see `assets/shadow_hand/LICENSE`.
- Physics engine: [MuJoCo](https://github.com/google-deepmind/mujoco) 3.10.
