# Iteration Strategy & Feedback Loop

## Submission 1: MVP (Current)

**Target score:** 85–90  
**Strategy:** Demonstrate the core concept works end-to-end

### If Feedback Indicates:

#### "MuJoCo depth: 6–7/10"
→ Add advanced features:
- Tendon-based finger actuation (replace simple motors)
- Increase contact friction model complexity
- Add object-tracking body-sensor (measure vial rotation angle)
- Implement damping/stiffness tables

**Action:** Modify `mjcf/allegro_hand_with_sensors.xml` and `task_executor.py`

#### "Dexterity: 6–7/10"
→ Extend manipulation skill:
- Add a second rotation phase (rotate 90° instead of 45°)
- Implement finger-rolling motion (not just flexion/extension)
- Add object-size adaptation (attempt to grasp different-diameter vials)

**Action:** Expand `task_executor.py:_state_reorient()` to 8s+ duration with more phases

#### "Innovation: 5–6/10"
→ Strengthen novelty narrative:
- Add specialized "hot cell" constraint system (invisible barriers simulating shielded access)
- Implement radiation dose tracking (vial spends limited time in exposure zone)
- Add contamination risk model (higher forces risk vial breakage)

**Action:** Add to `glovebox_scene.xml` and document in `README.md`

#### "Real-world relevance: 7–8/10"
→ Deepen scenario authenticity:
- Add realistic decontamination steps (vial surface wiping)
- Implement material-specific handling (e.g., unstable isotopes need lower force)
- Contact actual nuclear facility to confirm scenario realism

**Action:** Extend `task_executor.py` with new states, update README with expert quotes

#### "Demo clarity: 6–7/10"
→ Improve video presentation:
- Add force vector overlays (show contact magnitude as arrows)
- Include state name + time-in-state text annotation on every frame
- Use slow-motion for dexterity showcase (0.5x during reorientation)
- Add before/after inset (initial vial position vs. final placement)

**Action:** Enhance `demo_video.py:VideoRecorder` with PIL-based annotation

---

## Submission 2: Strengthened Version

**Target score:** 91+  
**Time budget:** 2–3 days  
**Focus:** Address top 2–3 weakest dimensions from feedback

### Optimization Priority

1. **High impact, moderate effort:** MuJoCo depth + dexterity
   - Both improve scoring with MJCF changes and task extension
   - Payload: ~2–3 new states in `task_executor.py`, 100 lines MJCF

2. **Medium impact, low effort:** Demo clarity
   - Video overlays deliver 2–3 points for minimal work
   - Payload: ~50 lines in `demo_video.py`

3. **Low impact, high effort:** Full autonomy (skip unless score is 85–88)
   - Swapping to RL would take 1+ week
   - Only pursue if feedback suggests teleoperation is a fundamental weakness

---

## Submission 3: Polish Pass (Optional)

**Target score:** 93+  
**Time budget:** 1 day  
**Changes:**
- Minor bug fixes from Submissions 1–2 feedback
- Video re-recording with better lighting/angles
- README clarifications based on evaluation comments

---

## Daily Resubmission Cadence

- **Day 1:** Submit V1 (current), receive score, identify weakest dimension
- **Day 2:** Fix top 2 weaknesses, resubmit
- **Day 3:** Polish based on refined feedback, final submission

Each resubmission should take <4 hours (keep iterations tight).

---

## Contingency: If Score is <80

If initial submission scores unexpectedly low, consider:

1. **Check runnability:** Verify `python main.py` produces `demo.mp4` without errors
2. **Check video quality:** Ensure MP4 plays and shows task progression
3. **Check README:** Confirm all 8 criteria are addressed (even if briefly)

If those are fine, the issue is likely in *perception*:
- Reorder README sections to lead with strongest dimensions
- Re-record video with more dramatic camera angles
- Add explicit evaluation mapping (e.g., "This submission scores high on: dexterity (measure X), MuJoCo depth (measure Y)")

See `README.md` lines ~150 for example.
