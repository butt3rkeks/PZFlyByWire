# HEF Technical Internals

Deep technical reference for contributors working on the framework or FBW engine internals.
For the engine authoring API, see [DEVELOPER.md](DEVELOPER.md). For configuration, see [ADMIN.md](ADMIN.md).

---

## Architecture Overview

### Original (Teleport-Based, Debug Required)
- Uses debug reflection to access `vehicle.jniTransform` (a Bullet physics Transform)
- Directly modifies `jniTransform.origin` (position) by adding deltas
- Calls `vehicle:setWorldTransform()` which teleports the vehicle
- Vehicle has **zero velocity at all times** — pure position manipulation
- No momentum, no inertia, instant direction changes

### New (Fly-By-Wire, No Debug Required)

**Tilt-as-Desired PD with Reference Model** — the tilt angle IS the throttle:
- **Simulation model** (`sim_pos`, `sim_vel`) tracks where the helicopter SHOULD be (pure math, no Bullet)
- `sim_vel` is set DIRECTLY from `computeHorizontalTargets()` output (tilt angle x maxHorizontalSpeed)
- When keys released, auto-leveling gradually reduces tilt -> `sim_vel` naturally decays to zero
- **Visual tilt matches physics deceleration** — no separate accel/decel model
- **Position error** (`sim_pos - actual_pos`) drives correction forces via PD controller
- **Soft saturation**: `tanh(rawError / maxError) * maxError` applied to the ERROR, not to `sim_pos`
- **All forces via `applyImpulseGeneric`** — unified force path, no more addImpulse

**Three key fixes for stable flight:**
1. **Separated yaw/tilt rotation** — `_yawDeg` (scalar) + `_tiltQuat` (quaternion). No Euler feedback loop. Bullet's angular physics is overwritten each frame. No gimbal lock at any heading.
2. **Yaw hard lock** — `_yawDeg = _simYaw` (scalar assignment) during straight flight, preventing heading drift. No quaternion extraction needed.
3. **Forward direction from quaternion** — extracted from composed orientation matrix (`yawQ * _tiltQuat`), heading-independent and gimbal-free

---

## Frame Timing — Verified

**Source-verified** from IngameState.java, IsoWorld.java, WorldSimulation.java, BaseVehicle.java:

```
Frame N:
  IngameState.updateInternal():
    L1280: OnTickEvenPaused                          <- before physics (unconditional)
    L1454: pause check (SP blocks if paused, MP always passes)
    L1470: IsoWorld.instance.update()
      IsoWorld.updateInternal():
        L2917: WorldSimulation.instance.update()     <- PHYSICS STEP
          updatePhysic():
            L89: sub-step loop {
              L94: applyAccumulatedImpulsesFromHitObjects <- FORCES PROCESSED HERE
              L98: Bullet.stepSimulation(0.01)
            }
          L130-228: getVehiclePhysics()              <- reads back jniLinearVelocity, jniTransform
        L2973: this.updateWorld()
          -> BaseVehicle.update()
            L3582: updateVelocityMultiplier()
    L1497: this.onTick()                             <- OUR LUA OnTick RUNS HERE
```

### Force Timing

| Source | API | Event | Delay | Used for |
|--------|-----|-------|-------|----------|
| Horizontal corrections | `applyImpulseGeneric` | `OnTickEvenPaused` | **0 frames** | Position error PD correction |
| Vertical thrust | `applyImpulseGeneric` | `OnTick` | **1 frame** | Ascend/descend/hover + gravity comp |
| Ground forces | `applyImpulseGeneric` | `OnTick` | **1 frame** | Liftoff, velocity zeroing |

All forces use `applyImpulseGeneric` (impulsesFromHitObjects path). The deferred `addImpulse` path was removed — its 2-frame delay caused vertical oscillation with gravity compensation.

### Force Limit (verified from BaseVehicle.java)
```java
float limit = 1500.0F * this.getFudgedMass();  // = 1,560,000 for mass 1040
if (force.lengthSquared() > limit * limit) {
    float limitScalar = limit / force.length();
    force.mul(limitScalar);  // CLAMP, not drop
}
Bullet.applyCentralForceToVehicle(force.x * 30, force.y * 30, force.z * 30);  // x30 AFTER limit
```
Our forces never exceed ~13% of the limit. Safe headroom.

---

## Vertical Thrust — Dual-Rate Gravity Compensation

`computeThrustForces()` handles vertical PD + gravity compensation. `getSubStepsThisFrame()` returns
two values: `(subSteps, physicsDelta)`.

**Normal path (subSteps > 0):** Both PD and gravity scale by integer `subSteps`. This matches
Bullet's discrete sub-step processing. Covers all frames at ≤90 FPS.

**High FPS fallback (subSteps == 0, physicsDelta > 0):** At ≥100 FPS, some frames accumulate
less than one full 0.01s sub-step. The old code skipped ALL forces on these frames, but Bullet's
internal gravity acts continuously. Gravity compensation now uses fractional `physicsDelta / 0.01`
on these frames. PD correction is still skipped (negligible error at 120+ FPS intervals).

---

## Rotation System

### Separated Yaw + Tilt Architecture
- `_yawDeg` (scalar): heading in degrees, PZ convention. No gimbal ambiguity.
- `_tiltQuat` (quaternion): pitch/roll only. Always near identity when level.
- Composed for output: `fullQ = fromAngleAxis(rad(_yawDeg), 0,1,0) * _tiltQuat`
- Initialized once: `_yawDeg` from forward-vector projection (branch-unambiguous), `_tiltQuat = inv(yawQ) * fromEulerAngles(...)`
- Pitch/roll corrections: `_tiltQuat = _tiltQuat * nqx * nqz` (body-frame, right-multiply)
- Yaw corrections: `_yawDeg += ay` (scalar, no quaternion)
- Body-frame pitch/roll extracted from `_tiltQuat`'s matrix (heading-independent, gimbal-free)
- Euler conversion only at output: `toEulerAngles(toRotationMatrix(fullQ))` → `setAngles`
- **No Euler feedback loop** — Bullet's angular physics is overwritten each frame by setAngles

### Why Quaternion State (Euler Feedback Loop — FAILED)
Previous approach: `getAngleX/Y/Z → fromEuler → quat × corrections → toEuler → setAngles → Bullet → getAngle`
- At headings near ±90°, Euler Y approaches ±90° (gimbal lock)
- `toEulerAngles` decomposition redistributes body-pitch across Euler X and Z
- Roll auto-leveling fights the redistributed pitch, reducing effective tilt
- Caused: sudden stops during 180°+ turns, speed cap at ±90° headings (~40% of max)
- Also caused: Euler delta tracking failure (accumulated deltas summed to near-zero during prolonged rotation)

### Matrix Row Mapping (verified empirically)
```
Row 0 = PZ world X
Row 1 = PZ world Z (vertical/height)  ← Y/Z SWAPPED vs PZ convention
Row 2 = PZ world Y (horizontal)       ← Y/Z SWAPPED vs PZ convention
```
- Forward PzX = R02, Forward PzY = R22, Forward Vert = R12
- Right PzX = R00, Right PzY = R20, Right Vert = R10
- Body pitch (angleZ equivalent) = `acos(clamp(R12, -1, 1))`
- Body roll (angleX equivalent) = `acos(clamp(R10, -1, 1))`

### Why Yaw is a Scalar, Not in the Quaternion

To do yaw hard lock on a quaternion, you must: (1) extract current yaw, (2) remove it,
(3) replace with desired yaw. Step 1 uses `atan2(2*(w*y+x*z), 1-2*(y²+z²))` which suffers
from **Euler branch ambiguity**: the same physical orientation can map to different yaw values
depending on the pitch/roll state. On init, `_ourQuat` is built from PZ's potentially degenerate
Euler angles (X≈180, Y=12, Z≈-180), and `extractYaw` returns 168° instead of 12°. Decomposing
with the wrong yaw corrupts the tilt → orientation snaps by 156°.

**Structural fix**: yaw as a scalar (`_yawDeg`) and tilt as a separate quaternion (`_tiltQuat`).
Yaw lock = `_yawDeg = _simYaw` (scalar assignment — no extraction, no decomposition, no branch
ambiguity). The two are only composed at the output for setAngles, where we never decompose back.

This makes extraction errors **structurally impossible** — there is no yaw extraction anywhere
in the code. This approach is the **gold standard in flight sims** (separate heading from tilt).

### Alternative: Forward-Vector Yaw Extraction (viable but not used)

If a single quaternion were needed, yaw CAN be extracted without branch ambiguity by projecting
the forward vector onto the horizontal plane:
```
fwd_x = 2*(q.x*q.z + q.w*q.y)
fwd_z = 1 - 2*(q.x*q.x + q.y*q.y)
yaw = atan2(fwd_x, fwd_z)
```
This measures the **physical nose direction**, not an Euler parameterization. No branch ambiguity
(a forward vector is unique). Only singular at pitch = ±90° (nose straight up/down — irrelevant
for a helicopter). Equivalent to what we compute via `R02, R22` from the rotation matrix.

The standard `atan2(2*(w*y+x*z), 1-2*(y²+z²))` formula extracts the CANONICAL Euler Y angle,
which normalizes pitch to [-90°,+90°]. When the input quaternion was built from non-canonical
Euler (pitch outside that range, e.g., X=180,Y=12,Z=-180), the extraction returns the canonical
equivalent (Y=168) — different yaw, same physical orientation. This is mathematically correct
but operationally wrong for yaw lock.

The **swing-twist decomposition** (`twist = normalize(0, q.y, 0, q.w)`) is the rigorous version
of forward-vector projection. Gives the twist quaternion directly for yaw replacement.

Sources: Allen Chou (swing-twist sterp), Marc B. Reynolds (quaternion axis factoring),
aerospace literature (DTIC, NASA quaternion control architectures).

### Yaw Hard Lock
- During A/D rotation: `_yawDeg += ay` (scalar accumulation)
- During straight flight: `_yawDeg = _simYaw` (scalar assignment)
- `_simYaw` re-anchors from `_yawDeg` when A/D rotation stops
- No quaternion extraction needed at any point

### Direction Computation
Forward direction extracted from `composeOrientation()` (`yawQ * _tiltQuat`) rotation matrix:
`fwdPzX = R02, fwdPzY = R22`. Returned via results table to the framework. Always correct,
heading-independent, no calibration needed.

**Failed approaches** (historical):
1. Offset-based (`_yawToWorldOffset`): recalibrated every frame → cumulative error after turns
2. Tracked 2D vector with Euler delta: Euler deltas summed to near-zero during prolonged rotation
3. Tracked 2D vector with getWorldPos recalibration: pitch/roll contamination during auto-leveling
4. Single `_ourQuat` with `extractYaw` decomposition: branch ambiguity at degenerate Euler init → 156° snap

---

## Stopping System

### Tilt-Based Braking (during auto-leveling)
- When keys released, tilt auto-levels -> desired velocity shrinks
- Sim model follows tilt -> sim decelerates
- Actual overshoots sim -> negative position error -> opposing force
- This phase uses full PD correction (positionProportionalGain=7)

### Error-Based PD/Damping Transition
- PD correction stays active as long as position error > 0.5m (regardless of tilt state)
- During braking: sim stops advancing, actual overshoots → negative error → PD pushes back (smooth)
- Velocity damping only activates when error ≤ 0.5m (final stop phase)
- `FINAL_STOP_GAIN = 0.3` (tunable via `/hef finalStopDampingGain`), capped at 0.8
- Prevents the harsh PD→damping force transition that killed velocity in 2 frames

### Soft Anchor (replaces hard sim-snap)
- When no input and low speed: sim position blends toward actual at 10% per frame
- Prevents stale position error from accumulating during hover
- No discontinuity — smooth convergence over ~22 frames
- Doesn't prevent acceleration — blend stops when input resumes

### FA-Off (Flight Assist Off)
- `_hasTiltInput` distinguishes real tilt from FA-off velocity maintenance
- Above 1.5 m/s: free coast (zero force)
- Below 1.5 m/s (deadzone): velocity damping to allow full stop
- Sim re-anchored in deadzone to prevent stale error accumulation

---

## Altitude Limits

### Ceiling Taper
- Ascend speed linearly tapers from full at `MAX_ALTITUDE - CEILING_ZONE_HEIGHT` to zero at `MAX_ALTITUDE`
- `CEILING_ZONE_HEIGHT = 1.0` Z-level (mirrors landing zone)
- Prevents micro-oscillation at the ceiling (was a hard cutoff → hover snap → overshoot → repeat)

### Landing Zone
- Descent speed tapers in last `LANDING_ZONE_HEIGHT = 1.0` Z-levels above ground
- Minimum speed factor `LANDING_MIN_SPEED_FACTOR = 0.3` prevents descent from stopping entirely

---

## Heading Re-Anchor — Current State

**Sim position re-anchor**: When heading changes significantly (per-frame yaw delta exceeds
time-scaled threshold), the sim position resets to actual position. History is cleared.
Sim velocity is PRESERVED (not zeroed) — the sim inertia handles the direction transition
naturally, preventing violent force spikes from instant velocity mismatch.

**Yaw re-anchor**: When A/D is released (`_wasRotating → false`), `_simYaw` re-anchors from
`_yawDeg` (scalar assignment — no quaternion extraction needed).

**Threshold scaling**: `2.0° × (TARGET_FPS / actualFPS)` prevents triggering during normal
A/D rotation at low FPS.

**Heading interaction**: The forward direction is always extracted from `composeOrientation()`
(`yawQ * _tiltQuat`) rotation matrix. When yaw changes (A/D rotation or re-anchor), the
forward direction updates automatically from the matrix — no separate heading calibration needed.

---

## Kahlua (PZ Lua) — Critical Lessons

### Java Float -> Lua Number Coercion
**PZ's Kahlua VM does NOT auto-unbox Java Float objects to Lua numbers.** Methods like `vehicle:getX()`, `vehicle:getAngleY()`, `vehicle:getMass()` return Java Float objects that:
- Work in arithmetic (`getX() + 1`) — operator overloading
- **Fail** with `tonumber()` — returns `nil`
- **Fail** with comparison operators (`<`, `>`) — `__lt not defined`
- **`math.sqrt()` may return Java Float** — must coerce before comparison
- **`getPhysicsSecondsSinceLastUpdate()` returns Java Float** — must coerce before `>=` comparison (this caused vertical forces to never fire)

**Reliable conversion**: `toLuaNum(v) = tonumber(tostring(v)) or 0`

**All force computation functions coerce `vehicle:getMass()` and velocity values via `toLuaNum`.**

**Return type consistency**: All paths through `computeVerticalTarget()` return Lua numbers.
The freeMode path coerces `getVelocity()` via `toLuaNum` before returning.

### Kahlua Modulo
- C-style: `-5 % 30 = -5` (not 25). Fix: add modulus before `%`.

---

## Vehicle Script Modifications

### Invisible Wheel (UH1BHuey.txt override)
- Single invisible center wheel (FrontLeft at offset 0,−2.5,0), no model attached
- Bypasses 0-wheel velocity dampener (wheelCount > 0 → velocity only capped above 34 Bullet/s)
- Single center wheel eliminates asymmetric btRaycastVehicle ground contacts that caused torque/tilt drift with 4 spread wheels during descent
- Zero suspension/friction (suspensionStiffness=0, wheelFriction=0)

### Other: `brakingForce=0`, `stoppingMovementForce=0`, `steeringClamp=0`

For parameter reference and tuning, see [ADMIN.md](ADMIN.md).
For project structure, API reference, and engine authoring guide, see [DEVELOPER.md](DEVELOPER.md).

---

## Known Issues / Open Items

1. **Sim inertia causes pretzel after short turns** — The sim velocity (5%/frame inertia)
   continues in the turn direction after A/D release. During a 2-second turn, the sim
   accumulates velocity that takes 60+ frames to reverse. The helicopter swings past the
   intended heading, reverses, and overshoots again. May need faster inertia rate or
   turn-aware inertia that increases rate when heading changes.

2. **Velocity read oscillation** — Bullet's `getLinearVelocity` returns stale ~3.5 m/s on
   ~2/3 of frames while actual speed is 30+ m/s. Position-delta smoothing catches the worst
   cases but doesn't eliminate the oscillation entirely. Root cause is unknown — possibly
   Bullet read timing relative to sub-step boundaries. The soft anchor uses position-delta
   speed (immune to stale reads) for its low-speed detection.

3. **Visual jitter from setAngles teleport** — the teleport snaps orientation each frame.
   At high speed, micro-differences between frames are visible. Not a force issue.

4. **PZ grid Z vs Bullet altitude** — the helicopter is always on floor 0 in PZ's grid.
   Ghost mode prevents zombie interaction.

## Removed Features

- **Coordinated turn** — banking into turns (UP + A/D) added roll via `HeliConfig.COORDINATED_TURN_FACTOR`.
  Removed because it fed artificial roll into the tilt decomposition, making turn behavior unpredictable
  and amplifying the heading calibration corruption. Roll now always auto-levels during turns.

## Verified Working

**Pre-HEF (verified before restructuring):**
- **Hover stability**: Zero drift, zero horizontal speed for 20+ seconds. Gravity comp avg vY < 0.02.
- **Yaw hard lock**: 0.000° heading drift across 3000+ straight-flight frames.
- **Descent from hover**: 5/8 descents perfectly clean (zero horizontal thrust). Noise floor effective.
- **Thrust direction clamping**: 30° max lead angle works — delta locked at exactly 30° during sustained turns.
- **Sim virtual inertia**: Sim velocity blends at 5%/frame. Eliminates instant velocity mismatch on re-anchor.
- **Invisible wheel override**: Dampener bypass works (wheelCount=1 > 0). Single center wheel, zero suspension/friction.

**Needs re-verification after HEF restructuring:**
- Hover stability (Quaternion model swap: CustomQuaternion → Models/Quaternion)
- Yaw (FBWOrientation uses Quaternion.fromAxisAngle instead of CustomQuaternion.fromAngleAxis)
- Full 360° rotation without heading drift
- Vertical ascend/descend (FBWEngine absorbed vertical from HeliMove)
- Ground liftoff/landing (updateGround path)
- Engine-off fall (framework-level PD, no engine involvement)
- `/hef show`, `/hef positionProportionalGain 5`, `/hef reset` through dispatcher chain
- Sandbox UI: FlightEngine string field (type engine name, default "FBW")
- Braking / error-based PD transition / soft anchor
- FA-off (flight assist off) coast mode
- Flight data CSV recording (`/hef record`)

---

## Architecture History

**Phase 1 — Layered restructuring**: Entangled modules (HeliRotation, HeliDirection,
HeliSimulation, HeliForceController) restructured into Util → Core → Adapters → Flight → Debug layers.
Plan: fluttering-discovering-spark (Claude Code session)

**Phase 2 — HEF framework**: Core/Flight modules moved into `Engines/FBW/` (strategy pattern).
IFlightEngine interface (required + optional methods), thin HeliSimService dispatcher,
Models/ shared data structures, typed data contracts (HEFCtx, HEFCorrectionCtx, result classes).
Framework owns all per-frame reads (velocity, position, mass, terrain, keyboard, physics timing)
via `HEFContext.build()` — engines receive ctx, never call adapters directly.
Each engine owns its sandbox namespace. EmmyLua annotations on all engine-facing surfaces.
Plan: nifty-munching-pearl (Claude Code session)

Benefits:
- Swap entire flight engine via sandbox setting (`HEF.FlightEngine` string)
- New engines are self-contained folders, no framework edits needed
- Shared Models (Quaternion, RotationMatrix) reusable across engines
- Auto-discovery of tunables and commands via interface
- HeliMove is engine-agnostic — builds typed ctx, reads typed results
- IDE autocompletion for engine authors via EmmyLua `@class` on all data contracts
- Framework-owned reads prevent stale-read bugs from multiple adapter calls

---

## Unit Reference

| Measurement | Unit | Conversion |
|-------------|------|------------|
| Bullet velocity | m/s | x 3.6 = km/h |
| PZ tile | ~1 meter | 1 Bullet horizontal unit = 1 tile |
| Bullet Y (height) | meters | / 2.44949 = PZ Z level |
| `getCurrentSpeedKmHour()` | km/h | `jniLinearVelocity.length() x 3.6` |

---

## Research Sources

- Bullet Physics: [btRigidBody.h](https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/Dynamics/btRigidBody.h)
- Bullet Physics: [btRaycastVehicle](https://pybullet.org/Bullet/BulletFull/classbtRaycastVehicle.html)
- PZ Decompiled: `E:/Modding/zomboid/ZomboidDecompiler/bin/output/source/zombie/`
  - `vehicles/BaseVehicle.java` — setAngles, addImpulse, applyImpulseGeneric, updateVelocityMultiplier
  - `Lua/LuaManager.java` — debug-gated reflection APIs
  - `core/physics/Bullet.java` — native JNI method signatures
  - `core/physics/WorldSimulation.java` — physics sub-step loop
  - `gameStates/IngameState.java` — frame timing
- PZBullet64.dll analysis — version 1.0.0.28, Bullet SDK ~2.83-2.89
