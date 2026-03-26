# Helicopter Flight Engine Framework (HEF) — Knowledge Base

## Project Overview

Pluggable flight engine framework for helicopter mods. Ships with the FBW (fly-by-wire) engine. Companion mod for the UH-1B Helicopter mod (Workshop ID: 3409723807) and its B42 fix (Workshop ID: 3688683236). The original helicopter movement uses `moveVehicle()` which relies on Java reflection APIs (`getClassFieldVal`, `getClassField`, `getNumClassFields`) that are gated behind `Core.debug` mode. The FBW engine replaces the debug-dependent movement system with a physics-force-based approach that works without debug mode, enabling helicopter flight on dedicated servers.

### Engine Framework (Strategy Pattern)
- `IFlightEngine` defines the interface + registry. Engines register at load time.
- `HeliSimService` is a thin dispatcher: resolves engine from sandbox setting, delegates all calls.
- `HeliMove` builds a `ctx` table per frame, passes to engine via HeliSimService. Reads results.
- Sandbox setting `HEF.FlightEngine` (enum) selects the active engine. HeliConfig maps index → name.
- Each engine owns its own sandbox namespace (e.g., `SandboxVars.FBW.*`).
- Chat command `/hef` auto-discovers tunables and commands from the active engine.

---

## Architecture Overview

### Original (Teleport-Based, Debug Required)
- Uses debug reflection to access `vehicle.jniTransform` (a Bullet physics Transform)
- Directly modifies `jniTransform.origin` (position) by adding deltas
- Calls `vehicle:setWorldTransform()` which teleports the vehicle
- Vehicle has **zero velocity at all times** — pure position manipulation
- No momentum, no inertia, instant direction changes

### New (Fly-By-Wire, No Debug Required)

**Tilt-as-Desired MPC** — the tilt angle IS the throttle:
- **Simulation model** (`sim_pos`, `sim_vel`) tracks where the helicopter SHOULD be (pure math, no Bullet)
- `sim_vel` is set DIRECTLY from `computeHorizontalTargets()` output (tilt angle x hspeed)
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
`fwdPzX = R02, fwdPzY = R22`. Passed to HeliDirection as a parameter. Always correct,
heading-independent, no calibration needed.

**Failed approaches** (historical):
1. Offset-based (`_yawToWorldOffset`): recalibrated every frame → cumulative error after turns
2. Tracked 2D vector with Euler delta: Euler deltas summed to near-zero during prolonged rotation
3. Tracked 2D vector with getWorldPos recalibration: pitch/roll contamination during auto-leveling
4. Single `_ourQuat` with `extractYaw` decomposition: branch ambiguity at degenerate Euler init → 156° snap

---

## Stopping System

### Tilt-Based MPC Braking (during auto-leveling)
- When keys released, tilt auto-levels -> desired velocity shrinks
- Sim model follows tilt -> sim decelerates
- Actual overshoots sim -> negative position error -> opposing force
- This phase uses full PD correction (pgain=7)

### Error-Based PD/Damping Transition
- PD correction stays active as long as position error > 0.5m (regardless of tilt state)
- During braking: sim stops advancing, actual overshoots → negative error → PD pushes back (smooth)
- Velocity damping only activates when error ≤ 0.5m (final stop phase)
- `FINAL_STOP_GAIN = 0.3` (tunable via `/hef fstopgain`), capped at 0.8
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

**Heading interaction**: The forward direction is always extracted from `_ourQuat`'s rotation
matrix. When yaw changes (A/D rotation or re-anchor), the forward direction updates
automatically from the matrix — no separate heading calibration needed.

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

### Invisible Wheels (UH1BHuey.txt override)
- 4 wheels with no model, zero suspension/friction
- Bypasses 0-wheel velocity dampener (0.1x per frame)

### Other: `brakingForce=0`, `stoppingMovementForce=0`, `steeringClamp=0`

---

## Tunable Parameters

### Via `/hef` Chat Commands

**Engine tunables** (auto-discovered from active engine's `getTunables()`):
| Command | Default | Description |
|---------|---------|-------------|
| `/hef pgain <v>` | 7.0 | Position error P gain |
| `/hef dgain <v>` | 0.3 | Position error D gain (damping) |
| `/hef maxerr <v>` | 10.0 | Max position error (tanh saturation, meters) |
| `/hef yawgain <v>` | 0.9 | Yaw correction gain |
| `/hef autolevel <v>` | 1.0 | Auto-leveling speed multiplier |
| `/hef fstopgain <v>` | 0.3 | Velocity damping strength (0.3=smooth, 0.8=snappy) |

**Sandbox params** (from HeliConfig, persisted per-save):
| Command | Default | Description |
|---------|---------|-------------|
| `/hef gravity <v>` | 9.80 | Gravity compensation estimate |
| `/hef kp <v>` | 8.0 | Vertical PD gain |
| `/hef hspeed <v>` | 450 | Max horizontal speed (m/s at max tilt) |
| `/hef ascend <v>` | 8.0 | Ascend speed (Bullet Y/s) |
| `/hef descend <v>` | 14.0 | Descend speed |
| `/hef fall <v>` | 24.5 | Engine-off fall speed |
| `/hef deadfall <v>` | 35.0 | Engine-dead fall speed |

**Note**: Existing saves retain old sandbox defaults. Use `/hef` commands to override per-session.

---

## Module Responsibilities

### Models/ — Shared OOP data structures (`shared/HEF/Models/`)

**`Quaternion.lua`** — Metatable-based quaternion class
- `new()`, `identity()`, `fromAxisAngle()`, `fromEuler()`
- `multiply()`, `__mul`, `conjugate()`, `normalize()`, `length()`, `unpack()`
- Zero-alloc: `multiplyInto()`, `conjugateInto()`, `_temp1/_temp2`
- Conversion: `toMatrixComponents()` (9 numbers), `toMatrix()` (RotationMatrix), `matrixToEuler()` (static)
- **Changes when**: math operations needed by new engines

**`RotationMatrix.lua`** — 3x3 rotation matrix with semantic accessors
- `new()`, `fromQuaternion(q)`
- Semantic: `getForwardPzX()` (R02), `getForwardPzY()` (R22), `getVertical()` (R12), `getRightVertical()` (R10)
- PZ convention: Row0=PzX, Row1=PzZ(vertical), Row2=PzY(horizontal). Y/Z SWAPPED.
- **Changes when**: matrix operations needed by new engines

### Engines/ — Flight engine implementations (`shared/HEF/Engines/`)

**`IFlightEngine.lua`** — Interface definition + registry
- `REQUIRED_METHODS` — validated at register time (crash early on missing method)
- `register(name, engine)`, `get(name)`, `getRegisteredNames()`
- **Changes when**: interface contract changes (new mandatory method)

### Engines/FBW/ — Fly-by-wire engine (`shared/HEF/Engines/FBW/`)

**`FBWEngine.lua`** — Facade implementing IFlightEngine
- `update(ctx)` → results table (horizontal + vertical + rotation)
- `updateGround(ctx)` → {liftoff, displaySpeed}
- `applyCorrectionForces(vehicle)` — 0-frame delay path
- Lifecycle, tunables, sandbox options, debug state, commands
- Registers itself: `IFlightEngine.register("FBW", FBWEngine)`
- **Changes when**: FBW orchestration flow changes

**`FBWOrientation.lua`** — Quaternion-state yaw/tilt (uses Models/Quaternion)
**`FBWFilters.lua`** — Composable filter pipeline (stateless)
**`FBWRawSim.lua`** — Simulation advance (position + velocity)
**`FBWErrorTracker.lua`** — History buffer + tanh error saturation
**`FBWForceComputer.lua`** — PD correction + thrust force math
**`FBWYawController.lua`** — Yaw MPC (simYaw tracking + re-anchor)
**`FBWInputProcessor.lua`** — Key → rotation deltas
**`FBWFlightModel.lua`** — Vertical flight behavior

### Util/ — Foundational utilities + config (`shared/HEF/Util/`)

**`HeliUtil.lua`** — Shared utilities
- `toLuaNum(v)`, `normalizeAngle(angle)`
- **Changes when**: Kahlua coercion behavior changes

**`HeliConfig.lua`** — Centralized configuration
- Single source of truth for all tuning parameters (sandbox defaults + runtime overrides)
- `get(shorthand)` — reads: runtime override → SandboxVars → hardcoded default
- `set(shorthand, value)` — session-only override
- `getParamDefs()` — returns PARAMS/PARAM_ORDER for HeliDebugCommands display
- Named constants for all flight model magic numbers (thresholds, margins, multipliers)
- Physics constants: `VEL_FORCE_FACTOR`, `PD_ERROR_THRESHOLD`
- `TARGET_FPS = 90` and `MIN_FPS = 10` — shared by all modules (no local duplicates)
- **Changes when**: new tunable parameter added, default value changes

**`HeliCompat.lua`** — Handler conflict resolution
- pcall wrapper for `getNumClassFields` (returns 0 instead of throwing in non-debug mode)
- Does NOT override `GetHeliType` (would break library's radial menu)
- **Changes when**: library conflict behavior changes

**`HeliTerrainUtil.lua`** — World geometry queries
- `getNowMaxZ()`, `isBlocked()`, `hasFlag()`, `sqobject()`
- Per-frame isBlocked cache (`invalidateBlockedCache()`) — eliminates redundant 132-square scans
- **Changes when**: PZ changes flag names or grid square API

### Adapters/ — Bullet physics I/O (`shared/HEF/Adapters/`)

**`HeliForceAdapter.lua`** — Force output + sub-steps
- `applyForceImmediate(vehicle, fx, fy, fz)` — Y/Z swap + applyImpulseGeneric
- `getSubStepsThisFrame()` → subSteps, physicsDelta (accumulator-based)
- `getLastSubSteps()`, `resetPhysicsTime()`
- **Changes when**: PZ physics API changes, force application path changes

**`HeliVelocityAdapter.lua`** — Velocity reading + smoothing
- `getVelocity(vehicle)` → smoothVelX, rawVelY, smoothVelZ (stale-read detection via position delta)
- `getPositionDeltaSpeed()` → m/s (ground truth, immune to stale Bullet reads)
- `resetSmoothing()`
- **Changes when**: PZ exposes direct velocity setter

### Debug/ — Debug logging + chat commands (`client/HeliAbility/Debug/`)

**`HeliDebug.lua`** — Debug state + logging
- `captureState()`, `periodicLog()`, `logEngineOff()`
- Flight data CSV recorder: `startRecording()`, `stopRecording()`, `writeFlightFrame()`
- **Changes when**: debug output format changes

**`HeliDebugCommands.lua`** — `/hef` runtime tuning
- Auto-discovers tunables from active engine via `HeliSimService.getTunables()`
- Auto-discovers commands from active engine via `HeliSimService.getCommands()`
- Fallback: HeliConfig params (sandbox-level tuning)
- **Changes when**: framework command structure changes (engine tunables auto-discovered)

### Root — Orchestrator + gameplay (`client/HeliAbility/`)

**`HeliSimService.lua`** — Thin dispatcher (engine-agnostic)
- Resolves engine from `HeliConfig.getEngineName()` → `IFlightEngine.get(name)`
- Every public method delegates 1:1 to active engine
- **Changes when**: interface contract changes (new method added)

**`HeliMove.lua`** — Orchestrator (thin controller)
- Builds `ctx` table once per frame: {vehicle, playerObj, keys, fpsMultiplier, heliType, curr_z, nowMaxZ, tempVector2}
- Passes ctx to engine via HeliSimService, reads results
- Explicit flight state machine: `inactive → warmup → airborne` (3 states)
- Module lifecycle: `onFlightInit(fn)` / `onFlightCleanup(fn)` registration for extensions
- `helicopterMovementUpdate()` (OnTick), `helicopterCorrectionUpdate()` (OnTickEvenPaused)
- Handles gameplay side-effects: engine dead → ISVehicleMenu.onShutOff, displaySpeed → setSpeedKmHour
- The ONLY file that registers event handlers
- **Changes when**: framework orchestration flow changes
- **Adding new modules**: call `HeliMove.onFlightInit(fn)` at load time, no HeliMove edit needed

**`HeliAuxiliary.lua`** — Ghost mode, lights, gas, damage
- `updateGhostMode()`, `updateNightLight()`, `consumeGas()`, `applyWallDamage()`, `cleanup()`
- **Changes when**: gameplay features change (light color, gas rate, etc.)

### `client/PanzerAbility/OnHitGround.lua` — Weapon damage (verbatim)

---

## File Structure

```
HeliFlightEngineFramework/
  KNOWLEDGE.md
  Contents/mods/HeliFlightEngineFramework/
    mod.info
    42.0/
      mod.info
      media/
        sandbox-options.txt
        lua/
          shared/
            HEF/
              Models/
                Quaternion.lua          ← OOP quaternion (shared by all engines)
                RotationMatrix.lua      ← OOP 3x3 matrix with PZ semantic accessors
              Util/
                HeliUtil.lua
                HeliConfig.lua          ← + getEngineName() engine selector
                HeliCompat.lua
                HeliTerrainUtil.lua
              Engines/
                IFlightEngine.lua       ← interface + registry
                FBW/
                  FBWEngine.lua         ← facade implementing IFlightEngine
                  FBWOrientation.lua    ← uses Models/Quaternion
                  FBWFilters.lua
                  FBWRawSim.lua
                  FBWErrorTracker.lua
                  FBWForceComputer.lua
                  FBWYawController.lua
                  FBWInputProcessor.lua
                  FBWFlightModel.lua
              Adapters/
                HeliForceAdapter.lua
                HeliVelocityAdapter.lua
            Translate/EN/
              Sandbox_EN.txt
          client/
            HeliAbility/
              HeliSimService.lua        ← thin dispatcher (delegates to active engine)
              Debug/
                HeliDebug.lua
                HeliDebugCommands.lua   ← auto-discovers tunables/commands
              HeliMove.lua              ← builds ctx, reads results
              HeliAuxiliary.lua
            PanzerAbility/
              OnHitGround.lua
        scripts/vehicles/UH1BHuey/
          UH1BHuey.txt
```

### Load Order

PZ loads `shared/` before `client/`, directories recursively. With subdirectories,
exact traversal order is filesystem-dependent. To eliminate all load-order concerns:
**no file-scope `local x = Module.y` for cross-module refs in Core/ or Adapters/ files**.
Instead, access globals inside function bodies. Util/ files define their globals at file
scope as before — they have no cross-module dependencies.

All cross-module references are inside function bodies (called after all files load).
PZ completes all file loading before the game loop starts any ticks.

### Dependency Graph (layered, no cycles)

```
  ┌── Models/ ──────────────────────┐
  │ Quaternion                      │ (shared by all engines)
  │ RotationMatrix                  │
  └─────────────────────────────────┘
                  │
  ┌── Util/ ────────────────────────┐
  │ HeliUtil                        │
  │ HeliConfig                      │ (read by all + engine selector)
  │ HeliCompat                      │
  │ HeliTerrainUtil                 │
  └─────────────────────────────────┘
                  │
  ┌── Engines/IFlightEngine ────────┐
  │ Interface + registry            │
  └─────────────────────────────────┘
                  │
  ┌── Engines/FBW/ ─────────────────┐
  │ FBWEngine (facade)              │ → registers with IFlightEngine
  │ FBWOrientation, FBWFilters,     │
  │ FBWRawSim, FBWErrorTracker,     │
  │ FBWForceComputer, FBWYawCtrl,   │
  │ FBWInputProcessor, FBWFlightMdl │
  └─────────────────────────────────┘
                  │
  ┌── Adapters/ (Bullet I/O) ──────┐
  │ HeliForceAdapter                │
  │ HeliVelocityAdapter             │
  └─────────────────────────────────┘
                  │
  ┌── HeliSimService (dispatcher) ──┐
  │ Resolves engine, delegates 1:1  │
  └─────────────────────────────────┘
                  │
  ┌── Debug/ ───────────────────────┐
  │ HeliDebug                       │
  │ HeliDebugCommands               │ (auto-discovers from engine)
  └─────────────────────────────────┘
                  │
          ┌───────┴───────┐
          │   HeliMove    │  (orchestrator + events)
          │ HeliAuxiliary │  (gameplay side-effects)
          └───────────────┘
```

No mutual runtime dependencies. Each layer only calls downward.
Engines are self-contained: register at load time, discovered via IFlightEngine.

### Architectural Patterns

**Layered Architecture** — Code is organized into layers with strict downward-only dependencies:
Util → Core → Adapters → Flight → Debug → Root. Core modules are pure math (no game API),
Adapters handle Bullet I/O, and the Facade orchestrates everything with zero computation.

**Strategy Pattern** — `IFlightEngine` defines the contract. Engines register at load time.
`HeliSimService` dispatches to the active engine. `HeliMove` never sees the engine directly.
Adding a new engine: create `Engines/NewEngine/`, implement interface, register, add sandbox enum option.

**Centralized Config** — `HeliConfig.lua` is the single source of truth for sandbox params and named constants.
Engine-specific runtime tunables are owned by the engine via `getTunables()/setTunable()`.
`HeliConfig.getEngineName()` maps the sandbox enum to an engine name string.

**Keys as Parameter** — HeliMove reads all keys once per frame into a `ctx` table, passes to engine.
No module calls `isKeyDown()` directly except HeliMove's `readKeys()`.

**Dispatcher Pattern** — HeliSimService is a thin dispatcher (zero computation). Resolves engine once,
delegates all calls 1:1. HeliMove only talks to HeliSimService.

**Module Lifecycle** — `HeliMove.onFlightInit(fn)` and `HeliMove.onFlightCleanup(fn)` allow modules to
self-register initialization/cleanup callbacks. Adding a new subsystem (e.g., HeliTurbulence) doesn't
require editing HeliMove — just call `HeliMove.onFlightInit(myInitFn)`.

**Explicit Flight State** — `_flightState` is one of: `inactive`, `warmup`, `airborne`.
Ground and engine-off paths set `_flightState = inactive` (triggers re-init next frame).

**Per-Frame Cache** — `HeliTerrainUtil.invalidateBlockedCache()` eliminates redundant isBlocked grid scans
between HeliInputProcessor and HeliSimService wall-blocking in the same frame.

### Change Scenario Matrix

| Change scenario | Files affected | Folder |
|----------------|---------------|--------|
| New sandbox parameter | HeliConfig + sandbox-options.txt | Util/ |
| Add new flight engine | New Engines/X/ folder, implement IFlightEngine, add sandbox enum | Engines/ |
| FBW PD controller tuning | FBWEngine tunables (auto-discovered by /hef) | Engines/FBW/ |
| FBW rotation feel / key mappings | FBWInputProcessor only | Engines/FBW/ |
| FBW quaternion math | FBWOrientation (uses Models/Quaternion) | Engines/FBW/ |
| FBW add filter (turbulence) | FBWFilters + FBWEngine (add stage) | Engines/FBW/ |
| FBW swap sim model | FBWRawSim only | Engines/FBW/ |
| Shared quaternion math | Quaternion.lua or RotationMatrix.lua | Models/ |
| PZ exposes velocity setter | HeliVelocityAdapter only | Adapters/ |
| Force application changes | HeliForceAdapter only | Adapters/ |
| Wall collision detection | HeliTerrainUtil only | Util/ |
| Gas consumption, lights | HeliAuxiliary only | root |
| New gameplay system | New file + HeliMove.onFlightInit | root |

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

- **Hover stability**: Zero drift, zero horizontal speed for 20+ seconds. Gravity comp avg vY < 0.02.
- **Yaw hard lock**: 0.000° heading drift across 3000+ straight-flight frames.
- **Descent from hover**: 5/8 descents perfectly clean (zero horizontal thrust). Noise floor effective.
- **Braking**: Error-based PD transition + soft anchor. PD stays active during deceleration (smooth). (NEEDS RE-VERIFICATION with current code)
- **Thrust direction clamping**: 30° max lead angle works — delta locked at exactly 30° during sustained turns.
- **Sim virtual inertia**: Sim velocity blends at 5%/frame. Eliminates instant velocity mismatch on re-anchor.
- **Single center wheel**: Dampener bypass works (wheelCount=1 > 0). Reduced torque source.
- **Direction reversal unclamped**: Thrust clamping skipped when angle > 90° from movement (direction reversal). (NEEDS IN-GAME VERIFICATION)
- **Separated yaw/tilt rotation**: `_yawDeg` (scalar) + `_tiltQuat` (quaternion). No Euler feedback loop, no gimbal lock, no branch ambiguity. Init yaw via forward-vector projection. (NEEDS IN-GAME VERIFICATION)
- **Error-based PD/damping**: PD stays active during deceleration (error > 0.5m). No harsh force transition. (NEEDS IN-GAME VERIFICATION)
- **Soft anchor**: Replaces hard sim-snap. Gradual blend at low speed, no discontinuity. (NEEDS IN-GAME VERIFICATION)

---

## Architecture History

**Phase 1 — Layered restructuring**: Entangled modules (HeliRotation, HeliDirection,
HeliSimulation, HeliForceController) restructured into Util → Core → Adapters → Flight → Debug layers.
Plan: `C:\Users\butt3rkeks\.claude\plans\fluttering-discovering-spark.md`

**Phase 2 — HEF framework**: Core/Flight modules moved into `Engines/FBW/` (strategy pattern).
IFlightEngine interface, thin HeliSimService dispatcher, Models/ shared data structures.
Plan: `C:\Users\butt3rkeks\.claude\plans\nifty-munching-pearl.md`

Benefits:
- Swap entire flight engine via sandbox setting
- New engines are self-contained folders, no framework edits needed
- Shared Models (Quaternion, RotationMatrix) reusable across engines
- Auto-discovery of tunables and commands via interface
- HeliMove is engine-agnostic — builds ctx, reads results

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
