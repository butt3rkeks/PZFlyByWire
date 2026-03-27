# Helicopter Flight Engine Framework (HEF) — Knowledge Base

## Project Overview

Pluggable flight engine framework for helicopter mods. Ships with the FBW (fly-by-wire) engine. Companion mod for the UH-1B Helicopter mod (Workshop ID: 3409723807) and its B42 fix (Workshop ID: 3688683236). The original helicopter movement uses `moveVehicle()` which relies on Java reflection APIs (`getClassFieldVal`, `getClassField`, `getNumClassFields`) that are gated behind `Core.debug` mode. The FBW engine replaces the debug-dependent movement system with a physics-force-based approach that works without debug mode, enabling helicopter flight on dedicated servers.

### Engine Framework (Strategy Pattern)
- `IFlightEngine` defines the interface + registry. Engines register at load time.
- `HeliSimService` is a thin dispatcher: resolves engine from sandbox setting, delegates all calls.
- `HeliMove` builds a `ctx` table per frame, passes to engine via HeliSimService. Reads results.
- Sandbox setting `HEF.FlightEngine` (string, default `"FBW"`) selects the active engine. Third-party mods can set any registered name.
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

---

## Tunable Parameters

There are two kinds of parameters, with different persistence:

### Engine Tunables (session-only, lost on game exit)

Set via `/hef <name> <value>`. Auto-discovered from the active engine's `getTunables()`.
These are runtime overrides for experimentation — they do NOT persist across sessions.

| Command | Default | Description |
|---------|---------|-------------|
| `/hef positionProportionalGain <v>` | 7.0 | Position error P gain |
| `/hef positionDerivativeGain <v>` | 0.3 | Position error D gain (damping) |
| `/hef maxPositionError <v>` | 10.0 | Max position error saturation (meters) |
| `/hef yawCorrectionGain <v>` | 0.9 | Yaw correction gain |
| `/hef autoLevelSpeed <v>` | 1.0 | Auto-leveling speed multiplier |
| `/hef finalStopDampingGain <v>` | 0.3 | Velocity damping strength (0.3=smooth, 0.8=snappy) |

### Sandbox Parameters (persisted per-save, set in sandbox options UI)

Defined in `sandbox-options.txt`, stored in the save file. `/hef` can override them
for the current session, but the sandbox default is restored on game restart.

| Sandbox field (FBW.*) | `/hef` shorthand | Default | Description |
|----------------------|-----------------|---------|-------------|
| GravityEstimate | gravity | 9.80 | Gravity compensation estimate |
| ResponsivenessGain | verticalGain | 8.0 | Vertical PD gain |
| MaxHorizontalSpeed | maxHorizontalSpeed | 90 | Max horizontal speed (m/s at max tilt) |
| AscendSpeed | ascend | 8.0 | Ascend speed (Bullet Y/s) |
| DescendSpeed | descend | 14.0 | Descend speed |
| GravityFallSpeed | fall | 24.5 | Engine-off fall speed |
| EngineDeadFallSpeed | engineDeadFallSpeed | 35.0 | Engine-dead fall speed |

---

## Module Responsibilities

### Core/Models/ — Shared OOP data structures (`shared/HEF/Core/Models/`)

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

### Type Classes — Framework data contracts (`shared/HEF/`)

**`HEFContext.lua`** — Per-frame context (OnTick phase)
- `@class HEFCtx` + `CTX_FIELDS` (runtime validators) + `build(vehicle, playerObj, tempVector2)`
- Reads: keyboard, velocity, terrain, wall blocking, position, mass, physics timing, angles
- Lazy reads (via `__index` + `rawset` cache): `fuelPercent`, `engineCondition` — deferred until first access so framework side-effects (e.g., consumeGas) are reflected
- Output closures: `applyForce(fx,fy,fz)`, `setAngles(x,y,z)`, `setPhysicsActive(active)` — engines write through these instead of calling adapters/game APIs directly
- Sub-types: `@class HEFKeys`, `@class HEFBlocked`
- **Changes when**: new data needed by engines (add field here + in builder)

**`HEFCorrectionCtx.lua`** — Correction-phase context (OnTickEvenPaused phase)
- `@class HEFCorrectionCtx` + `CTX_FIELDS` + `build(vehicle)`
- Reads: fresh post-physics velocity, mass
- Output closure: `applyForce(fx,fy,fz)`
- **Changes when**: correction path needs additional data

**`HEFUpdateResult.lua`** — Mandatory return from `update(ctx)` (`@class` + `new()`)
**`HEFGroundResult.lua`** — Mandatory return from `updateGround(ctx)` (`@class` + `new()`)
**`HEFTunable.lua`** — Return element from `getTunables()` (`@class` + `new()`)
**`HEFSandboxOptions.lua`** — Return from `getSandboxOptions()` (`@class` + `new()`, includes `HEFSandboxOption`)
- `namespace`: string — sandbox namespace (e.g. `"FBW"` → `SandboxVars.FBW.*`)
- `options[]`: field, type (`"double"`/`"integer"`/`"boolean"`/`"enum"`), default, min, max, desc
- **Important**: `getSandboxOptions()` is metadata for the debug/chat UI. Actual sandbox persistence
  requires matching entries in `sandbox-options.txt` (PZ's static DSL). These two systems must stay in sync.

**`HEFCommand.lua`** — Return element from `getCommands()` (`@class` + `new()`)
- `name`: string (e.g. `"recalibrate"`), `description`: string, `args`: string (e.g. `""` or `"<degrees>"`)

**`HEFEngineInfo.lua`** — Return from `getInfo()` (`@class` + `new()`)
- `name`: string (engine identifier, e.g. `"FBW"`), `version`: string (e.g. `"1.0"`), `description`: string

All type classes have EmmyLua annotations for IDE autocompletion. `HEF` prefix distinguishes them from PZ types.

### Engines/ — Flight engine implementations (`shared/HEF/Engines/`)

**`IFlightEngine.lua`** — Interface definition + registry
- `REQUIRED_METHODS` — validated at register time (crash early on missing method)
- `OPTIONAL_METHODS` — listed for discoverability, not validated (e.g. `applyCorrectionForces`)
- `register(name, engine)`, `get(name)`, `getRegisteredNames()`
- References type contracts: HEFContext, HEFCorrectionCtx, result classes
- **Changes when**: interface contract changes (new method or ctx field)

### Core/Toolkit/ — Optional building blocks (`shared/HEF/Core/Toolkit/`)

Reusable utilities for engine authors. Not required — engines can use raw math instead.

**`CoordUtil.lua`** — PZ coordinate system helpers
- `pzToStandard(x,y,z)` / `standardToPz(x,y,z)` — Y/Z swap
- `msToKmh(v)` / `kmhToMs(v)` — unit conversions

**`VelocityUtil.lua`** — Speed decomposition
- `horizontalSpeed(vx,vz)`, `totalSpeed(vx,vy,vz)`, `horizontalAngle(vx,vz)`, `decompose(vx,vy,vz)`

**`PDController.lua`** — Stateless PD controller
- `compute(err, errRate, Kp, Kd)`, `computeSaturated(...)` with tanh, `compute2DSaturated(...)`

**`SimModel2D.lua`** — Instance-based 2D sim (position + velocity with inertia)
- `new(targetFps)`, `:advance(desVx, desVz, dt, inertia)`, `:snapPosition(x,z)`, `:blendToward(x,z,blend)`, `:getState()`

**`ErrorTracker2D.lua`** — Instance-based ring buffer error tracker
- `new(historySize, lookback)`, `:record(actualX, actualZ, desiredX, desiredZ)`, `:getError(maxError)`, `:clearHistory()`

**`VerticalModel.lua`** — Default helicopter vertical behavior
- `computeTarget(keys, velY, currentAltitude, fuelPercent, engineCondition, freeMode, cfg)` → targetVelY, gravComp, vBraking, engineDead

**`GroundModel.lua`** — Default ground hold + liftoff
- `update(ctx, cfg)` → HEFGroundResult

### Engines/FBW/ — Fly-by-wire engine (`shared/HEF/Engines/FBW/`)

**`FBWHeliConfig.lua`** — FBW parameter definitions + typed getters
- Registers FBW params with `HeliConfig.registerParams()` at file scope
- Sandbox-tunable: gravity, engineDeadCondition, engineDeadFallSpeed, verticalGain, brake, accel, decel, ascend, descend, maxHorizontalSpeed, yawRotationSpeed
- Runtime-only: positionProportionalGain, positionDerivativeGain, maxPositionError, finalStopDampingGain, yawCorrectionGain, autoLevelSpeed
- Typed getters on HeliConfig (extension methods): `GetGravity()`, `GetAscend()`, etc.
- **Changes when**: FBW tuning parameter added/removed/renamed

**`FBWEngine.lua`** — Facade implementing IFlightEngine
- `update(ctx:HEFCtx)` → HEFUpdateResult (horizontal + vertical + rotation)
- `updateGround(ctx:HEFCtx)` → delegates to Toolkit/GroundModel
- `applyCorrectionForces(cctx:HEFCorrectionCtx)` — optional 0-frame delay path
- Uses ctx output closures: `ctx.applyForce()`, `ctx.setAngles()` (no direct adapter calls)
- Uses ctx read fields: `ctx.fuelPercent`, `ctx.engineCondition`, `ctx.angleX/Y/Z`, `ctx.positionDeltaSpeed`
- Lifecycle, tunables, sandbox options, debug state, commands
- Registers with deferred fallback: immediate if IFlightEngine loaded, else OnGameStart
- Uses Toolkit instances: `SimModel2D` for sim, `ErrorTracker2D` for error tracking
- **Changes when**: FBW orchestration flow changes

**`FBWOrientation.lua`** — Quaternion-state yaw/tilt (uses Models/Quaternion)
**`FBWFilters.lua`** — Composable filter pipeline (stateless)
**`FBWForceComputer.lua`** — PD correction + thrust force math
**`FBWYawController.lua`** — Yaw MPC (simYaw tracking + re-anchor)
**`FBWInputProcessor.lua`** — Key → rotation deltas (uses ctx.blocked, not HeliTerrainUtil)
**`FBWFlightModel.lua`** — Thin wrapper around Toolkit/VerticalModel (reads ctx fields, passes config)

### Core/Util/ — Foundational utilities + config (`shared/HEF/Core/Util/`)

**`HeliUtil.lua`** — Shared utilities
- `toLuaNum(v)`, `normalizeAngle(angle)`
- **Changes when**: Kahlua coercion behavior changes

**`HeliConfig.lua`** — Centralized configuration (framework params only)
- Framework params (HEF.*) defined here. Engine params registered via `registerParams()`.
- `get(shorthand)` — reads: runtime override → SandboxVars → hardcoded default (nil-guarded)
- `set(shorthand, value)` — session-only override (nil-guarded)
- `registerParams(params, order)` — engines register their own params at load time (see FBWHeliConfig)
- `getParamDefs()` — returns PARAMS/PARAM_ORDER (includes registered engine params)
- Typed getters: `GetMaxAltitude()`, `GetWarmupFrames()`, `GetWallDamageInterval()`, `GetEngineOffFallSpeed()`, `GetFallPdGain()`
- Named constants for all flight model magic numbers (thresholds, margins, multipliers)
- Physics constants: `VEL_FORCE_FACTOR`, `PD_ERROR_THRESHOLD`
- `TARGET_FPS = 90` and `MIN_FPS = 10` — shared by all modules (no local duplicates)
- **Changes when**: new framework parameter added, constant value changes

**`HeliCompat.lua`** — Handler conflict resolution
- pcall wrapper for `getNumClassFields` (returns 0 instead of throwing in non-debug mode)
- Does NOT override `GetHeliType` (would break library's radial menu)
- **Changes when**: library conflict behavior changes

**`HeliTerrainUtil.lua`** — World geometry queries
- `getNowMaxZ()`, `isBlocked()`, `hasFlag()`, `sqobject()`
- Per-frame isBlocked cache (`invalidateBlockedCache()`) — eliminates redundant 132-square scans
- **Changes when**: PZ changes flag names or grid square API

### Core/Adapters/ — Bullet physics I/O (`shared/HEF/Core/Adapters/`)

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
- Guards optional methods (`applyCorrectionForces`) with existence check
- Full EmmyLua annotations on all methods for IDE autocompletion
- **Changes when**: interface contract changes (new method added)

**`HeliMove.lua`** — Orchestrator (thin controller)
- Calls `HEFContext.build()` once per frame (velocity, position, mass, keys, terrain, physics timing)
- Passes HEFCtx to engine via HeliSimService, reads HEFUpdateResult/HEFGroundResult
- Calls `HEFCorrectionCtx.build()` on OnTickEvenPaused for the correction path
- Flight state init runs BEFORE ctx build (ensures velocity adapter is reset before first read)
- Explicit flight state machine: `inactive → warmup → airborne` (3 states)
- Engine-off fall: framework-level PD (engine-agnostic, no engine involved)
- Module lifecycle: `onFlightInit(fn)` / `onFlightCleanup(fn)` registration for extensions
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
HeliFlightEngineFramework/          ← project root (KNOWLEDGE.md, README.md, workshop.txt here)
  Contents/mods/HeliFlightEngineFramework/
    mod.info
    42.0/
      mod.info
      media/
        sandbox-options.txt
        lua/
          shared/
            HEF/
              HEFContext.lua            ← @class HEFCtx + CTX_FIELDS + build()
              HEFCorrectionCtx.lua     ← @class HEFCorrectionCtx + CTX_FIELDS + build()
              HEFUpdateResult.lua      ← @class + new()
              HEFGroundResult.lua      ← @class + new()
              HEFTunable.lua           ← @class + new()
              HEFSandboxOptions.lua    ← @class + new() (includes HEFSandboxOption)
              HEFCommand.lua           ← @class + new()
              HEFEngineInfo.lua        ← @class + new()
              Core/                    ← loads before Engines/ (C < E), guarantees globals
                Adapters/
                  HeliForceAdapter.lua
                  HeliVelocityAdapter.lua
                Models/
                  Quaternion.lua       ← OOP quaternion with convenience methods
                  RotationMatrix.lua   ← OOP 3x3 matrix with PZ semantic accessors
                Toolkit/               ← optional building blocks for engine authors
                  CoordUtil.lua        ← PZ Y/Z swap helpers, unit conversions
                  ErrorTracker2D.lua   ← ring buffer error tracking with derivative
                  GroundModel.lua      ← default ground hold + liftoff
                  PDController.lua     ← stateless PD with tanh saturation
                  SimModel2D.lua       ← 2D position/velocity sim with inertia
                  VelocityUtil.lua     ← speed decomposition helpers
                  VerticalModel.lua    ← default W/S/hover/fall vertical model
                Util/
                  HeliUtil.lua
                  HeliConfig.lua       ← framework params + registerParams() + getEngineName()
                  HeliCompat.lua
                  HeliTerrainUtil.lua
              Engines/
                IFlightEngine.lua      ← interface + registry (REQUIRED + OPTIONAL methods)
                FBW/
                  FBWHeliConfig.lua   ← FBW params + getters (registers with HeliConfig)
                  FBWEngine.lua        ← facade implementing IFlightEngine
                  FBWOrientation.lua   ← uses Models/Quaternion
                  FBWFilters.lua
                  FBWFlightModel.lua   ← thin wrapper around Toolkit/VerticalModel
                  FBWForceComputer.lua
                  FBWYawController.lua
                  FBWInputProcessor.lua
            Translate/EN/
              Sandbox_EN.txt
          client/
            HeliAbility/
              HeliSimService.lua       ← thin dispatcher (delegates to active engine)
              Debug/
                HeliDebug.lua
                HeliDebugCommands.lua  ← auto-discovers tunables/commands
              HeliMove.lua             ← orchestrator, builds ctx via HEFContext.build()
              HeliAuxiliary.lua
            PanzerAbility/
              OnHitGround.lua
        scripts/vehicles/UH1BHuey/
          UH1BHuey.txt
```

### Load Order

PZ loads `shared/` before `client/`, directories alphabetically depth-first.
Framework infrastructure is in `Core/` (C) which loads before `Engines/` (E),
guaranteeing all framework globals exist when engine files load. This holds
regardless of engine directory names.

Within Core/:  Adapters (A) < Models (M) < Toolkit (T) < Util (U).
No file-scope cross-module references within Core/ — all globals accessed in function bodies.

**FBWEngine registration**: `IFlightEngine.register("FBW", FBWEngine)` is the only
file-scope cross-module call in Engines/. Guarded: if `IFlightEngine` exists, registers
immediately; otherwise defers to `Events.OnGameStart` (all files loaded before events fire).

**FBWEngine toolkit instances**: `SimModel2D.new()` and `ErrorTracker2D.new()` at file scope
are safe because Core/Toolkit/ loads before Engines/FBW/.

### Dependency Graph (layered, no cycles)

```
  ┌── Core/Adapters/ ──────────────┐
  │ HeliForceAdapter                │ (Bullet I/O)
  │ HeliVelocityAdapter             │
  └─────────────────────────────────┘
                  │
  ┌── Core/Models/ ────────────────┐
  │ Quaternion, RotationMatrix      │ (shared by all engines)
  └─────────────────────────────────┘
                  │
  ┌── Core/Toolkit/ ───────────────┐
  │ CoordUtil, VelocityUtil,        │ (optional building blocks)
  │ PDController, SimModel2D,       │
  │ ErrorTracker2D, VerticalModel,  │
  │ GroundModel                     │
  └─────────────────────────────────┘
                  │
  ┌── Core/Util/ ──────────────────┐
  │ HeliUtil, HeliConfig,           │
  │ HeliCompat, HeliTerrainUtil     │
  └─────────────────────────────────┘
                  │
  ┌── HEF Types (data contracts) ──┐
  │ HEFContext, HEFCorrectionCtx,   │ (framework → engine)
  │ HEFUpdateResult, HEFGroundResult│ (engine → framework)
  │ HEFTunable, HEFSandboxOptions, │
  │ HEFCommand, HEFEngineInfo       │
  └─────────────────────────────────┘
                  │
  ┌── Engines/IFlightEngine ────────┐
  │ Interface + registry            │
  │ REQUIRED + OPTIONAL methods     │
  └─────────────────────────────────┘
                  │
  ┌── Engines/FBW/ ─────────────────┐
  │ FBWHeliConfig (params+getters)  │ → registers with HeliConfig
  │ FBWEngine (facade)              │ → registers with IFlightEngine
  │ FBWOrientation, FBWFilters,     │ → uses Core/Models, Core/Toolkit
  │ FBWForceComputer, FBWYawCtrl,   │ → uses ctx closures (no adapter calls)
  │ FBWInputProcessor, FBWFlightMdl │
  └─────────────────────────────────┘
                  │
  ┌── HeliSimService (dispatcher) ──┐
  │ Resolves engine, delegates 1:1  │
  │ Guards optional methods         │
  └─────────────────────────────────┘
                  │
  ┌── Debug/ ───────────────────────┐
  │ HeliDebug                       │
  │ HeliDebugCommands               │ (auto-discovers from engine)
  └─────────────────────────────────┘
                  │
          ┌───────┴───────┐
          │   HeliMove    │  (orchestrator, builds HEFCtx/HEFCorrectionCtx)
          │ HeliAuxiliary │  (gameplay side-effects)
          └───────────────┘
```

No mutual runtime dependencies. Each layer only calls downward.
Engines are self-contained: register at load time, discovered via IFlightEngine.
Data flows through typed contracts: HEFCtx down, HEFUpdateResult up.

### Architectural Patterns

**Strategy Pattern** — `IFlightEngine` defines the contract (required + optional methods).
Engines register at load time. `HeliSimService` dispatches to the active engine. `HeliMove`
never sees the engine directly. Adding a new engine: create `Engines/NewEngine/`, implement
interface, register, set `HEF.FlightEngine` sandbox string to engine name.

**Typed Data Contracts** — Framework↔engine communication uses typed classes with EmmyLua
annotations. `HEFContext.build()` constructs `HEFCtx` (framework→engine). Engines return
`HEFUpdateResult`/`HEFGroundResult` (engine→framework). All types have `@class` annotations
for IDE autocompletion, `CTX_FIELDS` for runtime validation, and `new()` constructors. `HEF`
prefix distinguishes from PZ types.

**Two-Phase Frame** — Phase 1 (OnTickEvenPaused, before physics): `HEFCorrectionCtx.build()`
→ engine `applyCorrectionForces(cctx)` (optional, 0-frame delay horizontal corrections).
Phase 2 (OnTick, after physics): `HEFContext.build()` → engine `update(ctx)` (main flight).
Each phase has its own typed context with fresh reads.

**Framework-Owned Reads** — Velocity, position, mass, physics timing, keyboard, and wall
blocking are read once per frame by the framework (`HEFContext.build`), passed to engines via
ctx. Engines never call `HeliVelocityAdapter`, `HeliTerrainUtil`, or `vehicle:getMass()` in
per-frame methods. Prevents stale-read corruption from multiple adapter calls.

**Centralized Config** — `HeliConfig.lua` is the single source of truth for sandbox params and
named constants. Engine-specific runtime tunables owned by the engine via `getTunables()/setTunable()`.
Each engine owns its own sandbox namespace (e.g., `SandboxVars.FBW.*`).

**Dispatcher Pattern** — HeliSimService resolves engine once, delegates all calls 1:1. Guards
optional methods with existence checks. Full EmmyLua annotations for IDE autocompletion.

**Module Lifecycle** — `HeliMove.onFlightInit(fn)` / `onFlightCleanup(fn)` for self-registration.
Flight state init runs before ctx build to ensure clean adapter state on first frame.

**Explicit Flight State** — `_flightState` is one of: `inactive`, `warmup`, `airborne`.
Ground and engine-off paths set `_flightState = inactive` (triggers re-init next frame).

### Change Scenario Matrix

| Change scenario | Files affected | Folder |
|----------------|---------------|--------|
| New ctx field for engines | HEFContext.lua (annotation + CTX_FIELDS + builder) | HEF/ |
| New correction-phase field | HEFCorrectionCtx.lua (same pattern) | HEF/ |
| New result field | HEFUpdateResult.lua or HEFGroundResult.lua | HEF/ |
| New sandbox parameter | HeliConfig + sandbox-options.txt | Util/ |
| Add new flight engine | New Engines/X/ folder, implement IFlightEngine, set sandbox string | Engines/ |
| FBW PD controller tuning | FBWEngine tunables (auto-discovered by /hef) | Engines/FBW/ |
| FBW rotation feel / key mappings | FBWInputProcessor only | Engines/FBW/ |
| FBW quaternion math | FBWOrientation (uses Models/Quaternion) | Engines/FBW/ |
| FBW add filter (turbulence) | FBWFilters + FBWEngine (add stage) | Engines/FBW/ |
| FBW swap sim model | SimModel2D instance in FBWEngine | Engines/FBW/ + Core/Toolkit/ |
| Shared quaternion math | Quaternion.lua or RotationMatrix.lua | Models/ |
| PZ exposes velocity setter | HeliVelocityAdapter + HEFContext builder | Adapters/ + HEF/ |
| Force application changes | HeliForceAdapter only | Adapters/ |
| Wall collision detection | HeliTerrainUtil (+ HEFContext builder reads it) | Util/ |
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
