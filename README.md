# Helicopter Flight Engine Framework (HEF)

A pluggable flight engine framework for Project Zomboid helicopter mods (Build 42). Select the active flight physics model via sandbox settings. Ships with the **FBW (Fly-By-Wire)** engine.

## Installation

Subscribe on Steam Workshop, or copy the mod folder to your PZ mods directory. Load order:

1. WarThunder Vehicle Library
2. UH-1B Helicopter
3. **Helicopter Flight Engine Framework** (this mod)

### Requirements

- [UH-1B Helicopter](https://steamcommunity.com/sharedfiles/filedetails/?id=3409723807)
- WarThunder Vehicle Library (required by UH-1B)

## What It Does

The UH-1B Helicopter mod's flight system requires the `-debug` flag because it uses Java reflection APIs gated behind `Core.debug` mode. HEF replaces the debug-dependent movement system with Bullet physics forces, enabling helicopter flight on dedicated servers without debug mode.

The framework uses a **strategy pattern** — the sandbox setting **HEF > Flight Engine** selects which engine handles flight. Each engine is a self-contained implementation behind a common interface.

## FBW Engine

The included Fly-By-Wire engine replaces teleport-based movement with force-based physics:

- Tilt the helicopter to move — **tilt angle controls speed**, auto-leveling decelerates naturally
- Hover is stable (zero drift for 20+ seconds)
- Full 360° yaw rotation without heading drift
- Works on dedicated servers without `-debug` flag
- All flight parameters tunable via sandbox options and `/hef` chat commands

## Sandbox Options

### Flight Engine Framework (HEF page)
| Setting | Description |
|---------|-------------|
| Flight Engine | Active engine (default: FBW) |

### FBW Engine (FBW page)
| Setting | Default | Description |
|---------|---------|-------------|
| Gravity Estimate | 9.8 | Gravity compensation strength |
| Ascend Speed | 8.0 | W key ascent speed |
| Descend Speed | 14.0 | S key descent speed |
| Gravity Fall Speed | 24.5 | Fall speed when engine off |
| Engine Dead Fall Speed | 35.0 | Fall speed when engine destroyed |
| Responsiveness Gain | 8.0 | Vertical responsiveness (also `/hef kp`) |
| Max Horizontal Speed | 450.0 | Speed at maximum tilt |
| Braking Multiplier | 0.05 | How quickly the helicopter stops |

## Chat Commands

Type `/hef` in chat. Commands auto-discover from the active engine.

```
/hef help     — list all commands and tunables
/hef show     — display all current values
/hef reset    — reset to defaults
/hef vel      — show current velocity
/hef log      — toggle console logging
/hef snap     — detailed state dump to chat
/hef record   — toggle CSV flight data recording
/hef <name> <value> — set any tunable (e.g. /hef pgain 5)
```

### FBW Runtime Tunables (session-only, not saved)
| Tunable | Default | Description |
|---------|---------|-------------|
| pgain | 7.0 | Horizontal position correction strength |
| dgain | 0.3 | Horizontal correction damping |
| maxerr | 10.0 | Maximum correction distance (meters) |
| fstopgain | 0.3 | Braking sharpness (0.3=smooth, 0.8=snappy) |
| yawgain | 0.9 | Heading hold strength |
| autolevel | 1.0 | How fast tilt returns to level |

## For Engine Authors

Want to create a custom flight engine? See `KNOWLEDGE.md` for the full technical reference.

### Quick Start

1. Create `shared/HEF/Engines/YourEngine/YourEngineHeliConfig.lua` — register params + define getters
2. Create `shared/HEF/Engines/YourEngine/YourEngine.lua` — implement the flight engine
3. Implement all methods listed in `IFlightEngine.REQUIRED_METHODS`:

| Category | Methods |
|----------|---------|
| Frame update | `update(ctx)` → HEFUpdateResult, `updateGround(ctx)` → HEFGroundResult |
| Lifecycle | `resetFlightState()`, `initFlight(vehicle)`, `tickWarmup()`, `isWarmedUp()` |
| Tunables | `getTunables()`, `getTunable(name)`, `setTunable(name, value)` |
| Sandbox | `getSandboxOptions()` |
| Debug | `getDebugState()`, `getDebugColumns()`, `getIntendedYaw()` |
| Commands | `getCommands()`, `executeCommand(name, args)` |
| Metadata | `getInfo()` |

4. Optionally implement `applyCorrectionForces(cctx)` (see `IFlightEngine.OPTIONAL_METHODS`)
5. Register: `IFlightEngine.register("YourEngine", YourEngineTable)`
6. Add your sandbox options to `sandbox-options.txt` under your own namespace:
```
option YourEngine.SomeSetting
{
    type = double, min = 0, max = 100, default = 50,
    page = YourEngine,
    translation = YourEngine_SomeSetting,
}
```
7. Add translation strings to `Translate/EN/Sandbox_EN.txt`
8. Set `HEF.FlightEngine` sandbox string to your engine name (default is `"FBW"`). For a standalone third-party mod, users set this in sandbox settings or `SandboxVars.lua`

### What the Framework Provides

Every frame, your engine receives a typed `HEFCtx` table (see `HEFContext.lua`):

| Field | Type | Description |
|-------|------|-------------|
| vehicle | BaseVehicle | The helicopter (escape hatch for engine-specific state) |
| playerObj | IsoPlayer | The pilot |
| keys | HEFKeys | {up,down,left,right,w,s,a,d} booleans |
| fpsMultiplier | number | Frame time scaling |
| fps | number | Current FPS (clamped) |
| heliType | string | Helicopter type name |
| curr_z | number | Altitude (z-levels) |
| nowMaxZ | number | Ground height |
| posX, posZ | number | Vehicle position |
| velX, velY, velZ | number | Velocity (smoothed horizontal, raw vertical) |
| mass | number | Vehicle mass |
| subSteps, physicsDelta | number | Physics timing |
| blocked | HEFBlocked | {up,down,left,right} wall collision |
| fuelPercent | number | Remaining fuel (0..100, lazy-read) |
| engineCondition | number | Engine part condition (0..100, -1 if absent, lazy-read) |
| angleX, angleY, angleZ | number | Vehicle Euler angles (degrees) |
| positionDeltaSpeed | number | Ground speed from position delta (m/s) |

### Output Closures (ctx)

Instead of calling adapters or game APIs directly, use ctx closures:

| Closure | Description |
|---------|-------------|
| `ctx.applyForce(fx, fy, fz)` | Apply physics force (Bullet space, adapter-wrapped) |
| `ctx.setAngles(x, y, z)` | Set vehicle Euler angles (degrees) |
| `ctx.setPhysicsActive(active)` | Wake/sleep Bullet physics body |

The correction context (`cctx`) also provides `cctx.applyForce(fx, fy, fz)`.

### Registering Engine Parameters

Register your engine's tunable parameters via `HeliConfig.registerParams()`:

```lua
-- In your engine's config file (e.g., YourEngineHeliConfig.lua):
HeliConfig.registerParams({
    myParam = { ns = "YourEngine", field = "MyParam", default = 1.0, min = 0, max = 10, desc = "..." },
}, { "myParam" })  -- display order

-- Define typed getters on HeliConfig (Lua extension method pattern):
function HeliConfig.GetMyParam() return HeliConfig.get("myParam") end
```

The string key lives only in the getter definition. All engine code uses `HeliConfig.GetMyParam()`.
See `FBWHeliConfig.lua` for a complete example.

### What Your Engine Must Return

**From `update(ctx)`** — return a table with at minimum:

| Field | Type | Description |
|-------|------|-------------|
| engineDead | boolean | Engine below death threshold (framework shuts off engine) |
| dualPathActive | boolean | True = framework calls `applyCorrectionForces` this frame |
| displaySpeed | number | km/h for speedometer |

**From `updateGround(ctx)`** — return a table with at minimum:

| Field | Type | Description |
|-------|------|-------------|
| liftoff | boolean | True = framework transitions to warmup/airborne |
| displaySpeed | number | km/h for speedometer |

All types have EmmyLua `@class` annotations — type `ctx.` in IntelliJ with EmmyLua and get full autocompletion.

## Project Structure

```
shared/HEF/
  HEFContext.lua              — per-frame context (HEFCtx) builder + contract
  HEFCorrectionCtx.lua        — correction-phase context builder + contract
  HEF*Result.lua, HEF*.lua    — typed return contracts (8 type files)
  Core/                        — loads before Engines/ (C < E alphabetically)
    Adapters/                  — HeliForceAdapter, HeliVelocityAdapter
    Models/                    — Quaternion, RotationMatrix (shared OOP)
    Toolkit/                   — optional building blocks for engine authors
      CoordUtil.lua            — PZ Y/Z swap, unit conversions
      VelocityUtil.lua         — speed decomposition helpers
      PDController.lua         — stateless PD with tanh saturation
      SimModel2D.lua           — 2D position/velocity sim (instance-based)
      ErrorTracker2D.lua       — ring buffer error tracking (instance-based)
      VerticalModel.lua        — default W/S/hover/fall vertical model
      GroundModel.lua          — default ground hold + liftoff
    Util/                      — HeliConfig, HeliUtil, HeliCompat, HeliTerrainUtil
  Engines/
    IFlightEngine.lua          — interface + registry
    FBW/                       — fly-by-wire engine (8 files, uses Core/Toolkit)
      FBWHeliConfig.lua        — FBW params + typed getters (registers with HeliConfig)

client/HeliAbility/
  HeliSimService.lua           — thin dispatcher (delegates to active engine)
  HeliMove.lua                 — orchestrator (builds ctx, reads results, events)
  Debug/                       — HeliDebug, HeliDebugCommands
  HeliAuxiliary.lua            — ghost mode, lights, gas, damage
```

## License

This mod is provided as-is for use with Project Zomboid. See workshop page for terms.
