# Helicopter Flight Engine Framework (HEF)

A pluggable flight engine framework for Project Zomboid helicopter mods (Build 42). Select the active flight physics model via sandbox settings. Ships with the **FBW (Fly-By-Wire)** engine.

## Requirements

- [UH-1B Helicopter](https://steamcommunity.com/sharedfiles/filedetails/?id=3409723807)
- [WarThunder Vehicle Library](https://steamcommunity.com/workshop/browse/?appid=108600&searchtext=WarThunderVehicleLibrary)

## What It Does

The UH-1B Helicopter mod's flight system requires the `-debug` flag because it uses Java reflection APIs that are gated behind `Core.debug` mode. HEF replaces the debug-dependent movement system with Bullet physics forces, enabling helicopter flight on dedicated servers without debug mode.

The framework uses a **strategy pattern** so the flight physics model is swappable. The sandbox setting `HEF > Flight Engine` selects which engine handles flight. Each engine is a self-contained implementation behind a common interface.

## FBW Engine

The included **Fly-By-Wire** engine uses a tilt-as-desired MPC (Model Predictive Control) architecture:

- **Simulation model** tracks where the helicopter should be (pure math, no Bullet)
- **Tilt angle is the throttle** — tilt sets desired velocity, auto-leveling decelerates naturally
- **Position error** (sim vs actual) drives PD correction forces via `applyImpulseGeneric`
- **Separated yaw/tilt** — scalar heading + quaternion tilt, no gimbal lock, no Euler feedback
- **Dual-path forces** — horizontal corrections on OnTickEvenPaused (0-frame delay), vertical thrust on OnTick

## Sandbox Options

### Flight Engine Framework
| Setting | Description |
|---------|-------------|
| Flight Engine | Active engine (default: FBW) |

### FBW Engine
| Setting | Default | Description |
|---------|---------|-------------|
| Gravity Estimate | 9.8 | Gravity compensation (Bullet units/s^2) |
| Ascend Speed | 8.0 | W key ascent (Bullet Y/s) |
| Descend Speed | 14.0 | S key descent (Bullet Y/s) |
| Gravity Fall Speed | 24.5 | Engine-off fall (Bullet Y/s) |
| Engine Dead Fall Speed | 35.0 | Engine-destroyed fall (Bullet Y/s) |
| Responsiveness Gain | 8.0 | Vertical PD gain |
| Max Horizontal Speed | 450.0 | Speed at max tilt (m/s) |
| Braking Multiplier | 0.05 | Base inertia rate |

## Chat Commands

`/hef` auto-discovers tunables and commands from the active engine.

```
/hef help     — list all commands
/hef show     — display current values
/hef reset    — reset to defaults
/hef vel      — show Bullet velocity
/hef log      — toggle console logging
/hef snap     — one-shot state dump
/hef record   — toggle CSV flight recorder
/hef <param> <value> — set any tunable (e.g. /hef pgain 5)
```

### FBW Tunables
| Tunable | Default | Description |
|---------|---------|-------------|
| pgain | 7.0 | Position error P gain |
| dgain | 0.3 | Position error D gain |
| maxerr | 10.0 | Max error saturation (meters) |
| fstopgain | 0.3 | Velocity damping (0.3=smooth, 0.8=snappy) |
| yawgain | 0.9 | Yaw correction strength |
| autolevel | 1.0 | Auto-leveling speed multiplier |

## For Engine Authors

Want to create a custom flight engine? Implement the `IFlightEngine` interface:

1. Create a folder: `shared/HEF/Engines/YourEngine/`
2. Implement all methods in `IFlightEngine.REQUIRED_METHODS`
3. Optionally implement `OPTIONAL_METHODS` (e.g. `applyCorrectionForces` for 0-frame delay)
4. Register: `IFlightEngine.register("YourEngine", YourEngineTable)`
5. Add your sandbox options under your own namespace
6. Add the enum option to `sandbox-options.txt` and the mapping in `HeliConfig.getEngineName()`

The framework provides a typed context (`HEFCtx`) every frame with vehicle state, keyboard input, velocity, position, mass, physics timing, and wall collision data. Your engine receives this and returns typed results (`HEFUpdateResult` / `HEFGroundResult`).

All types have EmmyLua `@class` annotations — type `ctx.` in IntelliJ and get full autocompletion.

See `KNOWLEDGE.md` for the full technical reference.

## Project Structure

```
shared/HEF/
  HEFContext.lua              — per-frame context builder + contract
  HEFCorrectionCtx.lua        — correction-phase context
  HEF*Result.lua, HEF*.lua    — typed data contracts (8 files)
  Models/                      — Quaternion, RotationMatrix (shared OOP)
  Util/                        — HeliConfig, HeliUtil, HeliCompat, HeliTerrainUtil
  Engines/
    IFlightEngine.lua          — interface + registry
    FBW/                       — fly-by-wire engine (9 files)
  Adapters/                    — HeliForceAdapter, HeliVelocityAdapter

client/HeliAbility/
  HeliSimService.lua           — thin dispatcher
  HeliMove.lua                 — orchestrator
  Debug/                       — HeliDebug, HeliDebugCommands
  HeliAuxiliary.lua            — ghost mode, lights, gas, damage
```

## License

This mod is provided as-is for use with Project Zomboid. See workshop page for terms.
