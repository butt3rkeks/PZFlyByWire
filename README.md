# Helicopter Flight Engine Framework (HEF)

A pluggable flight engine framework for Project Zomboid helicopter mods (Build 42). Ships with the **FBW (Fly-By-Wire)** engine.

## What It Does

The WarThunder Vehicle Library's helicopter flight system requires the `-debug` flag because it uses Java reflection APIs gated behind debug mode. HEF replaces the debug-dependent movement system with Bullet physics forces, enabling helicopter flight on dedicated servers without debug mode.

Works with any WTVL helicopter, including the [UH-1B Helicopter](https://steamcommunity.com/sharedfiles/filedetails/?id=3409723807).

## Features

- Stable hover, smooth ascent/descent
- Tilt to fly — arrow keys for pitch/roll, A/D for yaw, W/S for altitude
- Flight Assist Off mode (toggle in vehicle menu) for manual control
- Wall collision detection prevents flying through buildings
- Engine damage and fuel consumption
- All parameters tunable via sandbox options and `/hef` chat commands

## Installation

Subscribe on Steam Workshop, or copy the mod folder to your PZ mods directory.

**Load order:**

1. WarThunder Vehicle Library
2. A WTVL helicopter mod (e.g., UH-1B Helicopter)
3. **Helicopter Flight Engine Framework** (this mod)

### Requirements

- [WarThunder Vehicle Library](https://steamcommunity.com/workshop/filedetails/?id=3399660368) (provides `GetHeliType`, `HeliList`, vehicle scripts)
- At least one WTVL helicopter mod

### Replaces

This replaces the old "UH-1B Helicopter B42 Fix" mod (3688683236). Unsubscribe from that mod when using this one.

## Controls

| Key | Action |
|-----|--------|
| Arrow Up/Down | Pitch (tilt forward/backward) |
| Arrow Left/Right | Roll (tilt left/right) |
| A / D | Yaw (rotate heading) |
| W | Ascend |
| S | Descend |
| (no input) | Hover in place |

Tilt angle controls speed — the more you tilt, the faster you go. Releasing the arrow keys auto-levels the helicopter and decelerates naturally.

## Quick Tuning

Type `/hef help` in chat to see all available commands and settings. See [ADMIN.md](ADMIN.md) for the full configuration reference.

## Documentation

| Document | Audience | Contents |
|----------|----------|----------|
| [ADMIN.md](ADMIN.md) | Server admins | Sandbox options, chat commands, runtime tuning |
| [DEVELOPER.md](DEVELOPER.md) | Engine authors | IFlightEngine API, context fields, project structure |
| [KNOWLEDGE.md](KNOWLEDGE.md) | Contributors | Deep technical internals (physics timing, rotation math, Kahlua quirks) |

## License

This mod is provided as-is for use with Project Zomboid. See workshop page for terms.
