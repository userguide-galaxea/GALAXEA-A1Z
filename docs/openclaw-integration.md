# OpenClaw Integration

Adds interactive robot arm control via [OpenClaw](https://openclaw.ai) using a persistent server/client architecture.

## New Files

### `a1z/robots/server.py`

Unix socket server that holds a live `ArmRobot` instance and dispatches JSON commands.

- Socket path: `/tmp/a1z.sock`
- Protocol: one newline-terminated JSON request → one JSON response per connection
- Commands serialized by a lock; `ArmRobot` runs its own 250 Hz control thread internally

Supported commands:

| Command | Args | Description |
|---------|------|-------------|
| `status` | — | Joint pos (deg), vel (rad/s), torque (Nm), gripper |
| `move` | `preset` or `joints` (deg list), optional `speed` | Move to pose, blocking |
| `gripper` | `value` [0.0–1.0] | Set gripper opening |
| `dance` | optional `moves` list, `speed` | Run choreography sequence |
| `info` | — | List presets, dance moves, joint limits |
| `stop` | — | Graceful shutdown |

Preset poses (degrees): `home`, `ready`, `salute`, `wave_l`, `wave_r`, `twist_a`, `twist_b`, `reach`, `bow`

### `tools/a1zctl`

Executable Python CLI. The `serve` subcommand starts the server in-process; all other subcommands connect to the socket and return once the command completes.

```
a1zctl serve [--can can0] [--with-gripper] [--gravity-mode]
a1zctl status
a1zctl move <j1,j2,j3,j4,j5,j6>        # degrees
a1zctl move --preset <name> [--speed N]
a1zctl gripper <0.0–1.0>
a1zctl dance [--moves a,b,c] [--speed N]
a1zctl info
a1zctl stop
```

### `openclaw/skills/a1z/SKILL.md`

OpenClaw skill descriptor. After OpenClaw loads this file the AI assistant can control the arm via natural language (e.g. "move to home position", "open the gripper halfway", "run the dance sequence").

## Usage

```bash
# Terminal 1 — keep running
python3 tools/a1zctl serve --with-gripper

# Terminal 2 or OpenClaw
python3 tools/a1zctl status
python3 tools/a1zctl move --preset home
python3 tools/a1zctl gripper 0.5
python3 tools/a1zctl stop
```

## Design Notes

- **Server/client** — required because `ArmRobot` needs a persistent 250 Hz control loop; one-shot scripts would re-enable the motors on every call.
- **Position hold mode** (`zero_gravity_mode=False`) is the server default — motors resist disturbances and hold position between commands.
- **Blocking moves** — `move` and `dance` commands block until the arm arrives, so OpenClaw sees a clean response with the final position.
- **CAN interface** must be up before starting the server: `sudo ip link set can0 up type can bitrate 1000000`
