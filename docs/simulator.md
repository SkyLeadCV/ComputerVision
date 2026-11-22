# Simulator protocol v1 (Python ↔ Unity simulator)

The brain (Python, on the laptop) flies the simulated drone in Unity (GPU PC) exactly as it flies the real Tello:
with **Tello SDK text commands**. Only the `DroneInterface` implementation changes between the two, never the brain.

This is a different channel from the map protocol (`docs/protocol.md`): the map only draws, while the simulator
is the drone.

## 1. `DroneInterface`

The brain only uses these four methods. `TelloDrone` implements them with `djitellopy` and `UnityDrone` with this protocol.

| Method | Returns | Notes |
|---|---|---|
| `send(command: str) -> str` | the reply: `"ok"`, `"error <reason>"` | One SDK command (section 4). Blocks until the move has finished. |
| `take_photo() -> np.ndarray` | BGR image, 960×720, **undistorted** | What `extract_frames.py` produces, so the vision pipeline is the same for both drones. |
| `get_state() -> dict` | `yaw_deg`, `pitch_deg`, `height_cm`, `battery_pct` | Signs and units as in `docs/protocol.md`. |
| `close() -> None` | — | Lands if still flying and closes the connection. |

- **Position is not part of the state.** The brain dead-reckons x and y from the commands it sends, as it will with the real drone, so the simulator never tells it where the drone really is.
- `TelloDrone` undistorts each frame with `calibration/camera_calibration.npz`. The simulator renders with a distortion-free pinhole camera, so its photos are already undistorted.

## 2. Transport

- **TCP over Tailscale.** Unity listens and the laptop connects, with the same rules as the map: Unity binds only to the PC's Tailscale address (or to `127.0.0.1` behind an SSH tunnel), never to `0.0.0.0`.
- **Port:** `5006`. The map uses `5005`, so both can run in the same Unity project at the same time.
- **Request/reply:** the brain sends one command and waits for its reply before sending the next, as the Tello SDK requires.
- **Framing:** every command and every text reply is one ASCII line terminated by `\n`. The only binary data is the photo (section 5).
- **Timeout:** the brain gives up after **20 s** without a reply, a wide margin over the longest move (`forward 500`).
- **A new connection is a new flight:** the drone is reset to the take-off pad, landed, with full battery.

## 3. The simulated drone

- **Camera:** the calibrated Tello intrinsics (fx = 931.5, fy = 924.5, cx = 441.9, cy = 344.2 px) at 960×720, without distortion. The horizontal field of view is therefore 54.5° and the vertical one +20.4° / −22.1°, as `docs/protocol.md` states.
- **Take-off height:** 80 cm, inside the 76–83 cm the real drone holds.
- **Moves are exact.** Real drift is a few centimetres per move, below the 10 cm voxels, so the simulator does not add noise in v1.
- **Collisions:** if a move would hit a wall, furniture or a person, the drone stops at the contact point and replies `error collision`. The real drone would crash there, so this is how the simulator exposes a planning bug.

## 4. Commands

Standard Tello SDK commands. Distances are in cm and angles in degrees, with the same limits as the real drone.

| Command | Reply | Notes |
|---|---|---|
| `command` | `ok` | Enters SDK mode. Kept so the brain's start-up sequence is identical for both drones. |
| `streamon` | `ok` | Same reason. |
| `takeoff` | `ok` | Rises to the take-off height. |
| `land` | `ok` | |
| `forward x`, `back x` | `ok` | x from 20 to 500. |
| `left x`, `right x` | `ok` | x from 20 to 500. |
| `up x`, `down x` | `ok` | x from 20 to 500. |
| `cw x`, `ccw x` | `ok` | x from 1 to 360. |
| `photo` | `photo <n>` + n bytes | **Simulator only** (section 5). |
| `state?` | state line | **Simulator only** (section 6). |

Errors:

| Reply | When |
|---|---|
| `error unknown command` | A command not in the table above. |
| `error out of range` | A value outside its limits. |
| `error not flying` | A move before `takeoff` or after `land`. |
| `error collision` | The move hit something (section 3). |

## 5. `photo`

The reply is the text line `photo <n>`, where `<n>` is the size in bytes, followed by exactly `<n>` bytes of a **JPEG** (quality 90) of the current camera view.

- The size comes first because JPEG bytes can contain `\n`, so the reader cannot look for the end of the line.
- JPEG keeps each photo at about 60 kB (measured on the calibration images), against about 2 MB for a raw 960×720 frame over Tailscale. The real drone's frames are also compressed, since they arrive as H.264.

## 6. `state?`

The reply is one line in the format of the Tello's own state packet, restricted to the fields the brain uses:

```
pitch:-3;roll:0;yaw:90;tof:82;bat:100;
```

| Field | Unit | Meaning |
|---|---|---|
| `pitch` | ° | Nose up is positive, as `pitch_deg` in `docs/protocol.md`. |
| `roll` | ° | Not used by the brain yet. Kept so the format stays the Tello's. |
| `yaw` | ° | From −180 to 180, 0 at take-off, **clockwise positive** (as `cw`). |
| `tof` | cm | Distance from the drone to the floor. |
| `bat` | % | Battery. |

The real drone sends the same packet every 100 ms over UDP and `djitellopy` parses it, so `TelloDrone` and `UnityDrone` build `get_state()` from the same fields.
The signs of the real drone's `pitch` and `yaw` still have to be checked in the first real flight. If they differ, `TelloDrone` converts them and this document does not change.

## 7. Example session

```
→ command                 ← ok
→ streamon                ← ok
→ takeoff                 ← ok
→ state?                  ← pitch:0;roll:0;yaw:0;tof:80;bat:100;
→ photo                   ← photo 61204
                          ← <61204 bytes of JPEG>
→ cw 15                   ← ok
→ forward 200             ← ok
→ forward 600             ← error out of range
→ forward 300             ← error collision
→ land                    ← ok
```

## 8. Versioning

The same rule as the map protocol: new commands or new state fields keep v1, and changing or removing one raises the version.
Neither side should depend on an unlisted field, so a new field never breaks the other side.
