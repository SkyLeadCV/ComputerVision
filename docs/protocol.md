# Map protocol v1 (Python → Unity)

The brain (Python, on the laptop) streams what the drone sees and does to the 3D map (Unity, on the GPU PC).
Unity only draws: it never replies and never takes decisions.

This protocol is only for the map. Between the brain and the drone there is no JSON: both the real Tello and the simulated drone receive the plain Tello SDK text commands.

## 1. Transport

- **TCP over Tailscale.** Unity listens and the laptop connects.
- Unity binds **only to the PC's Tailscale address** (`100.x.y.z`), never to `0.0.0.0`, so the port is reachable from the tailnet only and nothing is exposed to the internet or the local network.
  - Alternative without Tailscale routing: Unity binds to `127.0.0.1` and the laptop opens an SSH tunnel, `ssh -L 5005:127.0.0.1:5005 <pc>`, then connects to `127.0.0.1:5005`.
- **Port:** `5005` (any free port works; it is a setting on both sides).
- **Framing:** one JSON object per line, UTF-8, terminated by `\n`, no pretty printing.
- **One way:** Python → Unity only.
- **The map is not critical:** if Unity is unreachable or the connection drops, the brain keeps flying and retries the connection between moves.
- **Recording:** the brain also appends every message to `flights/<session>.jsonl`, whether or not Unity is connected. Unity can replay that file without Python, using `t` to keep the real timing.

## 2. Reference frame and units

- Units: **cm**, **degrees** and **seconds**. The unit is part of the field name (`_cm`, `_deg`) except for the `x`, `y`, `z` and `t` fields shared by every message.
- Origin: the take-off point, on the floor.
- Axes, fixed at take-off: **x** to the right, **y** forward, **z** up.
- `yaw_deg`: heading, 0 = the take-off heading, **positive clockwise** seen from above (same sign as the `cw` command).
- `pitch_deg`: camera pitch, **positive nose up**. The Tello's camera looks 11° down from its body (`docs/simulator.md`), so it is the drone's own pitch minus 11: about −11 in level flight.
- Ray angles: relative to the drone's heading, **positive to the right**.
- A ray at `angle_deg` from a drone at (`x`, `y`, `yaw_deg`) points along `h = yaw_deg + angle_deg`, and the floor point at distance `d` is:

  ```
  (x + d·sin h, y + d·cos h)
  ```

- In Unity (y up, metres, left-handed):

  ```csharp
  Vector3 position = new Vector3(x, z, y) / 100f;
  Quaternion rotation = Quaternion.Euler(-pitch_deg, yaw_deg, 0f);
  ```

  Unity's positive rotation about its y axis is already clockwise seen from above, so `yaw_deg` maps directly. Its positive x rotation tilts the nose down, hence `-pitch_deg`.

## 3. Common fields

Every message has:

| Field | Type | Meaning |
|---|---|---|
| `type` | string | Message type (section 4). |
| `seq` | int | One counter for all messages of the session, starting at 0 with `session_start`. A gap means lost messages. |
| `t` | float | Seconds since `session_start`. |

## 4. Messages

### `session_start`

Sent first, and again after every reconnection. A new `session` value clears the map; the same value continues it.

```json
{"type": "session_start", "seq": 0, "t": 0.0, "version": 1, "session": "20261120_180211",
 "camera": {"hfov_deg": 54.5, "vfov_up_deg": 20.4, "vfov_down_deg": 22.1},
 "drone_radius_cm": 9, "safety_margin_cm": 15}
```

- `session`: local start time as `YYYYMMDD_HHMMSS`, the same pattern as the drone videos. It is also the recording file name, so it avoids `:` (not allowed in Windows file names).
- `camera`: field of view used to open the vertical fan of each ray.

### `observation`

What one photo saw: the drone pose and the free-space rays from `find_free_space.py`.

```json
{"type": "observation", "seq": 12, "t": 31.4,
 "pose": {"x": 120.0, "y": 340.0, "z": 82.0, "yaw_deg": 90.0, "pitch_deg": -3.1},
 "rays": {"angle_deg": [-27.0, -26.5], "entry_cm": [148, 147], "stop_cm": [212, 215], "kind": [0, 0]}}
```

- `pose`: position and attitude when the photo was taken. `z` is the camera height above the floor.
- `rays`: four lists of the same length, one item per ray, sorted by `angle_deg`. Distances are horizontal, on the floor, from the point under the camera.
  - `entry_cm`: where the visible floor starts. Closer than this is the camera's blind zone.
  - `stop_cm`: where the free floor ends, cut at the trusted range.
  - `kind`: why the ray stopped.
- **Trusted range:** the brain cuts every ray at `entry_cm + 200`, the blind zone plus one 2 m step (`STEP_LIMIT_CM` in `find_free_space.py`).
  - That band is exactly the floor the drone will cross on its next step, and it is measured precisely: under 2 cm per image pixel at 3.5 m, against about 5 cm at 6 m.
  - Farther floor is the least reliable (a wall of the floor's colour can pass for floor there), and excavation cannot be undone, so it is left solid until the drone sees it from closer.
  - It is relative to each ray's `entry_cm`, not a fixed distance, because the blind zone changes with the camera's tilt: 1.1–1.9 m in the test video.
- **A ray with `stop_cm <= entry_cm` saw no floor:** the photo shows an obstacle where the floor should start, so the obstacle stands somewhere inside the blind zone, at an unknown distance.
  - Distances are measured on the floor (the homography), so with no floor in view there is nothing to measure. `find_free_space.py` reports such a ray as `stop_cm = entry_cm` with `kind` 0.
  - The ray carries no free space and no obstacle position: Unity neither empties air nor paints a surface for it.
  - It is common in narrow passages: both arms of the classroom (`docs/classroom.md`) are narrower than the blind zone.

| `kind` | Meaning | Unity |
|---|---|---|
| 0 | obstacle | Empty the air up to `stop_cm` and paint the voxels at the stop as a seen surface. Nothing at all when `stop_cm <= entry_cm`. |
| 1 | image border (FOV) | Empty the air only. |
| 2 | trusted range (`entry_cm + 200`) | Empty the air only. |
| 3 | pocket lid | Empty the air only. |

### `move`

A command sent to the drone.

```json
{"type": "move", "seq": 13, "t": 35.0, "command": "cw 15, forward 200",
 "from": {"x": 120.0, "y": 340.0, "yaw_deg": 90.0},
 "to":   {"x": 313.2, "y": 288.2, "yaw_deg": 105.0}}
```

- `command`: the Tello SDK commands as sent, comma separated.
- `from` / `to`: pose before the move and the pose it should end at. Unity draws the trail from these. Height does not change during a move, so there is no `z`.

### `victim`

A person lying on the floor, placed on the map.

```json
{"type": "victim", "seq": 20, "t": 52.7, "id": 1, "x": 340.0, "y": 120.0, "confidence": 0.87}
```

- `id`: starts at 1. Python decides whether a detection is a known victim: a detection closer than **300 cm** to a known victim is the same one and reuses its `id`.
- A repeated `id` **moves** the existing marker to the new `x`, `y`; a new `id` adds a marker.
- `confidence`: detector score, 0 to 1.

### `state`

The current phase of the mission.

```json
{"type": "state", "seq": 41, "t": 180.2, "mode": "guiding", "person_distance_cm": 180.0}
```

- `mode`: `exploring`, `waiting`, `guiding`, `arrived`.
- `person_distance_cm`: distance to the rescuer, **-1** when no rescuer is seen (`JsonUtility` cannot read `null` into a number).

### `session_end`

```json
{"type": "session_end", "seq": 77, "t": 402.9, "reason": "back_at_start"}
```

- `reason`: `back_at_start`, `low_battery`, `user_stop`.

## 5. Reading the messages in C#

`JsonUtility` does not support polymorphism. Each line is read twice:

1. Into a small class with only `type`.
2. Into the class for that type.

Messages with an unknown `type` are ignored. Unknown fields are ignored too, which `JsonUtility` already does.

## 6. Design rules

Every new message or field follows the same rules:

1. **Only what Unity needs to draw:** rays, not photos or maps.
2. **Events, not full state:** each message is something that just happened. Unity accumulates them, which is also what makes replay work.
3. **Each message stands alone** (an observation carries its own pose), so losing one breaks nothing.
4. Every message has `type`, `seq` and `t`.
5. Units in the field name.
6. Fixed fields and lists of numbers, so that `JsonUtility` can read them.
7. **Versioning:** new optional fields keep `version` 1. Changing or removing a field raises `version`.
