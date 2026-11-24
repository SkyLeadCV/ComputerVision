# Classroom layout v1

The real classroom the drone explores, measured on site on 2026-11-23. The virtual classroom of the simulator
(021, 023) is built from this document, and the brain (020) can use it to check its map against the real room.

## 1. Room frame

- Units: **cm**.
- Origin: the **bottom-left outer corner** of the plan below, on the floor.
- **x** to the right, **y** up the plan (forward from the take-off pad), **z** up from the floor.
- This is a fixed frame of the room. The map protocol (`docs/protocol.md`) uses a different origin, the take-off point; section 5 converts between them.

## 2. Plan

The free floor is an inverted U. Everything inside the outline is free floor to fly over; everything outside is a
wall or furniture the drone would hit.

```
 y
560 +---------------------------------------+
    |                                       |
    |              free floor               |
    |                                       |
310 |           +----------------+          |
    |           |                |          |
    |   left    |  tables block  |  right   |
    |   arm     |  (obstacle)    |  arm     |
    |    T      |                |          |
  0 +-----------+                +----------+
    0          120              370        460  x

 T: take-off pad, facing +y
```

| Part | x (cm) | y (cm) | Size (cm) |
|---|---|---|---|
| Top band | 0–460 | 310–560 | 460 × 250 |
| Left arm | 0–120 | 0–310 | 120 × 310 |
| Right arm | 370–460 | 0–310 | 90 × 310 |
| Tables block (obstacle) | 120–370 | 0–310 | 250 × 310 |

Free floor polygon, counter-clockwise:

```
(0, 0) (120, 0) (120, 310) (370, 310) (370, 0) (460, 0) (460, 560) (0, 560)
```

- Free floor area: 18.01 m² (11.50 + 3.72 + 2.79).
- Every closed part of the plan was measured as furniture or wall, so the outline above is the complete obstacle map: there are no further obstacles inside the free floor.

## 3. Heights

| Element | Height (cm) | Notes |
|---|---|---|
| Flight height | 76–83 | Measured from the reference tiles in the test video. |
| Tables block | above the flight height | Treated as a solid obstacle from the floor up, like a wall: the drone cannot fly over or under it. |
| Ceiling | 300 | High enough for the flight; also the height of the map's voxel block. |

## 4. Take-off pad

- At **(60, 60)**, heading **+y** (up the left arm).
- Confirmed on site on 2026-11-24: the centre of the left arm and one 60 cm floor tile from the bottom wall.

## 5. From the map frame to the room frame

The map protocol puts its origin at the take-off point with y along the take-off heading. With the pad above, a
point (`x`, `y`) of `docs/protocol.md` is at room (`x + 60`, `y + 60`), and `yaw_deg` is the same in both frames.

## 6. What this means for flying

- With the drone radius and the safety margin of `find_free_space.py` (9 + 15 = 24 cm on each side), the drone's
  centre has a 72 cm wide lane in the left arm and only **42 cm** in the right arm. Both fit, the right one tightly.
- The longest straight line inside the free floor is about 5.7 m (from a bottom corner of an arm to the opposite top corner), below the 6 m sweep range, so a ray always ends on a wall or furniture within range.
- The room is far smaller than the 10 × 8 m assumed so far for the voxel map and the synthetic flights: 4.6 × 5.6 m.

## 7. Not measured yet

- Height of the green-grey strip at the base of the walls, and where the coloured tape that separates it from the floor is placed.
