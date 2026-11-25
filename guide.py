"""Guide the rescuer to the victim: the phase after the exploration has flown back to take-off.

The drone climbs to face height, turns to look back along the route and waits for the rescuer's face
(Viola-Jones, detect_rescuer.py). It then flies the A* route over floor it has already seen backwards, so the
camera keeps the rescuer in sight, and measures the rescuer's distance before every step so that it never grows
past MAX_RESCUER_CM. It lands STOP_SHORT_CM before the victim.
"""

import math
from typing import Callable

import numpy as np

from detect_rescuer import Rescuer
from find_free_space import MIN_FORWARD_CM

# From the 80 cm of the exploration to about 180 cm. The camera looks 11° down and sees up to 9.4° above the
# horizontal, so from there a whole face 1.55-1.75 m high shows from about 50 cm to 4 m; at 160 cm (up 80) the
# face of a 1.80 m person only shows beyond 140 cm.
CLIMB_CM = 100
# Decided in class: the rescuer is never farther than this from the drone.
MAX_RESCUER_CM = 150.0
# Clear of the body, with the rescuer, who follows behind, 1-2 m from the victim.
STOP_SHORT_CM = 100.0
# Photos without the rescuer within reach before the drone gives up and lands: about a minute on the real drone,
# which takes a photo a second after its last command (STREAM_SETTLE_S in drone_interface.py).
MAX_WAIT_PHOTOS = 60


class Guide:
    def __init__(self, find_rescuer: Callable[[], Rescuer | None], camera_matrix: np.ndarray) -> None:
        self.find_rescuer = find_rescuer
        self.camera_matrix = camera_matrix
        self.guiding = False

    def run(self, explorer) -> tuple[str, str]:
        """Lead the rescuer to the first victim found and land next to it; returns the session_end reason and
        what happened. explorer is the Explorer back at take-off, whose grid, pose and commands the guide uses."""
        victim_id, points = explorer.victims[0]
        victim = np.mean(points, axis=0)
        path = explorer.path_to(victim)
        if path is None:
            explorer.log.note("guide", f"no path to victim {victim_id} at ({victim[0]:.0f}, {victim[1]:.0f}): no guiding",
                              victim=victim)
            return "back_at_start", f"no path to victim {victim_id}"
        near = [index for index, cell in enumerate(path)
                if np.linalg.norm(explorer.grid.centre(*cell) - victim) <= STOP_SHORT_CM]
        full = len(path)
        path = path[:near[0] + 1] if near else path
        explorer.log.note("guide", f"victim {victim_id} at ({victim[0]:.0f}, {victim[1]:.0f}) from {len(points)} sightings: "
                          f"route {full} cells, cut to {len(path)} to stop {STOP_SHORT_CM:.0f} cm short", victim=victim,
                          sightings=points, path=path, full_path_cells=full)

        explorer.command(f"up {CLIMB_CM}")
        for target in explorer.legs(path):
            offset = target - explorer.position
            remaining = float(np.linalg.norm(offset))
            explorer.log.note("guide_leg", f"leg to ({target[0]:.0f}, {target[1]:.0f}), {remaining:.0f} cm backwards"
                              + (f": skipped, under {MIN_FORWARD_CM} cm" if remaining < MIN_FORWARD_CM else ""),
                              start=explorer.position, target=target, distance_cm=remaining)
            if remaining < MIN_FORWARD_CM:
                continue
            # Backwards along the leg, so the camera faces the rescuer following behind.
            explorer.turn(math.degrees(math.atan2(offset[0], offset[1])) + 180.0 - explorer.yaw_deg)
            while remaining >= MIN_FORWARD_CM:
                allowed = self.wait_for_rescuer(explorer)
                if allowed is None:
                    explorer.log.note("guide", f"rescuer not within reach in {MAX_WAIT_PHOTOS} photos: landing here",
                                      position=explorer.position)
                    land(explorer)
                    return "rescuer_lost", f"no rescuer within {MAX_RESCUER_CM:.0f} cm, landed on the way"
                step = int(min(remaining, allowed))
                explorer.log.note("guide_step", f"back {step}: {remaining:.0f} cm left on the leg, rescuer allows "
                                  f"{allowed:.0f}", step_cm=step, remaining_cm=remaining, allowed_cm=allowed)
                explorer.command(f"back {step}")
                remaining -= step

        distance = float(np.linalg.norm(explorer.position - victim))
        explorer.log.note("guide", f"arrived {distance:.0f} cm from victim {victim_id}: landing", distance_cm=distance)
        explorer.map.send("state", explorer.clock(), mode="arrived", person_distance_cm=-1.0)
        land(explorer)
        return "at_victim", f"landed {distance:.0f} cm from victim {victim_id}"

    def wait_for_rescuer(self, explorer) -> float | None:
        """Take photos until the rescuer is close enough for a step back of at least MIN_FORWARD_CM and return how
        far the drone may go back, or give up and return None. The drone is waiting until the rescuer first comes
        within reach and guiding from then on, also while the rescuer catches up after each step."""
        for attempt in range(1, MAX_WAIT_PHOTOS + 1):
            rescuer = self.find_rescuer()
            if rescuer is None:
                distance, allowed = -1.0, 0.0
            else:
                distance = math.hypot(*rescuer_offset_cm(rescuer, self.camera_matrix))
                # Going back d leaves the rescuer at most distance + d away, wherever it steps meanwhile.
                allowed = MAX_RESCUER_CM - distance
            close = allowed >= MIN_FORWARD_CM
            photo = getattr(explorer.drone, "photos", None)
            seen = "no face" if rescuer is None else f"face {rescuer.box} at {distance:.0f} cm, allows {allowed:.0f}"
            explorer.log.note("rescuer", f"photo {photo} (try {attempt}): {seen}" + ("" if close else ": waiting"),
                              photo=photo, attempt=attempt, box=None if rescuer is None else rescuer.box,
                              axis_cm=None if rescuer is None else rescuer.distance_cm, distance_cm=distance,
                              allowed_cm=allowed)
            self.guiding |= close
            explorer.map.send("state", explorer.clock(), mode="guiding" if self.guiding else "waiting",
                              person_distance_cm=round(distance, 1))
            if close:
                return allowed
            # The Tello lands by itself after 15 s without a command; this one only answers ok.
            explorer.drone.send("command")
        return None


def rescuer_offset_cm(rescuer: Rescuer, camera_matrix: np.ndarray) -> tuple[float, float]:
    """Where the rescuer's face is: ahead along the camera's axis, as detect_rescuer measures it, and to the side,
    from where the box's centre falls in the photo (pinhole model)."""
    x, _, width, _ = rescuer.box
    return rescuer.distance_cm, rescuer.distance_cm * (x + width / 2 - camera_matrix[0, 2]) / camera_matrix[0, 0]


def land(explorer) -> None:
    reply = explorer.drone.send("land")
    if reply != "ok":
        raise RuntimeError(f"land: {reply}")
