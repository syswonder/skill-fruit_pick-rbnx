# SPDX-License-Identifier: MulanPSL-2.0
"""Run bookkeeping for pick_fruit and the background capture.

States follow the Executor's long-task convention: PENDING, RUNNING,
SUCCEEDED, FAILED, CANCELED, TIMEOUT. One run may be active per skill
instance. A failed or canceled run stops issuing commands; it never moves
the arm on its own afterwards. The one exception is an empty grip: nothing
is between the jaws, so they open again before the run fails.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import logging
import math
from pathlib import Path
import threading
import time
import uuid

import cv2

from .clients import HOLDING, Canceled, PickError
from .config import Pose, quaternion
from .plan import plan_steps
from .vision import find_fruits, matrix

log = logging.getLogger("fruit_pick")
TERMINAL = frozenset({"SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"})
HISTORY = 20


@dataclass
class Run:
    """One pick_fruit call and its progress."""

    run_id: str
    destination: object
    state: str = "PENDING"
    detail: str = "queued"
    step: str = ""
    cancel: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None


class Runner:
    """Start, observe and cancel runs; capture the empty-table background."""

    def __init__(self, config, arm, gripper, camera, background_path, clock=time.time):
        self.config, self.arm, self.gripper, self.camera = config, arm, gripper, camera
        self.background_path = Path(background_path)
        self._clock = clock
        self._lock = threading.Lock()
        self._runs = {}
        self._capture = None   # cancel event of the capture in progress
        c = config.camera_in_tcp
        self._camera_in_tcp = matrix(c[:3], quaternion(c[3:]))
        self._background = self._load_background()

    def _interrupt(self):
        for client in (self.arm, self.gripper, self.camera):
            client.interrupt()

    # -- background ----------------------------------------------------------

    def _load_background(self):
        sidecar = self.background_path.with_suffix(".json")
        if not sidecar.exists():
            return None
        gray = cv2.imread(str(self.background_path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            log.warning("cannot read %s; capture the background again", self.background_path)
            return None
        meta = json.loads(sidecar.read_text())
        return gray, Pose(tuple(meta["position"]), tuple(meta["orientation"]))

    def capture_background(self):
        """Photograph the empty table from home, jaws open, and keep it; return a summary."""
        cancel = threading.Event()
        with self._lock:
            if self._capture is not None or self._active():
                raise RuntimeError("a pick_fruit run or capture is in progress")
            self._capture = cancel
        try:
            self._go_home(cancel)
            self.gripper.open(cancel)
            pose = self.arm.current_pose()
            gray, _, _ = self.camera.frame_after(self._clock(), cancel)
            self.background_path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self.background_path), gray):
                raise PickError(f"cannot write {self.background_path}")
            self.background_path.with_suffix(".json").write_text(json.dumps(
                {"position": pose.position, "orientation": pose.orientation}))
            self._background = gray, pose
        finally:
            with self._lock:
                self._capture = None
        return f"captured a {gray.shape[1]}x{gray.shape[0]} background of the empty table"

    # -- runs ----------------------------------------------------------------

    def _active(self):
        return [r for r in self._runs.values() if r.state not in TERMINAL]

    def start(self, destination):
        """Start a run; return its run_id.

        ``destination`` is a configured location name, or "" to put the fruit
        back where it was. Raises ValueError for an unknown location and
        RuntimeError while another run or a capture is in progress.
        """
        dest = self.config.location(destination) if destination else None
        run = Run(f"pick-fruit-{uuid.uuid4().hex[:12]}", dest)
        run.thread = threading.Thread(target=self._execute, args=(run,), name=run.run_id, daemon=True)
        with self._lock:
            active = self._active()
            if active:
                raise RuntimeError(f"run {active[0].run_id} is still {active[0].state}")
            if self._capture is not None:
                raise RuntimeError("the background is being captured")
            self._runs[run.run_id] = run
            for old in list(self._runs)[:-HISTORY]:
                del self._runs[old]
        run.thread.start()
        return run.run_id

    def status(self, run_id):
        """Return ``(state, detail)``; raise KeyError for an unknown run_id."""
        with self._lock:
            run = self._runs[run_id]
            return run.state, run.detail

    def cancel(self, run_id):
        """Request cancellation (idempotent); return False if already finished."""
        with self._lock:
            run = self._runs[run_id]
            finished = run.state in TERMINAL
            run.cancel.set()
        self._interrupt()
        return not finished

    def cancel_all(self, timeout_s=2.0):
        """Cancel every active run and capture and wait; return True when all have ended."""
        with self._lock:
            active = self._active()
            for run in active:
                run.cancel.set()
            if self._capture is not None:
                self._capture.set()
        self._interrupt()
        deadline = time.monotonic() + timeout_s
        for run in active:
            run.thread.join(timeout=max(0.0, deadline - time.monotonic()))
        while self._capture is not None and time.monotonic() < deadline:
            time.sleep(0.05)
        return self._capture is None and not any(run.thread.is_alive() for run in active)

    def _set(self, run, state, detail):
        with self._lock:
            run.state, run.detail = state, detail
        log.info("%s %s: %s", run.run_id, state, detail)

    def _go_home(self, cancel):
        """Require the arm near home, then settle exactly on it."""
        c = self.config
        self.arm.wait_commands_ready(cancel)
        translation, rotation = self.arm.current_pose().distance(c.home)
        if translation > c.start_tolerance_m or rotation > c.start_tolerance_rad:
            raise PickError(f"the arm is {translation * 1000:.0f} mm and "
                            f"{math.degrees(rotation):.1f} deg away from home; move it there first")
        self.arm.move(c.home, cancel)

    def _find_fruit(self, run):
        """Photograph the table and return (fruit, number of other fruit in view)."""
        c = self.config
        if self._background is None:
            raise PickError("no background of the empty table yet; call capture_background first")
        background, taken_at = self._background
        pose = self.arm.current_pose()
        translation, rotation = pose.distance(taken_at)
        # Both photos are taken after settling on home, each within the arrival tolerance.
        if translation > 2 * c.position_tolerance_m or rotation > 2 * c.orientation_tolerance_rad:
            raise PickError(f"home is {translation * 1000:.1f} mm from where the background "
                            f"was taken; capture it again")
        gray, K, d = self.camera.frame_after(self._clock(), run.cancel)
        camera = matrix(pose.position, pose.orientation) @ self._camera_in_tcp
        fruits = find_fruits(gray, background, K, d, camera, c)
        if not fruits:
            raise PickError("no fruit in view")
        fruit = fruits[0]
        if fruit.diameter_m + 2 * c.clearance_m > self.gripper.max_opening_width_m:
            raise PickError(f"the fruit looks {fruit.diameter_m * 1000:.0f} mm across; the open "
                            f"jaws are {self.gripper.max_opening_width_m * 1000:.0f} mm apart")
        return fruit, len(fruits) - 1

    def _grip(self, cancel):
        if self.gripper.grip(cancel) != HOLDING:
            self.gripper.open(cancel)
            raise PickError("the jaws closed on nothing; opened them again and left the arm "
                            "at the fruit (the fruit may have moved, or the location is off)")

    def _execute(self, run):
        """Thread body: settle at home, find a fruit, then run every step in order."""
        run.step = "moving to the photo pose"
        self._set(run, "RUNNING", run.step)
        try:
            self._go_home(run.cancel)
            run.step = "opening the gripper"
            self.gripper.open(run.cancel)
            run.step = "looking for fruit"
            self._set(run, "RUNNING", run.step)
            fruit, others = self._find_fruit(run)
            x, y, _ = fruit.centre
            log.info("%s fruit %.0f mm across at (%.1f, %.1f) mm", run.run_id,
                     fruit.diameter_m * 1000, x * 1000, y * 1000)
            steps = plan_steps(self.config, fruit, run.destination)
            for index, step in enumerate(steps, 1):
                if run.cancel.is_set():
                    raise Canceled("canceled")
                run.step = f"step {index}/{len(steps)}: {step.label}"
                self._set(run, "RUNNING", run.step)
                if step.kind == "move":
                    self.arm.move(step.pose, run.cancel)
                elif step.kind == "grip":
                    self._grip(run.cancel)
                else:
                    self.gripper.open(run.cancel)
            where = run.destination.name if run.destination else "back where it was"
            self._set(run, "SUCCEEDED",
                      f"picked a fruit about {fruit.diameter_m * 1000:.0f} mm across, "
                      f"put it {where} and returned home; {others} more fruit in view")
        except Canceled:
            self._set(run, "CANCELED", f"canceled at {run.step}; the arm finishes its current "
                                       f"motion and the gripper is left as it is")
        except TimeoutError as exc:
            self._set(run, "TIMEOUT", f"{run.step}: {exc}")
        except (PickError, ValueError) as exc:
            self._set(run, "FAILED", f"{run.step}: {exc}")
        except Exception as exc:  # keep the run observable whatever happened
            log.exception("run %s crashed", run.run_id)
            self._set(run, "FAILED", f"{run.step}: unexpected error: {exc}")
