"""
State machine for orchestrating multi-step manipulation task.
Coordinates: Extract → Reorient → Handoff → Place → Complete.
"""

import mujoco
import numpy as np
from enum import Enum
from typing import Optional


class TaskState(Enum):
    """Enumeration of task states."""
    EXTRACT = "extract"
    REORIENT = "reorient"
    HANDOFF = "handoff"
    PLACE = "place"
    COMPLETE = "complete"


class TaskExecutor:
    """
    Orchestrates a multi-step manipulation task using a state machine.

    States:
    - EXTRACT: Spread fingers, reach down, grip object. Transitions when >2 contacts.
    - REORIENT: Finger gaiting (4-phase cycle). Auto-transitions after 5s.
    - HANDOFF: Right hand approaches, dual grip, left releases. After 6s transitions to PLACE.
    - PLACE: Lower right hand, release grip. After 5s transitions to COMPLETE.
    - COMPLETE: Task finished.

    Safety: Each state has 10s timeout; if no transition occurs, auto-advance.
    """

    FORCE_THRESHOLD = 0.05  # N
    STATE_TIMEOUT = 10.0    # s

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 controller, sensor_reader):
        """
        Initialize the TaskExecutor.

        Args:
            model: MuJoCo model.
            data: MuJoCo data (simulation state).
            controller: TeleoperationController instance.
            sensor_reader: SensorReader instance.
        """
        self.model = model
        self.data = data
        self.controller = controller
        self.sensor_reader = sensor_reader

        self.state = TaskState.EXTRACT
        self.state_timer = 0.0
        self.reorient_phase = 0  # 0-3 for finger gaiting phases

    def get_state(self) -> str:
        """Return current state name (lowercase)."""
        return self.state.value

    def is_task_complete(self) -> bool:
        """Return True if task is in COMPLETE state."""
        return self.state == TaskState.COMPLETE

    def reset(self) -> None:
        """Reset task to initial state."""
        self.state = TaskState.EXTRACT
        self.state_timer = 0.0
        self.reorient_phase = 0

    def step(self, dt: float = 0.002) -> None:
        """
        Execute one control cycle.

        Updates state machine and generates controls for current state.
        Handles state transitions based on sensor feedback or timeouts.

        Args:
            dt: Time step in seconds (default 0.002 for 500 Hz).
        """
        self.state_timer += dt

        # Execute state-specific logic
        if self.state == TaskState.EXTRACT:
            self._state_extract()
        elif self.state == TaskState.REORIENT:
            self._state_reorient()
        elif self.state == TaskState.HANDOFF:
            self._state_handoff()
        elif self.state == TaskState.PLACE:
            self._state_place()
        elif self.state == TaskState.COMPLETE:
            pass  # No controls

        # Check for timeout transition (safety fallback)
        if self.state_timer > self.STATE_TIMEOUT and self.state != TaskState.COMPLETE:
            self._transition_to_next_state()

    def _state_extract(self) -> None:
        """
        Extract phase: spread fingers, reach down, grip object.

        Transitions to REORIENT when >2 contact forces detected.
        """
        # Spread left fingers (joints 1-3 slightly open)
        self.controller.set_control("left", 1, 0.3)
        self.controller.set_control("left", 2, 0.3)
        self.controller.set_control("left", 3, 0.3)

        # Reach down with left hand (joint 0 down)
        self.controller.set_control("left", 0, -0.5)

        # Grip (close fingers after 1s)
        if self.state_timer > 1.0:
            self.controller.set_control("left", 1, -0.8)
            self.controller.set_control("left", 2, -0.8)
            self.controller.set_control("left", 3, -0.8)

        # Check for contacts
        forces = self.sensor_reader.read_forces(self.data, self.FORCE_THRESHOLD)
        left_contacts = sum(1 for site in self.sensor_reader.left_touch_sites
                           if forces.get(site, False))

        # Transition when >2 contacts
        if left_contacts > 2:
            self.state = TaskState.REORIENT
            self.state_timer = 0.0
            self.reorient_phase = 0

    def _state_reorient(self) -> None:
        """
        Reorient phase: finger gaiting cycle (4-phase).

        Each phase lasts ~1.25s (5s / 4 phases).
        Transitions to HANDOFF after 5s.
        """
        phase_duration = 1.25
        current_phase = int(self.state_timer / phase_duration) % 4

        if current_phase != self.reorient_phase:
            self.reorient_phase = current_phase

        # Finger gaiting: cycle through phases to manipulate object orientation
        # Phase 0: lift index
        # Phase 1: lift middle
        # Phase 2: lift ring
        # Phase 3: return to neutral

        if self.reorient_phase == 0:
            self.controller.set_control("left", 1, 0.5)  # lift index
        elif self.reorient_phase == 1:
            self.controller.set_control("left", 2, 0.5)  # lift middle
        elif self.reorient_phase == 2:
            self.controller.set_control("left", 3, 0.5)  # lift ring
        else:  # phase 3
            self.controller.set_control("left", 1, -0.3)
            self.controller.set_control("left", 2, -0.3)
            self.controller.set_control("left", 3, -0.3)

        # Transition after 5s
        if self.state_timer > 5.0:
            self.state = TaskState.HANDOFF
            self.state_timer = 0.0

    def _state_handoff(self) -> None:
        """
        Handoff phase: right hand approaches, dual grip, left releases.

        Transitions to PLACE after 6s.
        """
        # Right hand approaches object
        if self.state_timer < 2.0:
            self.controller.set_control("right", 0, -0.5)  # reach down

        # Dual grip: close right hand (around 2-4s)
        if 2.0 <= self.state_timer < 4.0:
            self.controller.set_control("right", 1, -0.8)
            self.controller.set_control("right", 2, -0.8)
            self.controller.set_control("right", 3, -0.8)

        # Left hand releases (after 4s)
        if self.state_timer >= 4.0:
            self.controller.set_control("left", 1, 0.5)
            self.controller.set_control("left", 2, 0.5)
            self.controller.set_control("left", 3, 0.5)

        # Transition after 6s
        if self.state_timer > 6.0:
            self.state = TaskState.PLACE
            self.state_timer = 0.0

    def _state_place(self) -> None:
        """
        Place phase: lower right hand and release.

        Transitions to COMPLETE after 5s.
        """
        # Lower right hand (raise joint 0)
        if self.state_timer < 3.0:
            self.controller.set_control("right", 0, 0.5)

        # Release grip (open fingers)
        if self.state_timer >= 3.0:
            self.controller.set_control("right", 1, 0.5)
            self.controller.set_control("right", 2, 0.5)
            self.controller.set_control("right", 3, 0.5)

        # Transition after 5s
        if self.state_timer > 5.0:
            self.state = TaskState.COMPLETE
            self.state_timer = 0.0

    def _transition_to_next_state(self) -> None:
        """Force transition to next state (used as timeout fallback)."""
        if self.state == TaskState.EXTRACT:
            self.state = TaskState.REORIENT
        elif self.state == TaskState.REORIENT:
            self.state = TaskState.HANDOFF
        elif self.state == TaskState.HANDOFF:
            self.state = TaskState.PLACE
        elif self.state == TaskState.PLACE:
            self.state = TaskState.COMPLETE

        self.state_timer = 0.0
