#!/usr/bin/env python3
"""
MuJoCo Hand Simulation - Standalone Testbed for Admittance Control.

Tests pib_hand control algorithms in MuJoCo simulation environment
without requiring ROS2 or real hardware.

Fix: Disabled dynamics & viewer lock to prevent MuJoCo from overriding
kinematic commands with gravity/spring forces.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Dict

import mujoco
import mujoco.viewer
import numpy as np

# Import pib_hand modules (relative imports since we're in pib_hand.simulation)
from ..config import AxisConfig, load_hand_config
from ..admittance import (
    CONTROL_LOOP_RATE_HZ,
    calculate_next_reference,
    apply_admittance_logic,
    is_target_reached,
)
from ..hardware import tinkerforge_to_radians


# ============================================================================
# CONSTANTS & MAPPING
# ============================================================================

COMPLIANCE_THRESHOLD_FACTOR = 0.7

AXIS_TO_MUJOCO_JOINTS = {
    "daumen": "thumb_right_distal",
    "zeigefinger": "index_right_distal",
    "mittelfinger": "middle_right_distal",
    "ringfinger": "ring_right_distal",
    "kleinfinger": "pinky_right_distal",
}

PROXIMAL_COUPLINGS = {
    "thumb_right_distal": "thumb_right_proximal",
    "index_right_distal": "index_right_proximal",
    "middle_right_distal": "middle_right_proximal",
    "ring_right_distal": "ring_right_proximal",
    "pinky_right_distal": "pinky_right_proximal",
}

ELBOW_RIGHT_FIXED_RAD = float(np.deg2rad(90.0))
FOREARM_RIGHT_FIXED_RAD = float(np.deg2rad(180.0))


# ============================================================================
# STATE MACHINE
# ============================================================================

class GripState:
    """Simple state enum for simulation (mirrors controller.GripState)."""
    IDLE = "idle"
    MOVING = "moving"
    COMPLETED = "completed"


@dataclass
class SimAxisState:
    """Simulation axis state (lightweight version of hardware.AxisState)."""
    config: AxisConfig
    start_position: float
    target_position: float
    reference_position: float
    current_cmd_pos: float
    is_finished: bool


# ============================================================================
# GRIP SIMULATOR
# ============================================================================

class GripSimulator:
    """
    Simulates grip execution using pib_hand algorithms.
    
    Uses same control logic as real controller but without hardware access.
    """

    def __init__(
        self,
        axes_config: Dict[str, AxisConfig],
        grips: Dict[str, Dict[str, int]]
    ):
        self.axes_config = axes_config
        self.grips = grips
        self.grip_state = GripState.IDLE
        self.axis_states: Dict[str, SimAxisState] = {}
        self.current_positions: Dict[str, float] = {
            name: 0.0 for name in axes_config.keys()
        }

    def start_grip(self, grip_name: str) -> bool:
        """Start executing a grip."""
        if self.grip_state != GripState.IDLE or grip_name not in self.grips:
            return False

        target_positions = self.grips[grip_name]
        self.axis_states = {}

        for name, config in self.axes_config.items():
            curr = self.current_positions[name]
            target = float(target_positions[name])

            self.axis_states[name] = SimAxisState(
                config=config,
                start_position=curr,
                target_position=target,
                reference_position=curr,
                current_cmd_pos=curr,
                is_finished=False
            )

        self.grip_state = GripState.MOVING
        print(f"▶️ Simulation started: {grip_name}")
        return True

    def update(self) -> None:
        """
        Update simulation state (50Hz tick).
        
        Uses same algorithm as controller._control_loop_tick but without
        hardware current measurement (simulated as 0).
        """
        if self.grip_state != GripState.MOVING:
            return

        all_done = True
        for name, state in self.axis_states.items():
            if state.is_finished:
                continue

            all_done = False

            # 1. Update reference trajectory
            state.reference_position = calculate_next_reference(
                state.reference_position,
                state.target_position,
                state.config.max_speed
            )

            # 2. Apply admittance (no real resistance in simulation)
            meas_current = 0.0
            threshold = state.config.max_current * COMPLIANCE_THRESHOLD_FACTOR
            state.current_cmd_pos = apply_admittance_logic(
                state.reference_position, meas_current, threshold
            )

            # 3. Update position
            self.current_positions[name] = state.current_cmd_pos

            # 4. Check if finished
            if is_target_reached(state.reference_position, state.target_position):
                state.is_finished = True
                print(f"   ✓ {name} completed")

        if all_done:
            print("🏁 Grip completed")
            self.grip_state = GripState.COMPLETED

    def reset(self) -> None:
        """Reset to idle state."""
        self.grip_state = GripState.IDLE
        self.axis_states = {}

    def get_mujoco_positions(self) -> Dict[str, float]:
        """Convert current Tinkerforge positions to MuJoCo radians."""
        mujoco_pos = {}
        for name, tf_pos in self.current_positions.items():
            if name in AXIS_TO_MUJOCO_JOINTS:
                joint = AXIS_TO_MUJOCO_JOINTS[name]
                mujoco_pos[joint] = tinkerforge_to_radians(int(tf_pos))
        return mujoco_pos


# ============================================================================
# MUJOCO SETUP & MAIN
# ============================================================================

def load_mujoco_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load PIB upper body URDF model."""
    model_path = (
        Path(__file__).resolve().parent / "pib_upper_body" / "pib_upper_body.urdf"
    )

    if not model_path.exists():
        raise FileNotFoundError(f"URDF not found: {model_path}")

    # Remove package:// prefix for local loading
    urdf_text = model_path.read_text(encoding="utf-8").replace(
        "package://pib_head_module/meshes/", ""
    )

    with NamedTemporaryFile(
        mode="w", suffix=".urdf", dir=model_path.parent, delete=False, encoding="utf-8"
    ) as tmp:
        tmp.write(urdf_text)
        temp_model_path = Path(tmp.name)

    try:
        model = mujoco.MjModel.from_xml_path(str(temp_model_path))
        data = mujoco.MjData(model)
    finally:
        temp_model_path.unlink(missing_ok=True)

    return model, data


def setup_joint_mappings(
    model: mujoco.MjModel
) -> tuple[list[dict], list[dict], dict[int, float]]:
    """Setup mappings between PIB axes and MuJoCo joints."""
    joint_infos: list[dict] = []
    for joint_name in [
        "thumb_right_distal",
        "index_right_distal",
        "middle_right_distal",
        "ring_right_distal",
        "pinky_right_distal",
    ]:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id >= 0:
            joint_infos.append(
                {"name": joint_name, "qpos_adr": int(model.jnt_qposadr[joint_id])}
            )

    coupled_infos: list[dict] = []
    for distal_name, proximal_name in PROXIMAL_COUPLINGS.items():
        distal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, distal_name)
        prox_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, proximal_name)
        if distal_id >= 0 and prox_id >= 0:
            coupled_infos.append(
                {
                    "distal_qpos_adr": int(model.jnt_qposadr[distal_id]),
                    "prox_qpos_adr": int(model.jnt_qposadr[prox_id]),
                }
            )

    locked_targets: dict[int, float] = {}
    for joint_name in ["elbow_right", "forearm_right"]:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id >= 0:
            qpos_adr = int(model.jnt_qposadr[joint_id])
            if joint_name == "elbow_right":
                locked_targets[qpos_adr] = ELBOW_RIGHT_FIXED_RAD
            elif joint_name == "forearm_right":
                locked_targets[qpos_adr] = FOREARM_RIGHT_FIXED_RAD

    return joint_infos, coupled_infos, locked_targets


def main():
    """Main simulation entry point."""
    # Load config using pib_hand config loader
    axes_config, grips = load_hand_config()
    sim = GripSimulator(axes_config, grips)
    model, data = load_mujoco_model()

    joint_infos, coupled_infos, locked_targets = setup_joint_mappings(model)

    # Keyboard controls
    key_to_grip = {
        "o": "OPEN",
        "s": "SPITZGRIFF",
        "d": "DREIPUNKTGRIFF",
        "k": "SCHLUESSELGRIFF",
        "z": "ZYLINDERGRIFF",
        "h": "HAKENGRIFF",
        "p": "SPHAERISCHER_GRIFF",
    }
    last_pressed_time = {}

    def key_callback(keycode):
        try:
            key_char = chr(keycode).lower()
        except (ValueError, OverflowError):
            return

        if key_char in key_to_grip:
            current_time = time.time()
            # Debounce
            if key_char in last_pressed_time and (
                current_time - last_pressed_time[key_char] < 0.3
            ):
                return
            grip_name = key_to_grip[key_char]
            if grip_name in grips:
                sim.start_grip(grip_name)
                last_pressed_time[key_char] = current_time

    last_update = time.time()
    update_interval = 1.0 / CONTROL_LOOP_RATE_HZ  # 50Hz
    smoothing = 0.3

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        print("🚀 Viewer started - Press keys to control (o, s, d, k, z, h, p)\n")

        while viewer.is_running():
            current_time = time.time()
            if current_time - last_update >= update_interval:
                sim.update()
                last_update = current_time
                if sim.grip_state == GripState.COMPLETED:
                    sim.reset()

            mujoco_positions = sim.get_mujoco_positions()

            # CRITICAL: Lock viewer while overriding positions!
            with viewer.lock():
                # 1. Distal joints
                for info in joint_infos:
                    joint_name = str(info["name"])
                    adr = int(info["qpos_adr"])
                    if joint_name in mujoco_positions:
                        target = mujoco_positions[joint_name]
                        current = float(data.qpos[adr])
                        data.qpos[adr] = current + smoothing * (target - current)

                # 2. Couple proximal joints
                for info in coupled_infos:
                    distal_value = float(data.qpos[int(info["distal_qpos_adr"])])
                    data.qpos[int(info["prox_qpos_adr"])] = distal_value

                # 3. Lock arm joints
                for adr, value in locked_targets.items():
                    data.qpos[adr] = value

                data.qvel[:] = 0.0
                mujoco.mj_forward(model, data)

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
