#!/usr/bin/env python3
"""
Shared core utilities for PIB Hand Control - Refactored for Admittance Control.

Used by both:
- hand_controller.py (ROS2 node in motors package)
- hand_simulation/hand_sim.py (standalone MuJoCo simulation)

No ROS2 dependencies in this module!
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml


# ============================================================================
# CONSTANTS
# ============================================================================

CONTROL_LOOP_RATE_HZ = 50  # Increased from 20Hz to 50Hz for smoother admittance
POSITION_TOLERANCE = 100   # Tinkerforge units
ADMITTANCE_GAIN = 1.5      # K_a: How many units the finger yields per mA excess current


# ============================================================================
# ENUMS & DATACLASSES
# ============================================================================

class GripState(Enum):
    """State machine for grip execution."""
    IDLE = "idle"
    MOVING = "moving"
    COMPLETED = "completed"
    ABORTED = "aborted"


@dataclass
class AxisConfig:
    """Configuration for a single hand axis."""
    name: str
    motor_name: str
    min_pos: int
    max_pos: int
    max_speed: int
    max_current: int


# ============================================================================
# CONFIGURATION LOADING
# ============================================================================

def load_hand_config(config_path: Optional[Path] = None) -> Tuple[Dict[str, AxisConfig], Dict[str, Dict[str, int]]]:
    """
    Load hand_config.yaml from filesystem.
    
    Args:
        config_path: Optional path to hand_config.yaml. If None, tries to auto-detect:
                    - ROS2: Uses ament_index to find package share directory
                    - Standalone: Uses relative path from this file
    
    Returns:
        Tuple of (axes_config, grips):
        - axes_config: Dict[axis_name -> AxisConfig]
        - grips: Dict[grip_name -> Dict[axis_name -> position]]
    
    Raises:
        FileNotFoundError: If config file not found
        ValueError: If config parsing fails
    """
    if config_path is None:
        # Try ROS2 first
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("motors")
            config_path = Path(pkg_share) / "config" / "hand_config.yaml"
        except ImportError:
            # Fallback: relative path from this file (for standalone simulation)
            # motors/motors/hand_core.py -> motors/config/hand_config.yaml
            config_path = Path(__file__).parent.parent / "config" / "hand_config.yaml"
    
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")
    
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    
    # Parse axes
    axes_config: Dict[str, AxisConfig] = {}
    for axis_name, axis_data in config.get("axes", {}).items():
        axes_config[axis_name] = AxisConfig(
            name=axis_name,
            motor_name=axis_data["motor_name"],
            min_pos=axis_data["min_pos"],
            max_pos=axis_data["max_pos"],
            max_speed=axis_data["max_speed"],
            max_current=axis_data["max_current"],
        )
    
    # Parse grips
    grips = config.get("grips", {})
    
    return axes_config, grips


# ============================================================================
# MOTION CONTROL & ADMITTANCE
# ============================================================================

def calculate_next_reference(current: float, target: float, max_speed: int) -> float:
    """
    Calculate next point on ideal trajectory (q_ref).
    
    Linear interpolation with speed limit.
    
    Args:
        current: Current reference position (Tinkerforge units)
        target: Target position (Tinkerforge units)
        max_speed: Maximum speed in units per second
    
    Returns:
        Next reference position (Tinkerforge units)
    """
    distance = target - current
    step = max_speed / CONTROL_LOOP_RATE_HZ  # Units per tick
    
    if abs(distance) <= step:
        return target
    
    return current + (step if distance > 0 else -step)


def apply_admittance_logic(q_ref: float, current_measured: float, current_threshold: float) -> float:
    """
    Calculate commanded position (q_cmd) based on current measurement.
    
    This is the classic basis into which the FFNN will later be integrated.
    When excess current is detected, q_cmd deviates back from q_ref.
    
    Args:
        q_ref: Reference position (ideal trajectory point)
        current_measured: Measured motor current in mA
        current_threshold: Threshold current in mA (above this triggers admittance)
    
    Returns:
        Commanded position q_cmd (Tinkerforge units)
    """
    excess_current = max(0.0, current_measured - current_threshold)
    # When excess current exists, q_cmd deviates back from q_ref
    return q_ref - (ADMITTANCE_GAIN * excess_current)


def is_target_reached(current: float, target: float, tolerance: int = POSITION_TOLERANCE) -> bool:
    """
    Check if target position is reached within tolerance.
    
    Args:
        current: Current position (Tinkerforge units)
        target: Target position (Tinkerforge units)
        tolerance: Position tolerance (Tinkerforge units)
    
    Returns:
        True if within tolerance
    """
    return abs(current - target) < tolerance


# ============================================================================
# UNIT CONVERSION (for MuJoCo simulation)
# ============================================================================

def tinkerforge_to_radians(tf_position: int, tf_range: int = 9000) -> float:
    """
    Convert Tinkerforge position (0-9000) to radians for MuJoCo.
    
    Assumes:
    - 0 = fully open (0 rad)
    - 9000 = fully closed (90 degrees = π/2 rad)
    
    Args:
        tf_position: Position in Tinkerforge units (0-9000)
        tf_range: Maximum Tinkerforge position (default 9000)
    
    Returns:
        Position in radians (0 to π/2)
    """
    # Normalize to 0-1
    normalized = tf_position / tf_range
    
    # Map to 0 to π/2 (90 degrees)
    return normalized * (np.pi / 2.0)


def radians_to_tinkerforge(radians: float, tf_range: int = 9000) -> int:
    """
    Convert radians to Tinkerforge position (0-9000).
    
    Inverse of tinkerforge_to_radians().
    
    Args:
        radians: Position in radians (0 to π/2)
        tf_range: Maximum Tinkerforge position (default 9000)
    
    Returns:
        Position in Tinkerforge units (0-9000)
    """
    # Normalize from π/2 to 0-1
    normalized = radians / (np.pi / 2.0)
    
    # Map to 0-9000
    return int(normalized * tf_range)
