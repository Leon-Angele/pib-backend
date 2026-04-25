#!/usr/bin/env python3
"""
Hardware Access Layer for PIB Hand Control.

Provides abstraction for Tinkerforge motor access, unit conversions,
and axis state management.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from pib_motors.motor import Motor, name_to_motors
from .config import AxisConfig


# ============================================================================
# STATE MANAGEMENT
# ============================================================================

@dataclass
class AxisState:
    """
    Runtime state for a single axis during grip execution with admittance control.
    
    Tracks both the ideal reference trajectory (q_ref) and the actually
    commanded position (q_cmd), which may differ due to admittance compliance.
    """
    config: AxisConfig
    """Axis configuration (limits, speed, current threshold)."""
    
    motor: Motor
    """Tinkerforge motor instance for hardware access."""
    
    start_position: float
    """Initial position at grip execution start (Tinkerforge units)."""
    
    target_position: float
    """Final target position from grip definition (Tinkerforge units)."""
    
    reference_position: float
    """
    Current point on ideal trajectory (q_ref, Tinkerforge units).
    
    This advances smoothly toward target regardless of load/compliance.
    Used to determine when trajectory is complete.
    """
    
    current_cmd_pos: float
    """
    Currently commanded position sent to motor (q_cmd, Tinkerforge units).
    
    May deviate from reference_position due to admittance control.
    When excess current detected, q_cmd < q_ref (finger yields).
    """
    
    is_finished: bool
    """True when reference_position has reached target_position."""


# ============================================================================
# MOTOR ACCESS
# ============================================================================

def get_motor_by_name(motor_name: str) -> Optional[Motor]:
    """
    Get Tinkerforge motor instance by name.
    
    Uses pib_motors.motor.name_to_motors dictionary for lookup.
    Same pattern as motor_control.py - direct hardware access.
    
    Args:
        motor_name: Motor name as defined in PIB API (e.g., "thumb_right_stretch")
    
    Returns:
        Motor instance if found, None otherwise
    
    Example:
        >>> motor = get_motor_by_name("index_right_stretch")
        >>> if motor:
        ...     position = motor.get_position()
        ...     current = motor.get_current()
    """
    motors_list = name_to_motors.get(motor_name, [])
    if not motors_list:
        return None
    return motors_list[0]


def initialize_axis_states(
    axes_config: Dict[str, AxisConfig],
    grip_positions: Dict[str, int],
    dev_mode: bool = False
) -> Dict[str, AxisState]:
    """
    Initialize axis states for grip execution.
    
    Creates AxisState instances for all configured axes, reads current
    positions from hardware (or uses 0 in dev mode), and sets targets.
    
    Args:
        axes_config: Dict of axis configurations
        grip_positions: Target positions for the grip (axis_name -> position)
        dev_mode: If True, skip hardware access (use position=0)
    
    Returns:
        Dict mapping axis names to initialized AxisState instances
    
    Raises:
        ValueError: If motor not found for an axis (in production mode)
    
    Example:
        >>> from pib_hand.config import load_hand_config
        >>> axes, grips = load_hand_config()
        >>> states = initialize_axis_states(axes, grips["SPITZGRIFF"])
        >>> for name, state in states.items():
        ...     print(f"{name}: {state.start_position} → {state.target_position}")
    """
    axis_states: Dict[str, AxisState] = {}
    
    for axis_name, axis_config in axes_config.items():
        # Get motor instance
        motor = get_motor_by_name(axis_config.motor_name)
        
        if motor is None:
            if not dev_mode:
                raise ValueError(
                    f"Cannot access motor '{axis_config.motor_name}' for axis '{axis_name}'"
                )
            # In dev mode, create dummy state without real motor
            # (motor field will be None but code won't use it)
            current_pos = 0.0
        else:
            # Read current position from hardware
            try:
                current_pos = float(motor.get_position()) if not dev_mode else 0.0
            except Exception as e:
                # Fallback to 0 on read error
                current_pos = 0.0
        
        target_pos = float(grip_positions[axis_name])
        
        axis_states[axis_name] = AxisState(
            config=axis_config,
            motor=motor,  # type: ignore (may be None in dev mode)
            start_position=current_pos,
            target_position=target_pos,
            reference_position=current_pos,  # q_ref starts at current
            current_cmd_pos=current_pos,     # q_cmd starts at current
            is_finished=False,
        )
    
    return axis_states


# ============================================================================
# UNIT CONVERSION (for MuJoCo simulation)
# ============================================================================

def tinkerforge_to_radians(tf_position: int, tf_range: int = 9000) -> float:
    """
    Convert Tinkerforge position (0-9000) to radians for MuJoCo simulation.
    
    Mapping:
    - 0 = fully open (0 rad)
    - 9000 = fully closed (90 degrees = π/2 rad)
    
    Args:
        tf_position: Position in Tinkerforge units (0-9000)
        tf_range: Maximum Tinkerforge position (default 9000)
    
    Returns:
        Position in radians (0 to π/2)
    
    Example:
        >>> tinkerforge_to_radians(0)      # Fully open
        0.0
        >>> tinkerforge_to_radians(9000)   # Fully closed
        1.5707963267948966  # π/2
        >>> tinkerforge_to_radians(4500)   # Half closed
        0.7853981633974483  # π/4
    """
    normalized = tf_position / tf_range
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
    
    Example:
        >>> radians_to_tinkerforge(0.0)              # Fully open
        0
        >>> radians_to_tinkerforge(np.pi / 2.0)      # Fully closed
        9000
        >>> radians_to_tinkerforge(np.pi / 4.0)      # Half closed
        4500
    """
    normalized = radians / (np.pi / 2.0)
    return int(normalized * tf_range)
