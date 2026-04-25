#!/usr/bin/env python3
"""
Admittance Control Algorithms for PIB Hand.

Core motion control and compliance logic for soft grip execution.
No ROS2 dependencies - pure algorithmic implementation.
"""
from __future__ import annotations


# ============================================================================
# CONSTANTS
# ============================================================================

CONTROL_LOOP_RATE_HZ = 50
"""Control loop frequency in Hz for smooth admittance response."""

POSITION_TOLERANCE = 100
"""Position tolerance in Tinkerforge units for target reached detection."""

ADMITTANCE_GAIN = 1.5
"""
Admittance gain K_a: How many position units the finger yields per mA excess current.

Mathematical formulation:
    q_cmd = q_ref - K_a * max(0, I_measured - I_threshold)

Where:
- q_cmd: Commanded position sent to motor
- q_ref: Reference position from ideal trajectory
- I_measured: Measured motor current
- I_threshold: Current threshold for admittance activation
- K_a: Admittance gain (this constant)

Higher values → softer compliance, more yielding
Lower values → harder compliance, less yielding
"""


# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def calculate_next_reference(current: float, target: float, max_speed: int) -> float:
    """
    Calculate next point on ideal trajectory (q_ref) with speed limiting.
    
    Uses linear interpolation to smoothly approach target position
    without exceeding maximum speed constraint.
    
    Args:
        current: Current reference position in Tinkerforge units
        target: Target position in Tinkerforge units
        max_speed: Maximum speed in units per second
    
    Returns:
        Next reference position (Tinkerforge units)
    
    Example:
        >>> # 50Hz control loop, max_speed=2000 units/s
        >>> q_ref = 0.0
        >>> for _ in range(100):  # 2 seconds
        ...     q_ref = calculate_next_reference(q_ref, 9000, 2000)
        >>> print(q_ref)  # Should be ~4000 (2s * 2000 units/s)
    """
    distance = target - current
    step = max_speed / CONTROL_LOOP_RATE_HZ  # Units per tick
    
    if abs(distance) <= step:
        return target
    
    return current + (step if distance > 0 else -step)


# ============================================================================
# ADMITTANCE LOGIC
# ============================================================================

def apply_admittance_logic(
    q_ref: float,
    current_measured: float,
    current_threshold: float
) -> float:
    """
    Calculate commanded position (q_cmd) with admittance-based compliance.
    
    This implements soft compliance: when measured current exceeds threshold,
    the commanded position deviates back from the reference trajectory.
    This allows fingers to yield under load while continuing motion
    when resistance decreases.
    
    Mathematical formulation:
        excess_current = max(0, I_measured - I_threshold)
        q_cmd = q_ref - K_a * excess_current
    
    Args:
        q_ref: Reference position from ideal trajectory (Tinkerforge units)
        current_measured: Measured motor current in mA
        current_threshold: Current threshold in mA (above triggers admittance)
    
    Returns:
        Commanded position q_cmd (Tinkerforge units)
    
    Behavior:
    - No excess current (I ≤ threshold): q_cmd = q_ref (normal motion)
    - Excess current (I > threshold): q_cmd < q_ref (finger yields back)
    - Current decreases: q_cmd catches up to q_ref (continues motion)
    
    Example:
        >>> q_ref = 5000.0
        >>> # No excess current → normal motion
        >>> q_cmd = apply_admittance_logic(q_ref, 800, 1000)
        >>> print(q_cmd)  # 5000.0 (no yielding)
        >>> 
        >>> # Excess current → finger yields
        >>> q_cmd = apply_admittance_logic(q_ref, 1200, 1000)
        >>> print(q_cmd)  # 4700.0 (yielded back by 1.5 * 200mA)
    
    Integration point for FFNN:
        Future ML-based threshold prediction will replace static threshold:
        
        # Current (static):
        threshold = max_current * 0.7
        
        # Future (FFNN):
        threshold = ffnn_model.predict(q_ref, velocity, finger_id)
        
        The admittance logic itself remains unchanged.
    """
    excess_current = max(0.0, current_measured - current_threshold)
    return q_ref - (ADMITTANCE_GAIN * excess_current)


# ============================================================================
# TARGET DETECTION
# ============================================================================

def is_target_reached(
    current: float,
    target: float,
    tolerance: int = POSITION_TOLERANCE
) -> bool:
    """
    Check if position is within tolerance of target.
    
    Used to detect when reference trajectory has reached its goal,
    regardless of where the commanded position is (may differ due to admittance).
    
    Args:
        current: Current position (Tinkerforge units)
        target: Target position (Tinkerforge units)
        tolerance: Position tolerance (Tinkerforge units), defaults to POSITION_TOLERANCE
    
    Returns:
        True if |current - target| < tolerance
    
    Example:
        >>> is_target_reached(8950, 9000, tolerance=100)
        True
        >>> is_target_reached(8850, 9000, tolerance=100)
        False
    """
    return abs(current - target) < tolerance
