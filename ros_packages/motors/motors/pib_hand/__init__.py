#!/usr/bin/env python3
"""
PIB Hand Control Module - Modular Hand Control System.

Public API for hand control functionality including admittance control,
configuration management, and hardware access.

Usage:
    # For ROS2 nodes
    from motors.pib_hand.controller import HandController, main
    
    # For algorithms/testing
    from motors.pib_hand import (
        load_hand_config,
        calculate_next_reference,
        apply_admittance_logic,
        tinkerforge_to_radians,
    )
    
    # For simulation
    from motors.pib_hand.simulation.hand_sim import main as sim_main
"""

# Configuration
from .config import (
    AxisConfig,
    load_hand_config,
    validate_grip_positions,
)

# Admittance Control
from .admittance import (
    CONTROL_LOOP_RATE_HZ,
    POSITION_TOLERANCE,
    ADMITTANCE_GAIN,
    calculate_next_reference,
    apply_admittance_logic,
    is_target_reached,
)

# Hardware Access
from .hardware import (
    AxisState,
    get_motor_by_name,
    initialize_axis_states,
    tinkerforge_to_radians,
    radians_to_tinkerforge,
)

# Controller (ROS2)
from .controller import (
    HandController,
    GripState,
    COMPLIANCE_THRESHOLD,
)

# Data Logging (optional, for FFNN training)
from .data_logger import (
    TrajectoryLogger,
    VelocityCalculator,
    TrajectoryDataPoint,
    calculate_direction,
)

__all__ = [
    # Config
    "AxisConfig",
    "load_hand_config",
    "validate_grip_positions",
    # Admittance
    "CONTROL_LOOP_RATE_HZ",
    "POSITION_TOLERANCE",
    "ADMITTANCE_GAIN",
    "calculate_next_reference",
    "apply_admittance_logic",
    "is_target_reached",
    # Hardware
    "AxisState",
    "get_motor_by_name",
    "initialize_axis_states",
    "tinkerforge_to_radians",
    "radians_to_tinkerforge",
    # Controller
    "HandController",
    "GripState",
    "COMPLIANCE_THRESHOLD",
    # Data Logger
    "TrajectoryLogger",
    "VelocityCalculator",
    "TrajectoryDataPoint",
    "calculate_direction",
]
