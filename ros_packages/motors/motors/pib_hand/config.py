#!/usr/bin/env python3
"""
Configuration Management for PIB Hand Control.

Handles loading and validation of hand_config.yaml with grip definitions
and axis parameters.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import yaml


@dataclass
class AxisConfig:
    """Configuration for a single hand axis."""
    name: str
    motor_name: str
    min_pos: int
    max_pos: int
    max_speed: int
    max_current: int


def load_hand_config(
    config_path: Optional[Path] = None
) -> Tuple[Dict[str, AxisConfig], Dict[str, Dict[str, int]]]:
    """
    Load hand_config.yaml from filesystem with auto-detection.
    
    Auto-detection priority:
    1. Explicit config_path parameter
    2. ROS2 package share directory (ament_index)
    3. Relative path from this module (for standalone simulation)
    
    Args:
        config_path: Optional explicit path to hand_config.yaml
    
    Returns:
        Tuple of (axes_config, grips):
        - axes_config: Dict[axis_name -> AxisConfig]
        - grips: Dict[grip_name -> Dict[axis_name -> position]]
    
    Raises:
        FileNotFoundError: If config file not found at any location
        ValueError: If YAML parsing fails or required fields missing
    
    Example:
        >>> axes, grips = load_hand_config()
        >>> print(axes["daumen"].max_speed)
        2000
        >>> print(grips["SPITZGRIFF"]["daumen"])
        9000
    """
    if config_path is None:
        # Try ROS2 first
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("motors")
            config_path = Path(pkg_share) / "config" / "hand_config.yaml"
        except ImportError:
            # Fallback: relative path from this module
            # motors/motors/pib_hand/config.py -> motors/config/hand_config.yaml
            config_path = Path(__file__).parent.parent.parent / "config" / "hand_config.yaml"
    
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")
    
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    
    if not isinstance(config, dict):
        raise ValueError(f"Invalid YAML structure in {config_path}")
    
    # Parse axes with validation
    axes_config: Dict[str, AxisConfig] = {}
    axes_data = config.get("axes", {})
    
    if not axes_data:
        raise ValueError("No 'axes' section found in config")
    
    for axis_name, axis_data in axes_data.items():
        # Validate required fields
        required_fields = ["motor_name", "min_pos", "max_pos", "max_speed", "max_current"]
        missing = [f for f in required_fields if f not in axis_data]
        if missing:
            raise ValueError(f"Axis '{axis_name}' missing fields: {missing}")
        
        # Validate position range
        if axis_data["min_pos"] >= axis_data["max_pos"]:
            raise ValueError(
                f"Axis '{axis_name}': min_pos ({axis_data['min_pos']}) "
                f"must be < max_pos ({axis_data['max_pos']})"
            )
        
        axes_config[axis_name] = AxisConfig(
            name=axis_name,
            motor_name=axis_data["motor_name"],
            min_pos=axis_data["min_pos"],
            max_pos=axis_data["max_pos"],
            max_speed=axis_data["max_speed"],
            max_current=axis_data["max_current"],
        )
    
    # Parse grips (no strict validation - allow flexible definitions)
    grips = config.get("grips", {})
    
    if not grips:
        raise ValueError("No 'grips' section found in config")
    
    return axes_config, grips


def validate_grip_positions(
    grip_name: str,
    grip_positions: Dict[str, int],
    axes_config: Dict[str, AxisConfig]
) -> None:
    """
    Validate that grip positions are within axis limits.
    
    Args:
        grip_name: Name of the grip for error messages
        grip_positions: Dict mapping axis names to target positions
        axes_config: Dict of axis configurations with limits
    
    Raises:
        ValueError: If any position is out of bounds or axis unknown
    
    Example:
        >>> axes, grips = load_hand_config()
        >>> validate_grip_positions("SPITZGRIFF", grips["SPITZGRIFF"], axes)
    """
    for axis_name, position in grip_positions.items():
        if axis_name not in axes_config:
            raise ValueError(
                f"Grip '{grip_name}' references unknown axis: {axis_name}"
            )
        
        axis = axes_config[axis_name]
        if not (axis.min_pos <= position <= axis.max_pos):
            raise ValueError(
                f"Grip '{grip_name}', axis '{axis_name}': "
                f"position {position} out of range [{axis.min_pos}, {axis.max_pos}]"
            )
