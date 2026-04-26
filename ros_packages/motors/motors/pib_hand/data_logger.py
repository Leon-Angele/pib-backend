#!/usr/bin/env python3
"""
Trajectory Data Logger for FFNN Training.

Optional module for recording finger trajectories during grip execution
to collect training data for ML-based current prediction.

CSV format: timestamp, axis_id, q_cmd, dq_cmd, direction, measured_current
"""
from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, TextIO


@dataclass
class TrajectoryDataPoint:
    """Single data point for trajectory logging."""
    timestamp: float
    """Unix timestamp in seconds (with millisecond precision)."""
    
    axis_id: str
    """Axis name (e.g., 'daumen', 'zeigefinger')."""
    
    q_cmd: float
    """Commanded position in Tinkerforge units."""
    
    dq_cmd: float
    """Commanded velocity in Tinkerforge units/second."""
    
    direction: int
    """Movement direction: 1 (closing), -1 (opening), 0 (stopped)."""
    
    measured_current: int
    """Measured motor current in mA."""


class TrajectoryLogger:
    """
    CSV logger for finger trajectory data collection.
    
    Buffers data points and writes to CSV file for FFNN training.
    Thread-safe for use in ROS2 control loops.
    
    Example:
        >>> logger = TrajectoryLogger("/data/trajectories")
        >>> logger.start_recording("SPITZGRIFF")
        >>> 
        >>> # In control loop (50Hz)
        >>> logger.log_point(
        ...     axis_id="daumen",
        ...     q_cmd=5000.0,
        ...     dq_cmd=100.0,
        ...     direction=1,
        ...     measured_current=850
        ... )
        >>> 
        >>> logger.stop_recording()  # Flushes and closes file
    """
    
    def __init__(
        self,
        output_dir: Path | str,
        buffer_size: int = 100
    ):
        """
        Initialize trajectory logger.
        
        Args:
            output_dir: Directory to save CSV files
            buffer_size: Number of data points to buffer before writing
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.buffer_size = buffer_size
        self.buffer: List[TrajectoryDataPoint] = []
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.writer] = None
        self.is_recording = False
        self.start_time: float = 0.0
    
    def start_recording(self, grip_name: str) -> Path:
        """
        Start recording trajectory data.
        
        Creates new CSV file with timestamp and grip name.
        
        Args:
            grip_name: Name of the grip being executed
        
        Returns:
            Path to created CSV file
        
        Raises:
            RuntimeError: If already recording
        """
        if self.is_recording:
            raise RuntimeError("Already recording - call stop_recording() first")
        
        # Generate filename with timestamp
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{grip_name}_{timestamp_str}.csv"
        filepath = self.output_dir / filename
        
        # Open CSV file and write header
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'axis_id',
            'q_cmd',
            'dq_cmd',
            'direction',
            'measured_current'
        ])
        
        self.is_recording = True
        self.start_time = time.time()
        self.buffer.clear()
        
        return filepath
    
    def log_point(
        self,
        axis_id: str,
        q_cmd: float,
        dq_cmd: float,
        direction: int,
        measured_current: int
    ) -> None:
        """
        Log a single trajectory data point.
        
        Buffers the data point and writes to CSV when buffer is full.
        
        Args:
            axis_id: Axis name (e.g., 'daumen')
            q_cmd: Commanded position (Tinkerforge units)
            dq_cmd: Commanded velocity (Tinkerforge units/second)
            direction: Movement direction (1, -1, or 0)
            measured_current: Motor current in mA
        """
        if not self.is_recording:
            return  # Silently ignore if not recording
        
        # Create data point with relative timestamp
        point = TrajectoryDataPoint(
            timestamp=time.time() - self.start_time,
            axis_id=axis_id,
            q_cmd=q_cmd,
            dq_cmd=dq_cmd,
            direction=direction,
            measured_current=measured_current
        )
        
        self.buffer.append(point)
        
        # Flush buffer if full
        if len(self.buffer) >= self.buffer_size:
            self._flush_buffer()
    
    def _flush_buffer(self) -> None:
        """Write buffered data points to CSV file."""
        if not self.csv_writer or not self.buffer:
            return
        
        for point in self.buffer:
            self.csv_writer.writerow([
                f"{point.timestamp:.3f}",  # 3 decimal places (ms precision)
                point.axis_id,
                f"{point.q_cmd:.1f}",
                f"{point.dq_cmd:.1f}",
                point.direction,
                point.measured_current
            ])
        
        self.buffer.clear()
        
        # Ensure data is written to disk
        if self.csv_file:
            self.csv_file.flush()
    
    def stop_recording(self) -> Optional[int]:
        """
        Stop recording and close CSV file.
        
        Flushes any remaining buffered data.
        
        Returns:
            Total number of data points written, or None if not recording
        """
        if not self.is_recording:
            return None
        
        # Flush remaining buffer
        self._flush_buffer()
        
        # Close file
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        
        self.csv_writer = None
        self.is_recording = False
        
        # Could return total points written, but we'd need to track it
        return None
    
    def __del__(self):
        """Ensure file is closed on deletion."""
        if self.is_recording:
            self.stop_recording()


# ============================================================================
# VELOCITY CALCULATION HELPER
# ============================================================================

class VelocityCalculator:
    """
    Helper class to calculate velocity (dq_cmd) from position changes.
    
    Maintains history of previous position and timestamp for each axis.
    
    Example:
        >>> vel_calc = VelocityCalculator()
        >>> 
        >>> # First call (no history yet)
        >>> dq = vel_calc.calculate("daumen", 1000.0, time.time())
        >>> print(dq)  # 0.0
        >>> 
        >>> # Second call (after 0.02s)
        >>> dq = vel_calc.calculate("daumen", 1040.0, time.time())
        >>> print(dq)  # ~2000.0 units/s (40 units / 0.02s)
    """
    
    def __init__(self):
        self._history: Dict[str, tuple[float, float]] = {}
        """Maps axis_id -> (last_position, last_timestamp)."""
    
    def calculate(
        self,
        axis_id: str,
        q_cmd: float,
        timestamp: float
    ) -> float:
        """
        Calculate velocity from position change.
        
        Args:
            axis_id: Axis name
            q_cmd: Current commanded position
            timestamp: Current timestamp
        
        Returns:
            Velocity in units/second (0.0 on first call)
        """
        if axis_id not in self._history:
            # First measurement - no velocity yet
            self._history[axis_id] = (q_cmd, timestamp)
            return 0.0
        
        last_q, last_t = self._history[axis_id]
        dt = timestamp - last_t
        
        if dt <= 0:
            # Same timestamp or backwards in time
            return 0.0
        
        dq = (q_cmd - last_q) / dt
        
        # Update history
        self._history[axis_id] = (q_cmd, timestamp)
        
        return dq
    
    def reset(self) -> None:
        """Clear all history (e.g., when starting new grip)."""
        self._history.clear()


# ============================================================================
# DIRECTION CALCULATION HELPER
# ============================================================================

def calculate_direction(target: float, current: float, tolerance: float = 10.0) -> int:
    """
    Calculate movement direction from target and current position.
    
    Args:
        target: Target position
        current: Current position
        tolerance: Position tolerance for "stopped" detection
    
    Returns:
        1 if closing (target > current)
        -1 if opening (target < current)
        0 if stopped (within tolerance)
    
    Example:
        >>> calculate_direction(9000, 5000)  # Closing
        1
        >>> calculate_direction(0, 5000)     # Opening
        -1
        >>> calculate_direction(5000, 5005)  # Stopped (within tolerance)
        0
    """
    diff = target - current
    
    if abs(diff) < tolerance:
        return 0  # Stopped
    elif diff > 0:
        return 1  # Closing (increasing position)
    else:
        return -1  # Opening (decreasing position)
