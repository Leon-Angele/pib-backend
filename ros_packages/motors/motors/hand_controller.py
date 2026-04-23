#!/usr/bin/env python3
"""
Hand Controller Node - Refactored for Admittance Control.

Provides ExecuteGrip action server for grip control with admittance-based compliance.
Runs as fourth node in motors package alongside motor_control, motor_current, relay_control.

Uses direct hardware access via pib_motors.motor for precise 50Hz control.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from pib_motors.motor import Motor, name_to_motors
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Import action from datatypes package
from datatypes.action import ExecuteGrip

# Import shared core utilities
from motors.hand_core import (
    AxisConfig,
    GripState,
    CONTROL_LOOP_RATE_HZ,
    POSITION_TOLERANCE,
    calculate_next_reference,
    apply_admittance_logic,
    is_target_reached,
    load_hand_config,
)


# ============================================================================
# CONSTANTS
# ============================================================================

COMPLIANCE_THRESHOLD = 0.7  # 70% of max_current as basis for admittance start


@dataclass
class AxisState:
    """Runtime state for a single axis during grip execution with admittance control."""
    config: AxisConfig
    motor: Motor
    start_position: float
    target_position: float
    reference_position: float  # Ideal trajectory point (q_ref)
    current_cmd_pos: float     # Currently commanded position (q_cmd)
    is_finished: bool          # Trajectory completed?


# ============================================================================
# HAND CONTROLLER NODE
# ============================================================================

class HandController(Node):
    """
    ROS2 Action Server for hand grip control with admittance-based compliance.
    
    Provides /hand/execute_grip action that moves hand to predefined
    grip configurations with automatic admittance control (soft compliance).
    
    Key features:
    - 50Hz control loop for smooth admittance response
    - Separation of reference trajectory (q_ref) and commanded position (q_cmd)
    - Soft compliance: fingers yield under load but continue when resistance decreases
    - Ready for FFNN integration for dynamic threshold prediction
    """

    def __init__(self):
        super().__init__("hand_controller")
        
        # Parameters
        self.declare_parameter("hand_namespace", "hand")
        self.declare_parameter("dev", False)
        
        self.hand_namespace = self.get_parameter("hand_namespace").value
        self.dev_mode = self.get_parameter("dev").value
        
        # Load configuration
        self.axes_config: Dict[str, AxisConfig] = {}
        self.grips: Dict[str, Dict[str, int]] = {}
        self._load_config()
        
        # State
        self.grip_state = GripState.IDLE
        self.axis_states: Dict[str, AxisState] = {}
        self.current_goal_handle = None
        
        # Action Server
        self._action_server = ActionServer(
            self,
            ExecuteGrip,
            f"/{self.hand_namespace}/execute_grip",
            execute_callback=self._execute_grip_callback,
            goal_callback=self._on_new_grip_goal,
            cancel_callback=self._on_cancel_grip,
        )
        
        # Control Loop Timer (50Hz for admittance control)
        self.create_timer(1.0 / CONTROL_LOOP_RATE_HZ, self._control_loop_tick)
        
        # Optional: JointState publisher for visualization
        self.joint_state_pub = self.create_publisher(
            JointState, f"/{self.hand_namespace}/joint_states", 10
        )
        
        self.get_logger().info(
            f"Hand Controller started [namespace={self.hand_namespace}, "
            f"rate={CONTROL_LOOP_RATE_HZ}Hz, axes={len(self.axes_config)}, "
            f"grips={len(self.grips)}]"
        )
        
        if self.dev_mode:
            self.get_logger().warn("Running in DEV mode - motor access may be limited")

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    def _load_config(self):
        """Load hand_config.yaml from package share directory."""
        try:
            pkg_share = get_package_share_directory("motors")
            config_path = Path(pkg_share) / "config" / "hand_config.yaml"
            
            self.get_logger().info(f"Loading config from: {config_path}")
            
            # Use shared config loader
            self.axes_config, self.grips = load_hand_config(config_path)
            
            self.get_logger().info(
                f"Config loaded: {len(self.axes_config)} axes, {len(self.grips)} grips"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            raise

    # ========================================================================
    # MOTOR ACCESS (Direct Hardware - like motor_control.py)
    # ========================================================================

    def _get_motor(self, motor_name: str) -> Optional[Motor]:
        """
        Get motor instance by name using pib_motors.motor.
        
        Same pattern as motor_control.py - direct hardware access.
        """
        motors_list = name_to_motors.get(motor_name, [])
        if not motors_list:
            self.get_logger().warn(f"Motor not found: {motor_name}")
            return None
        return motors_list[0]

    # ========================================================================
    # ACTION SERVER CALLBACKS
    # ========================================================================

    def _on_new_grip_goal(self, goal_request) -> GoalResponse:
        """Called when new grip goal is received."""
        grip_name = goal_request.grip_name
        
        if self.grip_state != GripState.IDLE:
            self.get_logger().warn(
                f"Grip already active ({self.grip_state.value}), rejecting new goal"
            )
            return GoalResponse.REJECT
        
        if grip_name not in self.grips:
            self.get_logger().error(
                f"Unknown grip: {grip_name} (available: {list(self.grips.keys())})"
            )
            return GoalResponse.REJECT
        
        self.get_logger().info(f"Accepting grip goal: {grip_name}")
        return GoalResponse.ACCEPT

    def _on_cancel_grip(self, goal_handle):
        """Called when grip is cancelled."""
        self.get_logger().info("Grip cancelled by client")
        return CancelResponse.ACCEPT

    def _execute_grip_callback(self, goal_handle):
        """
        Main action execution callback.
        
        Initializes axis states and waits for control loop to complete grip.
        """
        self.current_goal_handle = goal_handle
        grip_name = goal_handle.request.grip_name
        
        self.get_logger().info(f"Executing grip: {grip_name}")
        
        # Initialize axis states
        target_positions = self.grips[grip_name]
        self.axis_states = {}
        
        for axis_name, axis_config in self.axes_config.items():
            motor = self._get_motor(axis_config.motor_name)
            
            if motor is None:
                self.get_logger().error(f"Cannot access motor for axis: {axis_name}")
                continue
            
            # Get current position from motor
            try:
                current_pos = float(motor.get_position()) if not self.dev_mode else 0.0
            except Exception as e:
                self.get_logger().warn(f"Failed to read position for {axis_name}: {e}")
                current_pos = 0.0
            
            target_pos = float(target_positions[axis_name])
            
            self.axis_states[axis_name] = AxisState(
                config=axis_config,
                motor=motor,
                start_position=current_pos,
                target_position=target_pos,
                reference_position=current_pos,  # q_ref starts at current position
                current_cmd_pos=current_pos,     # q_cmd starts at current position
                is_finished=False,
            )
            
            self.get_logger().info(
                f"  {axis_name}: {current_pos:.0f} → {target_pos:.0f}"
            )
        
        # Start movement
        self.grip_state = GripState.MOVING
        
        # Wait for completion (control loop will update state)
        rate = self.create_rate(10)  # Check at 10Hz
        while rclpy.ok() and self.grip_state == GripState.MOVING:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.grip_state = GripState.ABORTED
                result = ExecuteGrip.Result()
                result.success = False
                result.message = "Grip cancelled"
                self.grip_state = GripState.IDLE
                self.axis_states = {}
                self.current_goal_handle = None
                return result
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Return result
        result = ExecuteGrip.Result()
        if self.grip_state == GripState.COMPLETED:
            result.success = True
            result.message = f"Grip '{grip_name}' completed successfully"
            goal_handle.succeed()
            self.get_logger().info(f"✓ Grip completed: {grip_name}")
        else:
            result.success = False
            result.message = f"Grip '{grip_name}' aborted"
            goal_handle.abort()
            self.get_logger().warn(f"✗ Grip aborted: {grip_name}")
        
        # Reset to idle
        self.grip_state = GripState.IDLE
        self.axis_states = {}
        self.current_goal_handle = None
        
        return result

    # ========================================================================
    # CONTROL LOOP (50Hz) - Heart of Admittance Control
    # ========================================================================

    def _control_loop_tick(self):
        """
        Control loop tick at 50Hz - implements admittance control.
        
        For each axis:
        1. Update q_ref (ideal trajectory step)
        2. Measure current motor current
        3. Apply admittance logic to calculate q_cmd
        4. Send q_cmd to hardware
        5. Check if trajectory completed
        """
        if self.grip_state != GripState.MOVING:
            return
        
        all_done = True
        feedback = ExecuteGrip.Feedback()
        motor_currents = []
        
        for axis_name, state in self.axis_states.items():
            if state.is_finished:
                motor_currents.append(0)
                continue
            
            all_done = False
            
            # 1. Update q_ref (ideal trajectory step)
            state.reference_position = calculate_next_reference(
                state.reference_position,
                state.target_position,
                state.config.max_speed,
            )
            
            # 2. Measure real current
            try:
                measured_current = state.motor.get_current() if not self.dev_mode else 0
            except Exception as e:
                self.get_logger().warn(f"Failed to read current for {axis_name}: {e}")
                measured_current = 0
            
            motor_currents.append(measured_current)
            
            # 3. Apply admittance (calculate q_cmd)
            # HERE the FFNN will later dynamically set the threshold!
            static_threshold = state.config.max_current * COMPLIANCE_THRESHOLD
            state.current_cmd_pos = apply_admittance_logic(
                state.reference_position,
                measured_current,
                static_threshold,
            )
            
            # 4. Command to hardware
            try:
                if not self.dev_mode:
                    state.motor.set_position(int(state.current_cmd_pos))
            except Exception as e:
                self.get_logger().warn(f"Failed to set position for {axis_name}: {e}")
            
            # 5. Check if trajectory completed (q_ref reached target)
            if is_target_reached(state.reference_position, state.target_position):
                state.is_finished = True
                self.get_logger().info(
                    f"  ✓ {axis_name} trajectory completed "
                    f"(q_ref={state.reference_position:.0f}, "
                    f"q_cmd={state.current_cmd_pos:.0f})"
                )
            
            # Update feedback
            feedback.current_axis = axis_name
            feedback.progress_percent = self._calculate_progress(state)
        
        # Publish feedback
        feedback.motor_currents = motor_currents
        if self.current_goal_handle is not None:
            self.current_goal_handle.publish_feedback(feedback)
        
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        for axis_name, state in self.axis_states.items():
            js_msg.name.append(state.config.motor_name)  # z.B. "index_right_stretch"
            js_msg.position.append(float(state.current_cmd_pos))
        self.joint_state_pub.publish(js_msg)

        # Check if all axes finished
        if all_done:
            self.grip_state = GripState.COMPLETED
            self.get_logger().info("All axes completed")

    def _calculate_progress(self, state: AxisState) -> float:
        """Calculate progress percentage for an axis based on reference position."""
        total = abs(state.target_position - state.start_position)
        if total == 0:
            return 100.0
        
        current = abs(state.reference_position - state.start_position)
        return min(100.0, (current / total) * 100.0)


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    """Main entry point for hand_controller node."""
    rclpy.init(args=args)
    
    controller = HandController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
