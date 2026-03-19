# Runtime Module Responsibilities

## Purpose

This document is the unified source of truth for module responsibilities in
the ROS 2 runtime workspace. It uses the public names defined in `plan.md`
and is intended to be split into future per-module files such as
`{module}/docs/responsibilities.md`.

The goal is simple: each module must be understandable without relying on
repository-specific history or private abbreviations.


## System Boundary

- This ROS 2 repository owns the internal real-time runtime.
- The external task service stays outside this repository.
- All external task requests must enter through `task_service_bridge`.
- No module may bypass the declared interfaces to reach hardware directly.


## motion_control

**Purpose**

- Own robot motion planning, inverse kinematics, constraints, trajectory
  generation, and closed-loop control.

**Inputs**

- Task-level execution goals from `task_service_bridge`
- Perception updates from `vision_perception`
- Execution feedback from `execution_manager`

**Outputs**

- Motion commands to `execution_manager`
- Task progress and control status to `task_service_bridge`

**Dependencies**

- `motion_msgs`
- `perception_msgs`
- `task_api_msgs`

**Non-Responsibilities**

- No direct serial access
- No servo protocol logic
- No speech, vision inference, or LLM integration

**Failure Modes**

- Invalid target
- Constraint violation
- Replan required
- Control timeout


## speech_interface

**Purpose**

- Own audio capture, speech-to-text, and text-to-speech.

**Inputs**

- Microphone/audio stream
- Speech synthesis requests

**Outputs**

- Transcripts to `task_service_bridge`
- Playback status if needed

**Dependencies**

- `speech_msgs`

**Non-Responsibilities**

- No task interpretation
- No execution control
- No dialogue policy

**Failure Modes**

- Recognition failure
- Device unavailable
- Synthesis failure
- Interrupt or cancel


## vision_perception

**Purpose**

- Own camera ingestion, object detection, tracking, pose estimation, and
  scene summaries.

**Inputs**

- Image frames
- Video streams
- Camera calibration and sensor metadata

**Outputs**

- Structured object lists
- Scene state summaries
- Pose and tracking results

**Dependencies**

- `perception_msgs`

**Non-Responsibilities**

- No task planning
- No servo control
- No long-term memory or retrieval

**Failure Modes**

- Camera unavailable
- Low-confidence detection
- Tracking lost
- Pose estimation failure


## execution_manager

**Purpose**

- Own execution coordination between control and hardware.
- Validate motion commands, dispatch them to drivers, and aggregate feedback.
- Enforce safety at the execution boundary.

**Inputs**

- Motion commands from `motion_control`
- Hardware state from `servo_hardware` and `sensor_hardware`

**Outputs**

- Driver-level commands
- Structured execution feedback
- Execution result and safety events

**Dependencies**

- `motion_msgs`
- `servo_msgs`

**Non-Responsibilities**

- No high-level task interpretation
- No trajectory planning
- No external task service integration

**Failure Modes**

- Invalid command
- Driver unavailable
- Emergency stop
- Timeout or safety violation


## servo_hardware

**Purpose**

- Own bus servo, PWM servo, and related actuator driver logic.

**Inputs**

- Driver commands from `execution_manager`

**Outputs**

- Servo state
- Hardware error and diagnostics

**Dependencies**

- `servo_msgs`

**Non-Responsibilities**

- No task semantics
- No motion planning
- No direct external API exposure

**Failure Modes**

- Communication error
- Overload
- Overheat
- Invalid device mapping


## sensor_hardware

**Purpose**

- Own low-level sensor access such as IMU and future onboard sensors.

**Inputs**

- Sensor polling or hardware interrupts

**Outputs**

- Raw or minimally processed sensor state
- Driver diagnostics

**Dependencies**

- Sensor-specific messages or standard ROS sensor messages

**Non-Responsibilities**

- No task-level reasoning
- No perception inference
- No control decision making

**Failure Modes**

- Read failure
- Calibration error
- Device disconnected
- Timestamp drift


## task_service_bridge

**Purpose**

- Serve as the only formal bridge between the external task service and the
  ROS runtime.

**Inputs**

- External task requests
- Session context signals
- Internal status from `motion_control`
- Optional transcripts and perception summaries

**Outputs**

- Task execution requests into ROS
- Task status, dialogue replies, and result summaries back to the external
  task service

**Dependencies**

- `task_api_msgs`
- `speech_msgs`
- `perception_msgs`

**Non-Responsibilities**

- No direct driver control
- No servo command publishing
- No hardware protocol access

**Failure Modes**

- Invalid task payload
- Bridge timeout
- Downstream module unavailable
- Cancel or session mismatch


## teleoperation_bridge

**Purpose**

- Provide human teleoperation and debugging entry points.

**Inputs**

- WebSocket or other manual control streams

**Outputs**

- Teleoperation commands into the runtime
- Debug status to operator tools

**Dependencies**

- `task_api_msgs` or debug-only internal interfaces

**Non-Responsibilities**

- No formal production task orchestration
- No ownership of the main task execution path
- No implicit control priority over autonomous execution

**Failure Modes**

- Connection drop
- Invalid operator command
- Arbitration rejection
- Unsafe manual override


## simulation_bridge

**Purpose**

- Bridge the runtime to simulators for testing, replay, and validation.

**Inputs**

- Motion commands
- Simulator configuration

**Outputs**

- Simulated state feedback
- Simulated execution result

**Dependencies**

- `motion_msgs`
- `servo_msgs` where compatibility is required

**Non-Responsibilities**

- No production hardware control
- No task reasoning
- No direct external service integration

**Failure Modes**

- Simulator unavailable
- State sync mismatch
- Unsupported command mode


## Interface Packages

### task_api_msgs

**Purpose**

- Define the public ROS-facing contract used by the external task service.

**Owns**

- Task action definitions
- Task status messages
- Dialogue reply messages
- Context signal messages

**Does Not Own**

- Driver-level control messages
- Hardware diagnostics


### motion_msgs

**Purpose**

- Define internal control and execution coordination messages.

**Owns**

- Motion commands
- State feedback
- Execution results

**Does Not Own**

- Task dialogue
- Raw hardware protocols


### speech_msgs

**Purpose**

- Define speech input/output contracts.

**Owns**

- Transcript
- Speech synthesis request
- Optional speech status

**Does Not Own**

- Task execution status
- Motion commands


### perception_msgs

**Purpose**

- Define structured perception contracts.

**Owns**

- Object list
- Scene state
- Pose or tracking summaries

**Does Not Own**

- Hardware driver state
- Task dialogue


### servo_msgs

**Purpose**

- Define the low-level actuator contract used at the execution boundary.

**Owns**

- Servo command
- Servo state
- Servo services for diagnostics and reads

**Does Not Own**

- Task-level goals
- Perception outputs
- Dialogue or transcript messages


## Cross-Module Rules

1. `task_service_bridge` is the only formal external entry point.
2. `motion_control` cannot talk to serial drivers directly.
3. `execution_manager` cannot interpret task semantics.
4. `servo_hardware` and `sensor_hardware` cannot expose private driver
   details as public task APIs.
5. `teleoperation_bridge` must remain a debug or manual-control path, not
   the primary production task path.
6. Every new module must document:
   Purpose, Inputs, Outputs, Dependencies, Non-Responsibilities, and Failure
   Modes.


## Recommended Split

When the new directory structure is created, this file should be split into:

- `control/motion_control/docs/responsibilities.md`
- `speech/speech_interface/docs/responsibilities.md`
- `perception/vision_perception/docs/responsibilities.md`
- `execution/execution_manager/docs/responsibilities.md`
- `execution/servo_hardware/docs/responsibilities.md`
- `execution/sensor_hardware/docs/responsibilities.md`
- `bridges/task_service_bridge/docs/responsibilities.md`
- `bridges/teleoperation_bridge/docs/responsibilities.md`
- `bridges/simulation_bridge/docs/responsibilities.md`

Until then, this document remains the unified responsibility reference.
