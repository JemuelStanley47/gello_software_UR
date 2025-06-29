# Changelog

## Minor Updates ([(Commit)](https://github.com/JemuelStanley47/gello_software_UR/commit/3f776ea622de3f170874c29c7767a562afa87e13)) ([(Commit)](https://github.com/JemuelStanley47/gello_software_UR/commit/b21e320b10a76207353109f495ac01e622aa959a)) 
### Added 
- `send.trajectory.launch.py`: Launch file to initialize Gello-to-UR trajectory streaming node.
- `gello_to_ur_trajectory.py`: Node that listens to `/gello/joint_states` and publishes to UR ROS 2 Driver's `scaled_joint_trajectory_controller`.
- Integration with `ros2_control`-based UR driver using position control via `JointTrajectory` messages.
- Update `setup.py` to include new executable script.

### Changed
- Refactored Gello UR state publisher for cleaner exit handling and improved configuration parsing aligned with Gello hardware setup.
- Ensured compatibility with both IsaacSim robot and URSim for validation.

### Notes
- The UR driver already launches the necessary controllers; this node acts as a bridge for joint command streaming.
- Currently uses position control; movement is responsive after tuning `time_from_start` and update `rate`.
- Velocity limit warnings appear on UR teach pendant—velocity control mode may be considered in the future for smoother operation.


## Minor Updates [(Commit)](https://github.com/JemuelStanley47/gello_software_UR/commit/a2729f5d09c446b3a27c6b04b1b8424ba9cd149f)

- **Agent Integration**  
  Replaced direct usage of `DynamixelDriver` with `GelloAgent`, enabling cleaner abstraction and extensibility. Introduced `URGelloAgent` as a wrapper for easy configuration-based instantiation.

- **YAML-Driven Configuration**  
  Robot joint parameters (`joint_ids`, `offsets`, `signs`), gripper configuration (`gripper_config`), and publish rates (`publish_rate`) are now entirely configurable via `gello_config.yaml`. Removed all hardcoded joint setup logic.

- **Dynamic Publish Rate Resolution**  
  Added logic to resolve the effective publish rate with the following preference order:
  1. Launch argument (`publish_rate`)
  2. YAML config (`publish_rate`)
  3. Estimated max polling rate from hardware (`estimate_driver_polling_rate`)

  Final selected rate is clamped to stay within safe hardware polling limits.

- **Auto Estimation of Polling Rate**  
  Added method `estimate_driver_polling_rate()` using `time.perf_counter()` to benchmark and determine the maximum frequency at which the hardware can be polled.

- **Clean-up and Refactor**  
  - Removed legacy and hardcoded driver logic in favor of agent-based abstraction.
  - Added proper docstrings and type hints for clarity and maintainability.
  - Removed commented-out code and unused fallback logic.
  - Dropped unnecessary dynamic `sys.path` hacks in favor of clean import setup.

## Initial Updates [(Commit)](https://github.com/JemuelStanley47/gello_software_UR/commit/5dd76baafc02feba5e73e1572cea4e260bc5acde)

### Added
- Created a **separate ROS 2 package** for UR Gello state publisher (`ur_gello_state_publisher`) instead of modifying the existing Franka version.  
  _Note: Future enhancement could involve unifying both under a single scalable architecture and submitting a PR to the official repo._

- Support for **`joint_names`** and **`publish_rate`** in YAML configuration:
  - `joint_names` improves clarity and introspection in `/joint_states`.
  - `publish_rate` (e.g., `500 Hz`) allows hardware-specific tuning.

- Added **launch argument** `publish_rate` to override config or fallback values from the CLI.

- New method: `estimate_driver_polling_rate()` — uses timing (`perf_counter`) to estimate the **maximum polling frequency** supported by the hardware.

- Introduced `resolve_publish_rate()` method to select the final publish rate with the following **preference hierarchy**:
  1. User-supplied via launch argument
  2. Config-specified rate
  3. Hardware-estimated rate

### Changed
- ROS 2 publishers now use **QoS profiles** instead of raw depth integers to improve consistency and real-time compatibility.

### Fixed
- -

---

