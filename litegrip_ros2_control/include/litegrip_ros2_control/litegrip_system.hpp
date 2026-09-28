// litegrip_system.hpp — ros2_control SystemInterface for the LiteGrip gripper.
//
// The plugin never touches CAN directly: it only reads and writes one
// seqlock-protected shared memory segment, while a separate Python daemon
// (litegrip_hw_daemon) owns can0, runs the safety layer and sends DM MIT
// frames. So read()/write() are pure memcpy plus one unit conversion, and the
// real-time loop has no Python, no socket and no lock in it. The layering is
// exactly isomorphic to litearm_ros2_control.
//
// ★★ Interface surface (decided by the user on 2026-09-23)
// --------------------------------------------------------
//   Command interface: **position only** — matching the arm joints, with MIT
//             position control underneath. The trajectory advance rate ceiling
//             and the torque budget are **daemon parameters**, not per-command
//             fields (they describe "how this gripper is allowed to move" and
//             belong to the deployment configuration; they must not be decided
//             by each command publisher individually — otherwise one slip of
//             the hand could loosen the whole torque argument).
//   State interface: position / velocity / effort — matching the arm joints'
//             standard trio, so joint_state_broadcaster treats both identically
//             and the layers above need not distinguish the device. Plus
//             diagnostics (temperature / error_code / fault_code / feedback age
//             / daemon status), which are not part of the standard trio and will
//             not be claimed by JSB.
//
// ★★ Units: one number, one unit
// ------------------------------
//   ROS joint side (interface storage)  opening in **m** — matches the URDF joint limits
//   shm (daemon side)                   motor angle in **rad** — the red line, torque
//                                       budget and MIT quantization are all defined and
//                                       verified in this unit
//   Conversion is the **presentation layer**'s job and is concentrated in this
//   file's two private methods (see the formulas below).
//
// ★★ Conversion formulas (source: sdk/litegrip/physical_calibration.json, via
//    the same implementation as
//    litearm_manipulation/include/.../gripper_motor_scale.hpp):
//
//       opening_mm = (closed_rad − rad) × rad_to_mm
//       rad        = closed_rad − opening_mm / rad_to_mm
//
//   ⇒ **the more negative rad is, the wider the opening** (red_open_rad = −1.24
//     corresponds to fully open 83.32 mm, red_close_rad = −0.01 to 3.34 mm).
//     So the velocity conversion **must flip sign**:
//
//       opening velocity (m/s) = −v_rad × rad_to_mm × 1e-3
//
//   ⚠ The sign is the one place here that is "inconspicuous but wrong": miss
//     the minus sign and the position is right while the velocity is inverted,
//     and velocity feedback is often used to decide "stopped or not" — an
//     inverted sign raises no error, it just quietly makes that decision wrong.
//
// ★★ Why clamp here (instead of leaving out-of-range values for the daemon to reject)
// ------------------------------------------------------------------------------------
//   The URDF joint range is the **model layer's** 0~87 mm, while the
//   commandable range is limited by the **red line** to only ≈3.34~83.32 mm
//   (with margin left at both ends). The daemon **rejects the whole** of an
//   out-of-range command (it does not clamp), so without clamping here a
//   perfectly normal command like "fully open 87 mm" would be dropped entirely,
//   the gripper would not move at all and all that would be left behind is a 1
//   in fault_code — a phenomenon that is very hard to pin down.
//   ⇒ The presentation layer first clamps the target into the **commandable
//     range**, and the red line remains the daemon's second line of defence.

#ifndef LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_
#define LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "litegrip_ros2_control/litegrip_shm.h"

namespace litegrip_ros2_control {

/** Standard state interface names (corresponding to the hardware_interface
 *  constants). */
inline constexpr char kStatePosition[] = "position";
inline constexpr char kStateVelocity[] = "velocity";
inline constexpr char kStateEffort[] = "effort";
/** Diagnostic state interface names. */
inline constexpr char kStateTemperatureMos[] = "temperature_mos";
inline constexpr char kStateTemperatureCoil[] = "temperature_coil";
inline constexpr char kStateErrorCode[] = "error_code";
inline constexpr char kStateFaultCode[] = "fault_code";
inline constexpr char kStateFeedbackAge[] = "feedback_age";

/** Command interface name. **position only**, see the top of the file. */
inline constexpr char kCommandPosition[] = "position";

/**
 * The LiteGrip gripper (single joint), bridged to the Python daemon through
 * shared memory.
 *
 * Lifecycle
 * ---------
 * on_init       parse the URDF parameters and the joint name; check that the
 *               conversion parameters are positive finite numbers
 * on_configure  open the shared memory segment with create=false (the segment
 *               must already have been created by the daemon), wait for the
 *               daemon's heartbeat/connection to become ready, and set up the
 *               emergency-stop service
 * on_activate   initialize the command position to the **measured position** and
 *               publish the first frame (avoids a jump at the instant of
 *               activation)
 * read          take one frame from the state block (seqlock, bounded retries)
 *               and convert it into opening / opening velocity
 * write         convert the command position into a motor angle (with clamping),
 *               pack one frame and write it back into the command block
 * on_deactivate send one hold-in-place command frame, then stop
 *
 * ★ Emergency stop: the only command interface is position, and a ros2_control
 *   **hardware component has no access to a node** (only to a logger), so the
 *   plugin **cannot** provide an emergency-stop service of its own. This layer
 *   handles it exactly as the arm does: the shm command frame carries an
 *   `estop` field (the daemon honours it), and this round the plugin **always
 *   writes 0** — the field is **reserved**, waiting for the upper-layer
 *   migration round to give it a real source.
 *   ⚠ Relative to the old driver, this round therefore **loses one software
 *   emergency-stop entry point** (the old node used the `emergency_stop`
 *   parameter). That is a deliberately recorded known gap, not an oversight; a
 *   true power-cut emergency stop can only ever be the responsibility of the
 *   hardware circuit (a software emergency stop takes no effect when the
 *   process hangs or CAN drops, the same under the old and the new design).
 */
class LitegripSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LitegripSystem)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  /** Effective control mode; determines whether read()/write() really exchange
   *  data. */
  enum class Mode { kUnconfigured, kConfigured, kActive, kStopped };

  /** Pack the current command interface values into one command frame and
   *  publish it. */
  void publish_command();
  /** Convert the latest state from shared memory into the state interface
   *  storage. */
  void apply_state(const LitegripState &state);
  /** Pull the command position to the measured position, as a hold-in-place
   *  command frame. */
  void latch_command_to_measured();
  /** Read CLOCK_MONOTONIC seconds with millisecond precision; the same source as
   *  Python's time.monotonic(). */
  static double monotonic_seconds();

  // ── Conversion (the single place units are converted; formulas at the top) ──
  /** Opening m → motor angle rad, with clamping into the "commandable range"
   *  (the margin left at each end of the red line is described in the header). */
  double width_to_rad(double width) const;
  /** Motor angle rad → opening m (the reading may slightly exceed the model
   *  range; clamped to [0, max_width]). */
  double rad_to_width(double rad) const;
  /** Whether this motor angle falls inside the red line (for diagnostics). */
  bool within_red_line(double rad) const;

  // ── Conversion parameters (source: physical_calibration.json + safety baseline) ──
  double closed_rad_ = 0.04139;   // motor angle at opening = 0
  double rad_to_mm_ = 65.0231;    // mm / rad
  double red_open_rad_ = -1.24;   // red line, open end
  double red_close_rad_ = -0.01;  // red line, closed end
  double min_width_ = 0.0;        // model-layer opening lower bound, m
  double max_width_ = 0.087;      // model-layer opening upper bound, m

  // ── Other parameters (URDF <hardware><param>) ─────────────────────────
  std::string shm_name_ = LITEGRIP_SHM_DEFAULT_NAME;
  std::string joint_name_ = "gripper_opening_joint";
  double connect_timeout_s_ = 10.0;
  double heartbeat_timeout_s_ = 1.0;
  int state_read_retries_ = 8;
  int configure_read_retries_ = 512;
  bool export_diagnostics_ = true;

  Mode mode_ = Mode::kUnconfigured;
  litegrip_shm_handle_t shm_ = nullptr;

  // ── Interface storage (a single joint, hence scalars) ────────────────
  // ROS joint-side units: position/velocity in m and m/s, torque in N·m
  // (the driver readings are passed through without a gear-ratio conversion).
  double state_position_ = 0.0;
  double state_velocity_ = 0.0;
  double state_effort_ = 0.0;
  double state_temperature_mos_ = 0.0;
  double state_temperature_coil_ = 0.0;
  double state_error_code_ = 0.0;
  double state_fault_code_ = 0.0;
  double state_feedback_age_ = -1.0;

  double command_position_ = 0.0;

  // ── Latest state frame and daemon health ─────────────────────────────
  LitegripState state_buffer_{};
  bool have_state_ = false;
  double last_heartbeat_s_ = 0.0;
  double command_cycle_ = 0.0;
  double last_applied_cycle_ = -1.0;
  bool daemon_alive_ = false;
  bool reported_daemon_loss_ = false;
  bool reported_latched_ = false;
  bool reported_fault_ = false;
  // Self-managed log throttle timestamps. RCLCPP_*_THROTTLE is not used because
  // it holds a static rclcpp::Clock at the call site and constructs it on first
  // use — and this path runs on the real-time thread.
  double last_fault_log_s_ = 0.0;
  double last_stale_log_s_ = 0.0;
  std::uint64_t torn_state_reads_ = 0;

  // ── Emergency stop: always 0 this round (reserved field); see the class comment ──

  rclcpp::Logger logger_ = rclcpp::get_logger("LitegripSystem");
};

}  // namespace litegrip_ros2_control

#endif  // LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_
