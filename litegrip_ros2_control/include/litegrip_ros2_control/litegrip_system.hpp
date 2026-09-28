// litegrip_system.hpp — ros2_control SystemInterface for the LiteGrip gripper.
//
// This layer is now a THIN SHELL over litegrip_cpp::ControlLoop. It used to
// bridge to a separate Python daemon through a seqlock shared-memory segment,
// because a ros2_control hardware component must be a C++ pluginlib plugin
// while the SDK was Python-only. That reason is gone: the SDK is C++, so the
// plugin links it directly and there is no second process, no shared memory and
// no Python anywhere in the stack.
//
// Everything the old daemon did — rate-limiting the target, allocating the
// torque budget, applying the safety gate, streaming DM MIT frames, watching
// feedback, temperature and faults — lives inside ControlLoop, on the SDK's own
// thread. This file only does what a ros2_control component must do:
//
//   · parse and validate the URDF parameters,
//   · own the ControlLoop lifecycle (configure -> start, cleanup -> stop),
//   · convert between the ROS joint unit (opening in m) and the motor angle
//     (rad) that the SDK works in,
//   · publish the cached state into the state interfaces.
//
// ★★ Interface surface (unchanged — decided by the user on 2026-09-23)
// --------------------------------------------------------------------
//   Command: **position only**. The trajectory rate ceiling and the torque
//            budget are DEPLOYMENT parameters of this component, not
//            per-command fields: they describe "how this gripper is allowed to
//            move" and must not be decided by each command publisher.
//   State:   position / velocity / effort — the same trio as the arm joints, so
//            joint_state_broadcaster treats both identically — plus
//            diagnostics (temperature / error_code / fault_code / feedback
//            age), which JSB does not claim.
//
// ★★ Units: one number, one unit
// ------------------------------
//   ROS joint side (interface storage)  opening in **m** — matches the URDF
//   SDK side                            motor angle in **rad** — the red lines,
//                                       the torque budget and MIT quantization
//                                       are all defined and verified in rad
//
//       opening_mm = (closed_rad − rad) × rad_to_mm
//       rad        = closed_rad − opening_mm / rad_to_mm
//
//   ⇒ **the more negative rad is, the wider the opening**, so the velocity
//     conversion MUST flip sign:
//
//       opening velocity (m/s) = −v_rad × rad_to_mm × 1e-3
//
//   ⚠ The sign is the one place here that is "inconspicuous but wrong": miss it
//     and the position stays right while the velocity is inverted. Velocity
//     feedback is what decides "stopped or not", so an inverted sign raises no
//     error — it just quietly makes that decision wrong.
//
// ★★ Clamping: still here, and now for a simpler reason
// -----------------------------------------------------
//   The URDF joint range is the MODEL layer's 0~87 mm, while the commandable
//   range is narrower because the red lines bound it. The SDK **rejects** an
//   out-of-range target (it does not clamp), so without clamping here a
//   perfectly ordinary "fully open 87 mm" command would be dropped whole, the
//   gripper would not move at all, and all that would be left behind is a fault
//   code — a phenomenon that is hard to pin down.
//   ⇒ This layer clamps the target into the **commandable range**, and the red
//     line remains the SDK's second line of defence.
//
//   ⚠ The commandable range is derived from the red lines the SDK actually
//     loaded (ControlLoop::safety_limits()), NOT from a second copy in the
//     URDF. The old version had red_open_rad/red_close_rad as URDF parameters,
//     which meant two sources of truth for the same safety quantity; that
//     duplication is gone.

#ifndef LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_
#define LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <litegrip/control_loop.hpp>

namespace litegrip_ros2_control {

/** Standard state interface names (the hardware_interface constants). */
inline constexpr char kStatePosition[] = "position";
inline constexpr char kStateVelocity[] = "velocity";
inline constexpr char kStateEffort[] = "effort";
/** Diagnostic state interface names. */
inline constexpr char kStateTemperatureMos[] = "temperature_mos";
inline constexpr char kStateTemperatureCoil[] = "temperature_coil";
inline constexpr char kStateErrorCode[] = "error_code";
inline constexpr char kStateFaultCode[] = "fault_code";
inline constexpr char kStateFeedbackAge[] = "feedback_age";

/** Command interface name. position only, see the top of the file. */
inline constexpr char kCommandPosition[] = "position";

/**
 * The LiteGrip gripper (single joint), as a ros2_control hardware component
 * backed directly by the C++ SDK.
 *
 * Lifecycle
 * ---------
 * on_init       parse and validate the parameters; reject bad ones at
 *               configuration time rather than degrading into an invented
 *               default (exactly the behaviour a safety parameter must never
 *               have)
 * on_configure  construct the ControlLoop and start it. With dry_run=true this
 *               touches no hardware at all, which is what makes a dry-run stack
 *               meaningful; otherwise it opens CAN, enables the motor and leaves
 *               it holding its current position
 * on_activate   latch the command position to the MEASURED opening, then tell
 *               the loop to follow commands
 * read          copy the loop's cached snapshot into the state interfaces
 * write         convert the command position into a target and post it
 * on_deactivate keep holding at the measured position — do NOT disable: a
 *               disabled gripper drops whatever it is holding
 * on_cleanup    stop the loop (zero torque, then disable and close CAN)
 *
 * ★ Emergency stop: a ros2_control hardware component has no access to a node
 *   (only to a logger), so this layer cannot offer an emergency-stop service of
 *   its own. The old design reserved a shared-memory `estop` field for an
 *   upper-layer migration; that field no longer exists. The loop's own
 *   emergency_stop() is available to whoever owns it, and a true power-cut
 *   emergency stop is the hardware circuit's job in any case (a software one
 *   takes no effect when the process hangs or CAN drops).
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
  /** Convert the command opening (m) into a motor angle (rad), clamped into the
   *  commandable range derived from the loaded red lines. */
  double width_to_rad(double width) const;

  /** Convert a motor angle (rad) into an opening (m), clamped to the model
   *  range (a reading may slightly exceed it). */
  double rad_to_width(double rad) const;

  /** The opening range the red lines actually permit: (min_m, max_m). */
  std::pair<double, double> commandable_width_range() const;

  /** Read CLOCK_MONOTONIC seconds; the same source as Python's
   *  time.monotonic(). */
  static double monotonic_seconds();

  // ── Conversion parameters (from the calibration) ──────────────────────
  double closed_rad_ = 0.04139;  // motor angle at opening = 0
  double rad_to_mm_ = 65.0231;   // mm / rad
  double min_width_ = 0.0;       // model-layer opening lower bound (m)
  double max_width_ = 0.087;     // model-layer opening upper bound (m)

  std::string joint_name_ = "gripper_opening_joint";
  bool export_diagnostics_ = true;

  /** The whole control path, including the safety gate and the frame stream. */
  std::unique_ptr<litegrip::ControlLoop> loop_;
  /** Validated configuration, kept so on_configure builds the loop from exactly
   *  the values on_init checked. */
  litegrip::ControlLoopConfig loop_config_;
  bool active_ = false;

  // ── Interface storage (a single joint, hence scalars) ─────────────────
  // ROS joint-side units: position/velocity in m and m/s, torque in N·m (the
  // driver reading is passed through with no gear-ratio conversion).
  double state_position_ = 0.0;
  double state_velocity_ = 0.0;
  double state_effort_ = 0.0;
  double state_temperature_mos_ = 0.0;
  double state_temperature_coil_ = 0.0;
  double state_error_code_ = 0.0;
  double state_fault_code_ = 0.0;
  double state_feedback_age_ = -1.0;

  double command_position_ = 0.0;

  // Log throttling timestamps. Self-managed rather than RCLCPP_*_THROTTLE: the
  // latter holds a static rclcpp::Clock at the call site and constructs it on
  // first use, and these paths run on the real-time thread.
  double last_fault_log_s_ = 0.0;
  double last_stale_log_s_ = 0.0;
  bool reported_fault_ = false;

  rclcpp::Logger logger_ = rclcpp::get_logger("LitegripSystem");
};

}  // namespace litegrip_ros2_control

#endif  // LITEGRIP_ROS2_CONTROL__LITEGRIP_SYSTEM_HPP_
