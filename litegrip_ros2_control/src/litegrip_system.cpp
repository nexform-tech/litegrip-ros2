// litegrip_system.cpp — LiteGrip gripper SystemInterface implementation.
//
// Real-time notes
// ---------------
// read() copies the ControlLoop's cached snapshot (a mutex-protected struct
// copy, no I/O) and write() posts one target. Neither opens a socket, waits on
// the bus, allocates, or touches Python — all of that happens on the SDK's own
// control thread. So the controller_manager cycle stays cheap regardless of how
// the CAN traffic is behaving.

#include "litegrip_ros2_control/litegrip_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <litegrip/constants.hpp>
#include <litegrip/exceptions.hpp>

#ifndef LITEGRIP_DEFAULT_DATA_DIR
#define LITEGRIP_DEFAULT_DATA_DIR ""
#endif

namespace litegrip_ros2_control {
namespace {

/** Log throttle period (seconds). */
constexpr double kLogThrottleS = 5.0;

/**
 * Make the SDK's data files findable regardless of the process working
 * directory.
 *
 * The SDK resolves its calibration and safety-baseline files at run time and
 * deliberately has no machine-dependent path compiled in, so a bare version name
 * like "3.5" is searched for relative to the working directory. A
 * controller_manager, however, is started from wherever the user happened to be
 * standing, which makes that search fail — and a missing safety baseline is
 * fail-closed, so the component refuses to start.
 *
 * Precedence, strongest first:
 *   1. an explicit path in the safety_baseline parameter (handled by the SDK),
 *   2. LITEGRIP_DATA_DIR already in the environment (the deployment's choice),
 *   3. the directory litegrip_cpp was installed into, recorded at build time.
 *
 * setenv(..., overwrite=0) so an operator's value is never clobbered.
 */
void ensure_sdk_data_dir_known() {
  if (std::getenv("LITEGRIP_DATA_DIR") != nullptr) {
    return;
  }
  if (LITEGRIP_DEFAULT_DATA_DIR[0] == '\0') {
    return;
  }
  ::setenv("LITEGRIP_DATA_DIR", LITEGRIP_DEFAULT_DATA_DIR, 0);
}

std::string param_or(const hardware_interface::HardwareInfo &info,
                     const std::string &key, const std::string &fallback) {
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

/**
 * Read a double parameter. A parse failure or a non-finite number always throws
 * — a wrong parameter must be reported **at configuration time** and must never
 * quietly degrade into an invented default (exactly the behaviour safety
 * parameters must never have).
 */
double param_double(const hardware_interface::HardwareInfo &info,
                    const std::string &key, double fallback) {
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  const double value = std::stod(it->second);
  if (!std::isfinite(value)) {
    throw std::invalid_argument("parameter " + key + " is not finite: " +
                                it->second);
  }
  return value;
}

bool param_bool(const hardware_interface::HardwareInfo &info,
                const std::string &key, bool fallback) {
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  const std::string &text = it->second;
  return text == "true" || text == "True" || text == "1";
}

int param_int(const hardware_interface::HardwareInfo &info,
              const std::string &key, int fallback) {
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return std::stoi(it->second);
}

/** The joint name from the `<hardware>` block (this component has exactly 1). */
std::string sole_joint_name(const hardware_interface::HardwareInfo &info) {
  if (info.joints.size() != 1u) {
    throw std::invalid_argument(
        "this component supports 1 joint only (a real LiteGrip has a single "
        "transmission and the two fingers are mimic); the URDF gave " +
        std::to_string(info.joints.size()) + " of them");
  }
  return info.joints.front().name;
}

}  // namespace

double LitegripSystem::monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ───────────────────────── conversion (the only place) ─────────────────────────

double LitegripSystem::rad_to_width(double rad) const {
  const double mm = (closed_rad_ - rad) * rad_to_mm_;
  return std::clamp(mm * 1e-3, 0.0, std::max(max_width_, 0.0));
}

std::pair<double, double> LitegripSystem::commandable_width_range() const {
  // The red lines come from the SDK's loaded baseline — the single source of
  // truth — not from a second copy in the URDF parameters.
  double red_min = litegrip::kPackageRedMin;
  double red_max = litegrip::kPackageRedMax;
  if (loop_ != nullptr) {
    const litegrip::SafetyLimits &limits = loop_->safety_limits();
    if (limits.enabled) {
      red_min = std::min(limits.red_min_rad, limits.red_max_rad);
      red_max = std::max(limits.red_min_rad, limits.red_max_rad);
    }
  }
  // red_min is the wider (more negative) angle => the larger opening.
  const double width_at_red_min = (closed_rad_ - red_min) * rad_to_mm_ * 1e-3;
  const double width_at_red_max = (closed_rad_ - red_max) * rad_to_mm_ * 1e-3;
  const double lo = std::max(min_width_, std::min(width_at_red_min,
                                                  width_at_red_max));
  const double hi = std::min(max_width_, std::max(width_at_red_min,
                                                  width_at_red_max));
  return {lo, std::max(lo, hi)};
}

double LitegripSystem::width_to_rad(double width) const {
  // ① Clamp into the **commandable range** first: the model layer's 0~87 mm is
  //    wider than the red lines permit, and without clamping an ordinary
  //    "fully open 87 mm" would be rejected whole by the gate and nothing would
  //    move (see the header for details).
  const auto range = commandable_width_range();
  const double clamped = std::clamp(width, range.first, range.second);
  // ② Then clamp the angle by the red lines (belt and braces: if the conversion
  //    parameters were corrupted, this still cannot cross them).
  double red_min = litegrip::kPackageRedMin;
  double red_max = litegrip::kPackageRedMax;
  if (loop_ != nullptr) {
    const litegrip::SafetyLimits &limits = loop_->safety_limits();
    red_min = std::min(limits.red_min_rad, limits.red_max_rad);
    red_max = std::max(limits.red_min_rad, limits.red_max_rad);
  }
  return std::clamp(closed_rad_ - clamped * 1e3 / rad_to_mm_, red_min, red_max);
}

// ───────────────────────── lifecycle ─────────────────────────

hardware_interface::CallbackReturn LitegripSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Do this before anything can try to load a safety baseline: the baseline is
  // looked up by version name, and the working directory is not a reliable place
  // to find it from.
  ensure_sdk_data_dir_known();

  litegrip::ControlLoopConfig config;
  try {
    joint_name_ = sole_joint_name(info);

    closed_rad_ = param_double(info, "closed_rad", closed_rad_);
    rad_to_mm_ = param_double(info, "rad_to_mm", rad_to_mm_);
    min_width_ = param_double(info, "min_width", min_width_);
    max_width_ = param_double(info, "max_width", max_width_);

    config.channel = param_or(info, "channel", litegrip::DefaultParams::kCanChannel);
    config.can_id = param_int(info, "can_id", litegrip::GripperParams::kCanId);
    if (info.hardware_parameters.find("mst_id") != info.hardware_parameters.end()) {
      config.mst_id = param_int(info, "mst_id", litegrip::GripperParams::kMstId);
    }
    config.canfd_mode = param_bool(info, "canfd_mode", false);

    // The dual switch. dry_run=true must be the default: an unconfigured stack
    // must never reach for hardware.
    config.dry_run = param_bool(info, "dry_run", true);
    config.hardware_enable = param_bool(info, "hardware_enable", false);

    config.control_rate_hz = param_double(info, "control_rate_hz", config.control_rate_hz);
    config.feedback_timeout_s =
        param_double(info, "feedback_timeout_s", config.feedback_timeout_s);
    config.temperature_limit_c =
        param_int(info, "temperature_limit_c", config.temperature_limit_c);
    config.command_timeout_s =
        param_double(info, "command_timeout_s", config.command_timeout_s);

    config.max_velocity_rad_s =
        param_double(info, "max_velocity_rad_s", config.max_velocity_rad_s);
    config.torque_limit_nm =
        param_double(info, "torque_limit_nm", config.torque_limit_nm);
    config.safety_baseline =
        param_or(info, "safety_baseline", litegrip::kDefaultSafetyBaseline);
    config.max_position_error_rad =
        param_double(info, "max_position_error_rad", config.max_position_error_rad);
    config.max_feedback_velocity_rad_s = param_double(
        info, "max_feedback_velocity_rad_s", config.max_feedback_velocity_rad_s);

    config.kp = param_double(info, "kp", config.kp);
    config.kd = param_double(info, "kd", config.kd);

    config.pos_closed_rad = closed_rad_;
    config.rad_to_mm = rad_to_mm_;
    // The model layer's full-open angle, for consistency with the URDF limits.
    config.pos_open_rad = closed_rad_ - max_width_ * 1e3 / rad_to_mm_;

    export_diagnostics_ = param_bool(info, "export_diagnostics", true);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(logger_, "invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // A "mm per rad" figure of zero would make the conversion diverge.
  if (!(rad_to_mm_ > 0.0)) {
    RCLCPP_ERROR(logger_,
                 "rad_to_mm must be a positive finite number, got %g — the "
                 "conversion would diverge",
                 rad_to_mm_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(max_width_ >= min_width_)) {
    RCLCPP_ERROR(logger_, "max_width(%g) must be >= min_width(%g)", max_width_,
                 min_width_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_,
              "gripper hardware interface initialized: joint=%s channel=%s "
              "dry_run=%s hardware_enable=%s, calibration closed_rad=%g "
              "rad_to_mm=%g, model opening [%g, %g] m, safety baseline=%s",
              joint_name_.c_str(), config.channel.c_str(),
              config.dry_run ? "true" : "false",
              config.hardware_enable ? "true" : "false", closed_rad_, rad_to_mm_,
              min_width_, max_width_, config.safety_baseline.c_str());

  loop_config_ = config;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    loop_ = std::make_unique<litegrip::ControlLoop>(loop_config_);
    // start() loads the safety baseline (fail-closed on a missing/invalid file),
    // validates the deploy ceilings, and — unless dry_run — opens CAN, enables
    // the motor and leaves it holding its current position.
    loop_->start();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(logger_,
                 "could not start the gripper control loop: %s\n"
                 "  ① dry_run=false requires hardware_enable=true\n"
                 "  ② is the CAN interface up: ip -details link show %s\n"
                 "  ③ is the safety baseline reachable: %s",
                 error.what(), loop_config_.channel.c_str(),
                 loop_config_.safety_baseline.c_str());
    loop_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_,
              "gripper control loop started (dry_run=%s); red lines in effect: "
              "[%g, %g] rad",
              loop_config_.dry_run ? "true" : "false",
              loop_->safety_limits().red_min_rad,
              loop_->safety_limits().red_max_rad);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (loop_ == nullptr) {
    RCLCPP_ERROR(logger_,
                 "the control loop is not running in on_activate (did "
                 "on_configure fail?)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ★ Latch the command position to the MEASURED opening before letting the loop
  //   follow commands.
  //
  //   Whatever the command interface holds at this instant must be treated as
  //   meaningless, not as a goal. ros2_control initialises command interfaces to
  //   0.0, and that 0.0 is a fully closed gripper — so treating it as a goal
  //   makes the gripper travel to the closed end the moment the stack comes up,
  //   from wherever it actually was. Latching makes the target EQUAL the current
  //   position, so the initial value is irrelevant and the gripper simply stays
  //   put.
  const litegrip::GripperState state = loop_->state();
  if (!state.has_data() || !std::isfinite(state.position_rad)) {
    // Never latch a placeholder: without a real measurement there is no "current
    // position" to hold, and inventing one would command a motion nobody asked
    // for. Refuse instead — on_configure's init() already waits for a status
    // frame, so reaching here means something is genuinely wrong.
    RCLCPP_ERROR(logger_,
                 "refusing to activate: no valid position measurement yet "
                 "(feedback age %.3f s; inf means never received). The command "
                 "position cannot be latched to 'where it currently is', and "
                 "latching it to a default would command a motion nobody asked "
                 "for.",
                 state.data_age_s);
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_position_ = rad_to_width(state.position_rad);
  loop_->set_target_mm(command_position_ * 1e3);
  loop_->set_enable(true);
  active_ = true;

  RCLCPP_INFO(logger_,
              "gripper hardware interface activated: holding the measured "
              "position (%.4f rad -> %.4f m opening); the command interface's "
              "initial value was ignored",
              state.position_rad, command_position_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // ★ On handing control back, HOLD position rather than disable: disabling the
  //   gripper means it may drop whatever it is holding. The direction matches
  //   the arm (whose disable_on_shutdown also defaults to false and holds
  //   position).
  if (loop_ != nullptr) {
    command_position_ = rad_to_width(loop_->state().position_rad);
    loop_->set_target_mm(command_position_ * 1e3);
  }
  active_ = false;
  RCLCPP_INFO(logger_,
              "gripper hardware interface deactivated (holding at the measured "
              "opening %.4f m)",
              command_position_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (loop_ != nullptr) {
    loop_->stop();  // zero torque, then disable and close CAN
    loop_.reset();
  }
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // The same teardown as on_cleanup; written separately because the semantics
  // differ (shutdown = the process is about to go away).
  return on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn LitegripSystem::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (loop_ != nullptr) {
    loop_->stop();
    loop_.reset();
  }
  active_ = false;
  RCLCPP_ERROR(logger_,
               "gripper hardware interface entered the error state: "
               "controller_manager will stop the controllers");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ───────────────────────── interface export ─────────────────────────

std::vector<hardware_interface::StateInterface>
LitegripSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  // The standard trio: one to one with the arm joints, and
  // joint_state_broadcaster recognizes these three.
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION,
                          &state_position_);
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY,
                          &state_velocity_);
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT,
                          &state_effort_);

  if (export_diagnostics_) {
    // Diagnostics are not part of the standard trio and JSB will not claim
    // them; when controller_manager activates it will warn that "these
    // interfaces are unclaimed" — that is expected, and if it is too noisy set
    // export_diagnostics to false.
    interfaces.emplace_back(joint_name_, kStateTemperatureMos,
                            &state_temperature_mos_);
    interfaces.emplace_back(joint_name_, kStateTemperatureCoil,
                            &state_temperature_coil_);
    interfaces.emplace_back(joint_name_, kStateErrorCode, &state_error_code_);
    interfaces.emplace_back(joint_name_, kStateFaultCode, &state_fault_code_);
    interfaces.emplace_back(joint_name_, kStateFeedbackAge,
                            &state_feedback_age_);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
LitegripSystem::export_command_interfaces() {
  // ★ position only. The trajectory rate ceiling and the torque budget are
  //   component deployment parameters, not per-command fields.
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION,
                          &command_position_);
  return interfaces;
}

// ───────────────────────── read / write ─────────────────────────

hardware_interface::return_type LitegripSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (loop_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }

  // A cached snapshot: no CAN, no locks held across I/O, no allocation.
  const litegrip::GripperState state = loop_->state();

  state_position_ = rad_to_width(state.position_rad);
  // ★ The velocity MUST flip sign: the more negative rad is, the wider the
  //   opening, so d(opening)/dt = −v_rad × rad_to_mm×1e-3. Missing the minus
  //   sign raises no error — it only quietly inverts the "stopped or not"
  //   decision, so it is written out explicitly here.
  state_velocity_ = -state.velocity_rad_s * rad_to_mm_ * 1e-3;
  // Torque passes the driver reading straight through (no gear-ratio
  // conversion, same as the old driver).
  state_effort_ = state.torque_nm;
  state_temperature_mos_ = state.temperature_mos;
  state_temperature_coil_ = state.temperature_coil;
  state_error_code_ = state.error_code;
  state_fault_code_ = loop_->fault_code();
  state_feedback_age_ = state.data_age_s;

  const double now = monotonic_seconds();
  if (state_fault_code_ != 0.0) {
    if (!reported_fault_ && now - last_fault_log_s_ > kLogThrottleS) {
      reported_fault_ = true;
      last_fault_log_s_ = now;
      RCLCPP_ERROR(logger_,
                   "gripper fault code %d — the layer above should stop and "
                   "investigate on the strength of this",
                   static_cast<int>(state_fault_code_));
    }
  } else {
    reported_fault_ = false;
  }
  if (state.is_stale() && now - last_stale_log_s_ > kLogThrottleS) {
    last_stale_log_s_ = now;
    RCLCPP_WARN(logger_,
                "gripper feedback is stale (age %.2fs) — the state is frozen "
                "and whether the hardware is currently controllable is "
                "unknown",
                state.data_age_s);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LitegripSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (loop_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }
  if (!active_) {
    // While not active no command is posted: the loop's own staleness rule
    // makes it hold position.
    return hardware_interface::return_type::OK;
  }

  // A non-finite number is never sent: every comparison involving NaN is false,
  // so it would slip past every range check in the gate (that is the one input
  // shape it cannot catch).
  if (!std::isfinite(command_position_)) {
    RCLCPP_ERROR(logger_,
                 "command position is not finite (%g) — not posting this "
                 "target",
                 command_position_);
    return hardware_interface::return_type::OK;
  }

  // The SDK works in motor radians; the interface is in metres.
  loop_->set_target_rad(width_to_rad(command_position_));
  return hardware_interface::return_type::OK;
}

}  // namespace litegrip_ros2_control

PLUGINLIB_EXPORT_CLASS(litegrip_ros2_control::LitegripSystem,
                       hardware_interface::SystemInterface)
