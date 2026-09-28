// litegrip_system.cpp — LiteGrip gripper SystemInterface implementation.
//
// Real-time notes
// ---------------
// read()/write() do only this: one seqlock read or write (memcpy) plus a
// constant number of floating-point operations. No locks, no allocations, no
// socket, no Python. The seqlock read side has a bounded retry count
// (state_read_retries_); once it is exhausted the previous frame is kept and OK
// is returned — so controller_manager keeps running and the layer above decides
// whether to stop from fault_code / feedback age (same strategy as the arm).

#include "litegrip_ros2_control/litegrip_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace litegrip_ros2_control {
namespace {

/** Log throttle period (seconds). Self-managed instead of using
 *  RCLCPP_*_THROTTLE: the latter's static Clock is constructed on the first
 *  call, and this path runs on the real-time thread. */
constexpr double kLogThrottleS = 5.0;

/** Poll interval while on_configure waits for the daemon. A non-real-time
 *  context, so sleeping is allowed. */
constexpr auto kReadyPollInterval = std::chrono::milliseconds(50);

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

bool LitegripSystem::within_red_line(double rad) const {
  const double lo = std::min(red_open_rad_, red_close_rad_);
  const double hi = std::max(red_open_rad_, red_close_rad_);
  return rad >= lo - 1e-9 && rad <= hi + 1e-9;
}

double LitegripSystem::width_to_rad(double width) const {
  // ① Clamp the width into the **commandable range** first: the model layer's
  //    0~87 mm is wider than the ≈3.34~83.32 mm the red line permits, and
  //    without clamping "fully open 87 mm" would be rejected wholesale by the
  //    daemon (see the header for details).
  const double min_commandable =
      std::max(min_width_, rad_to_width(red_close_rad_));
  const double max_commandable =
      std::min(max_width_, rad_to_width(red_open_rad_));
  const double clamped = std::clamp(width, min_commandable, max_commandable);
  // ② Then clamp the angle by the red line (belt and braces: if the conversion
  //    parameters were corrupted, this still cannot cross the line).
  const double lo = std::min(red_open_rad_, red_close_rad_);
  const double hi = std::max(red_open_rad_, red_close_rad_);
  return std::clamp(closed_rad_ - clamped * 1e3 / rad_to_mm_, lo, hi);
}

// ───────────────────────── lifecycle ─────────────────────────

hardware_interface::CallbackReturn LitegripSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    joint_name_ = sole_joint_name(info);

    closed_rad_ = param_double(info, "closed_rad", closed_rad_);
    rad_to_mm_ = param_double(info, "rad_to_mm", rad_to_mm_);
    red_open_rad_ = param_double(info, "red_open_rad", red_open_rad_);
    red_close_rad_ = param_double(info, "red_close_rad", red_close_rad_);
    min_width_ = param_double(info, "min_width", min_width_);
    max_width_ = param_double(info, "max_width", max_width_);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(logger_, "invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The conversion parameter has to be a positive "mm per rad" figure; zero
  // would make the conversion produce inf.
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

  shm_name_ = param_or(info, "shm_name", LITEGRIP_SHM_DEFAULT_NAME);
  connect_timeout_s_ = param_double(info, "connect_timeout_s", connect_timeout_s_);
  heartbeat_timeout_s_ =
      param_double(info, "heartbeat_timeout_s", heartbeat_timeout_s_);
  state_read_retries_ = param_int(info, "state_read_retries", state_read_retries_);
  configure_read_retries_ =
      param_int(info, "configure_read_retries", configure_read_retries_);
  export_diagnostics_ = param_bool(info, "export_diagnostics", true);

  RCLCPP_INFO(logger_,
              "gripper hardware interface initialized: joint=%s, shm=%s, "
              "calibration closed_rad=%g rad_to_mm=%g, red line [%g, %g] rad, "
              "model opening [%g, %g] m",
              joint_name_.c_str(), shm_name_.c_str(), closed_rad_, rad_to_mm_,
              std::min(red_open_rad_, red_close_rad_),
              std::max(red_open_rad_, red_close_rad_), min_width_, max_width_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // The shared memory segment is created by the **daemon**. Here we wait for it
  // to appear and for "connected + fresh heartbeat": starting in the wrong order
  // (plugin first) is normal, so this waits rather than failing outright.
  const double deadline = monotonic_seconds() + connect_timeout_s_;
  std::string last_error = "the shared memory segment does not exist yet";

  while (monotonic_seconds() < deadline) {
    if (shm_ == nullptr) {
      const int rc = litegrip_shm_open(shm_name_.c_str(), /*create=*/0, &shm_);
      if (rc != LITEGRIP_SHM_OK) {
        last_error = "failed to open the shared memory segment (rc=" +
                     std::to_string(rc) +
                     "): the segment does not exist yet, or the layout version "
                     "does not match (a stale segment has to be rebuilt by the "
                     "daemon)";
        std::this_thread::sleep_for(kReadyPollInterval);
        continue;
      }
    }

    LitegripState probe{};
    const int rc =
        litegrip_shm_read_state(shm_, &probe, configure_read_retries_);
    if (rc != LITEGRIP_SHM_OK) {
      last_error = "failed to read the shared memory state (rc=" +
                   std::to_string(rc) + ")";
      std::this_thread::sleep_for(kReadyPollInterval);
      continue;
    }
    if (probe.connected == 0.0) {
      last_error = "the daemon has not connected to the gripper yet "
                   "(last_error=" +
                   std::to_string(static_cast<int>(probe.last_error)) + ")";
      std::this_thread::sleep_for(kReadyPollInterval);
      continue;
    }
    if (monotonic_seconds() - probe.heartbeat_s > heartbeat_timeout_s_) {
      last_error = "the daemon heartbeat has expired (heartbeat age " +
                   std::to_string(monotonic_seconds() - probe.heartbeat_s) +
                   "s)";
      std::this_thread::sleep_for(kReadyPollInterval);
      continue;
    }

    have_state_ = true;
    state_buffer_ = probe;
    last_heartbeat_s_ = probe.heartbeat_s;
    daemon_alive_ = true;
    apply_state(probe);
    mode_ = Mode::kConfigured;
    RCLCPP_INFO(logger_, "gripper daemon ready: dry_run=%s, enabled=%s",
                probe.dry_run != 0.0 ? "yes" : "no",
                probe.enabled != 0.0 ? "yes" : "no");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (shm_ != nullptr) {
    litegrip_shm_close(shm_);
    shm_ = nullptr;
  }
  RCLCPP_ERROR(logger_,
               "timed out waiting for the gripper daemon (%.1fs): %s\n"
               "  ① is the daemon up: grep litegrip_hw_daemon in litearm's launch\n"
               "  ② do the shm names agree: this plugin's shm_name='%s'\n"
               "  ③ does the segment exist: ls /dev/shm | grep litegrip",
               connect_timeout_s_, last_error.c_str(), shm_name_.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn LitegripSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (shm_ == nullptr) {
    RCLCPP_ERROR(logger_,
                 "shared memory is not open in on_activate (did on_configure "
                 "fail?)");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // ★ Pull the command position to the **measured** position before sending the
  //   first frame: otherwise, at the instant of activation, the daemon would
  //   rush towards the initial value in the command interface (0 = fully
  //   closed), and that is a real mechanical motion.
  latch_command_to_measured();
  mode_ = Mode::kActive;
  publish_command();
  RCLCPP_INFO(logger_,
              "gripper hardware interface activated: command position latched "
              "at the measured opening %.4f m",
              command_position_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // ★ On handing control back, **hold position** rather than disable: disabling
  //   the gripper means it may drop whatever it is holding. The direction
  //   matches the arm (the arm's disable_on_shutdown also defaults to false and
  //   holds position). After that the command frames go stale on their own and
  //   the daemon freezes by its own staleness rule — belt and braces.
  if (shm_ != nullptr) {
    latch_command_to_measured();
    publish_command();
  }
  mode_ = Mode::kStopped;
  RCLCPP_INFO(logger_,
              "gripper hardware interface deactivated (one hold-in-place "
              "command frame was sent)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (shm_ != nullptr) {
    litegrip_shm_close(shm_);
    shm_ = nullptr;
  }
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // The same teardown as on_cleanup; written separately because the semantics
  // differ (shutdown = the process is about to go away). The shared memory is
  // not unlinked here: the segment belongs to the daemon, and either the plugin
  // or the daemon may exit first.
  if (shm_ != nullptr) {
    litegrip_shm_close(shm_);
    shm_ = nullptr;
  }
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitegripSystem::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (shm_ != nullptr) {
    litegrip_shm_close(shm_);
    shm_ = nullptr;
  }
  mode_ = Mode::kUnconfigured;
  RCLCPP_ERROR(logger_,
               "gripper hardware interface entered the error state: "
               "command_manager will stop the controllers");
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
  // ★ position only (decided by the user on 2026-09-23). The trajectory rate
  //   ceiling and the torque budget are daemon parameters, not per-command
  //   fields — see the comments in litegrip_shm.h.
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION,
                          &command_position_);
  return interfaces;
}

// ───────────────────────── read / write ─────────────────────────

void LitegripSystem::apply_state(const LitegripState &state) {
  // Position/velocity: rad → opening in m and m/s.
  // ★ The velocity **must flip sign**: the more negative rad is, the wider the
  //   opening, so d(opening)/dt = −v_rad × rad_to_mm×1e-3. Missing the minus
  //   sign raises no error, it only quietly inverts the "stopped or not"
  //   decision, so it is written out explicitly here.
  state_position_ = rad_to_width(state.position[0]);
  state_velocity_ =
      -state.velocity[0] * rad_to_mm_ * 1e-3;
  // Torque passes the driver reading straight through (no gear-ratio
  // conversion, same as the old driver).
  state_effort_ = state.effort[0];
  state_temperature_mos_ = state.temperature_mos[0];
  state_temperature_coil_ = state.temperature_coil[0];
  state_error_code_ = state.error_code[0];
  state_fault_code_ = state.fault_code[0];
  state_feedback_age_ = state.feedback_age_s[0];

  const double now = monotonic_seconds();
  if (state.fault_code[0] != 0.0 && !reported_fault_ &&
      now - last_fault_log_s_ > kLogThrottleS) {
    reported_fault_ = true;
    last_fault_log_s_ = now;
    RCLCPP_ERROR(logger_,
                 "gripper fault code %d (last_error=%d) — the layer above "
                 "should stop and investigate on the strength of this",
                 static_cast<int>(state.fault_code[0]),
                 static_cast<int>(state.last_error));
  }
  if (state.fault_code[0] == 0.0) {
    reported_fault_ = false;
  }
  if (state.latched != 0.0 && !reported_latched_) {
    reported_latched_ = true;
    RCLCPP_ERROR(logger_,
                 "★ the daemon has **latched** a safe stop (no self-recovery); "
                 "a human must investigate and restart the daemon");
  }
}

hardware_interface::return_type LitegripSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (shm_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }

  LitegripState fresh{};
  const int rc = litegrip_shm_read_state(shm_, &fresh, state_read_retries_);
  if (rc == LITEGRIP_SHM_TORN) {
    // Torn read: keep the previous frame (the state interface's memory was not
    // modified) and skip the update this round. Not counted as a fault — this
    // is the normal cost of a lock-free read, not a hardware error.
    ++torn_state_reads_;
    return hardware_interface::return_type::OK;
  }
  if (rc != LITEGRIP_SHM_OK) {
    RCLCPP_ERROR(logger_, "failed to read the shared memory state: rc=%d", rc);
    return hardware_interface::return_type::ERROR;
  }

  const double now = monotonic_seconds();
  if (fresh.heartbeat_s > last_heartbeat_s_) {
    last_heartbeat_s_ = fresh.heartbeat_s;
    daemon_alive_ = true;
    reported_daemon_loss_ = false;
  } else if (daemon_alive_ &&
             now - last_heartbeat_s_ > heartbeat_timeout_s_) {
    daemon_alive_ = false;
    if (!reported_daemon_loss_ && now - last_stale_log_s_ > kLogThrottleS) {
      reported_daemon_loss_ = true;
      last_stale_log_s_ = now;
      RCLCPP_ERROR(logger_,
                   "gripper daemon heartbeat expired (%.2fs) — the state is "
                   "frozen and whether the hardware is currently controllable "
                   "is **unknown**; the layer above should stop",
                   now - last_heartbeat_s_);
    }
  }

  have_state_ = true;
  state_buffer_ = fresh;
  apply_state(fresh);
  return hardware_interface::return_type::OK;
}

void LitegripSystem::latch_command_to_measured() {
  command_position_ = have_state_ ? state_position_ : 0.0;
}

void LitegripSystem::publish_command() {
  if (shm_ == nullptr) {
    return;
  }
  // A non-finite number is never sent: any comparison involving NaN is false,
  // so it would slip past every range check the daemon has (that is the one
  // input shape it cannot catch).
  if (!std::isfinite(command_position_)) {
    RCLCPP_ERROR(logger_,
                 "command position is not finite (%g) — not publishing this "
                 "frame",
                 command_position_);
    return;
  }

  LitegripCommand command{};
  command.position[0] = width_to_rad(command_position_);
  // Enable: request enable whenever this component is active. The disable path
  // is left to the upper-layer migration round (see the class comment).
  command.enable = (mode_ == Mode::kActive) ? 1.0 : 0.0;
  // Emergency stop: a reserved field, always 0 this round (see the class
  // comment).
  command.estop = 0.0;
  command.stamp_s = monotonic_seconds();
  command.cycle_count = ++command_cycle_;

  const int rc = litegrip_shm_publish_command(shm_, &command);
  if (rc != LITEGRIP_SHM_OK) {
    RCLCPP_ERROR(logger_, "failed to publish the shared memory command: rc=%d",
                 rc);
  }
}

hardware_interface::return_type LitegripSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (shm_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }
  if (mode_ != Mode::kActive) {
    // While not active no command stream is produced: let the daemon hold
    // position by its own "the command is stale" rule.
    return hardware_interface::return_type::OK;
  }
  publish_command();
  return hardware_interface::return_type::OK;
}

}  // namespace litegrip_ros2_control

PLUGINLIB_EXPORT_CLASS(litegrip_ros2_control::LitegripSystem,
                       hardware_interface::SystemInterface)
