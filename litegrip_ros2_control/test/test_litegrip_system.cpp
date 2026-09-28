// test_litegrip_system.cpp — the LiteGrip hardware component, driven directly.
//
// The component is instantiated and stepped through its lifecycle in-process
// (not through pluginlib, which would need a controller_manager), with a
// synthetic HardwareInfo. Every case runs with dry_run=true, so no CAN socket is
// opened and no motor is touched — which is exactly what makes parameter
// parsing, the unit conversion and the read/write path testable here.

#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "litegrip_ros2_control/litegrip_system.hpp"

#ifndef LITEGRIP_TEST_BASELINE
#error "LITEGRIP_TEST_BASELINE must be defined by CMake"
#endif

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::HardwareInfo;
using litegrip_ros2_control::LitegripSystem;

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/// A synthetic single-joint <ros2_control> block in dry-run mode.
///
/// The calibration is chosen to be CONSISTENT with the shipped red lines
/// ([-1.24, -0.01] rad): the closed end sits at red_max. A deployment whose
/// calibration put the closed end outside the red lines would — correctly — be
/// refused all motion.
HardwareInfo make_info() {
  HardwareInfo info;
  info.name = "LitegripSystem";
  info.type = "system";

  hardware_interface::ComponentInfo joint;
  joint.name = "gripper_opening_joint";
  info.joints.push_back(joint);

  info.hardware_parameters["dry_run"] = "true";
  info.hardware_parameters["hardware_enable"] = "false";
  info.hardware_parameters["closed_rad"] = "-0.01";
  info.hardware_parameters["rad_to_mm"] = "100.0";
  info.hardware_parameters["min_width"] = "0.0";
  info.hardware_parameters["max_width"] = "1.23";
  info.hardware_parameters["max_feedback_velocity_rad_s"] = "1.0";
  info.hardware_parameters["max_velocity_rad_s"] = "0.4";
  info.hardware_parameters["control_rate_hz"] = "200.0";
  info.hardware_parameters["command_timeout_s"] = "10.0";
  info.hardware_parameters["safety_baseline"] = LITEGRIP_TEST_BASELINE;
  return info;
}

/// Bring a component up in dry-run mode.
std::unique_ptr<LitegripSystem> make_configured(const HardwareInfo &info) {
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  return system;
}

}  // namespace

// ── parameter handling ────────────────────────────────────────────────────

TEST(LitegripSystemTest, InitAcceptsAValidSingleJointInfo) {
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(make_info()), CallbackReturn::SUCCESS);
}

TEST(LitegripSystemTest, InitRejectsMoreThanOneJoint) {
  HardwareInfo info = make_info();
  info.joints.push_back(info.joints.front());
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, InitRejectsAZeroRadToMm) {
  HardwareInfo info = make_info();
  info.hardware_parameters["rad_to_mm"] = "0";
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, InitRejectsInvertedWidthLimits) {
  HardwareInfo info = make_info();
  info.hardware_parameters["min_width"] = "0.5";
  info.hardware_parameters["max_width"] = "0.1";
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, InitRejectsANonFiniteParameter) {
  HardwareInfo info = make_info();
  info.hardware_parameters["closed_rad"] = "nan";
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, InitRejectsANonNumericParameter) {
  HardwareInfo info = make_info();
  info.hardware_parameters["kp"] = "not-a-number";
  auto system = std::make_unique<LitegripSystem>();
  EXPECT_EQ(system->on_init(info), CallbackReturn::ERROR);
}

// ── interface surface (frozen 2026-09-23) ─────────────────────────────────

TEST(LitegripSystemTest, ExportsPositionCommandOnly) {
  auto system = make_configured(make_info());
  const auto commands = system->export_command_interfaces();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().get_name(), "gripper_opening_joint/position");
}

TEST(LitegripSystemTest, ExportsTheStandardTrioPlusDiagnostics) {
  auto system = make_configured(make_info());
  const auto states = system->export_state_interfaces();
  ASSERT_EQ(states.size(), 8u);
  EXPECT_EQ(states[0].get_name(), "gripper_opening_joint/position");
  EXPECT_EQ(states[1].get_name(), "gripper_opening_joint/velocity");
  EXPECT_EQ(states[2].get_name(), "gripper_opening_joint/effort");
  EXPECT_EQ(states[3].get_name(), "gripper_opening_joint/temperature_mos");
  EXPECT_EQ(states[4].get_name(), "gripper_opening_joint/temperature_coil");
  EXPECT_EQ(states[5].get_name(), "gripper_opening_joint/error_code");
  EXPECT_EQ(states[6].get_name(), "gripper_opening_joint/fault_code");
  EXPECT_EQ(states[7].get_name(), "gripper_opening_joint/feedback_age");
}

TEST(LitegripSystemTest, DiagnosticsCanBeSuppressed) {
  HardwareInfo info = make_info();
  info.hardware_parameters["export_diagnostics"] = "false";
  auto system = make_configured(info);
  EXPECT_EQ(system->export_state_interfaces().size(), 3u);
}

// ── lifecycle ─────────────────────────────────────────────────────────────

TEST(LitegripSystemTest, ConfigureFailsWhenTheDualSwitchIsIncomplete) {
  HardwareInfo info = make_info();
  info.hardware_parameters["dry_run"] = "false";
  info.hardware_parameters["hardware_enable"] = "false";
  auto system = std::make_unique<LitegripSystem>();
  ASSERT_EQ(system->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, ConfigureFailsOnAnUnknownSafetyBaseline) {
  HardwareInfo info = make_info();
  info.hardware_parameters["safety_baseline"] = "9.9";
  auto system = std::make_unique<LitegripSystem>();
  ASSERT_EQ(system->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, ConfigureFailsOnAMissingBaselineFile) {
  HardwareInfo info = make_info();
  info.hardware_parameters["safety_baseline"] = "/tmp/no/such/baseline.json";
  auto system = std::make_unique<LitegripSystem>();
  ASSERT_EQ(system->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::ERROR);
}

TEST(LitegripSystemTest, ActivateDeactivateCleanupRoundTrip) {
  auto system = make_configured(make_info());
  EXPECT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitegripSystemTest, ReadWriteFailBeforeConfigure) {
  auto system = std::make_unique<LitegripSystem>();
  ASSERT_EQ(system->on_init(make_info()), CallbackReturn::SUCCESS);
  // No loop yet: both must report ERROR rather than pretend to work.
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
}

// ── the read path ─────────────────────────────────────────────────────────

TEST(LitegripSystemTest, ReadsTheClosedEndAsZeroOpening) {
  auto system = make_configured(make_info());
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  sleep_ms(100);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);

  const double position = system->export_state_interfaces()[0].get_value();
  // dry_run starts the simulated plant at the calibrated closed end, which is
  // 0 mm by construction — but "the closed end" is a motor angle that the 16-bit
  // MIT position field cannot represent exactly. The safety gate quantizes the
  // target into the red lines, and the decoded value lands about one LSB away
  // (1 LSB = 25/65535 rad, ~3.8e-4 rad, ~3.8e-2 mm at 100 mm/rad). So the
  // tolerance here is the quantization floor, not slack.
  EXPECT_NEAR(position, 0.0, 1e-4) << "expected the closed end within one MIT "
                                      "position LSB";
}

// ── the write path and the conversion ─────────────────────────────────────

TEST(LitegripSystemTest, WriteConvertsMetresToRadiansAndTheLoopFollows) {
  auto system = make_configured(make_info());
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // Push a target through the command interface, exactly as a controller would.
  system->export_command_interfaces()[0].set_value(0.02);  // 20 mm
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);

  // 20 mm at 100 mm/rad is 0.2 rad, and the rate ceiling is 0.4 rad/s, so it
  // needs ~0.5 s; give it margin.
  sleep_ms(900);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  const double position = system->export_state_interfaces()[0].get_value();
  EXPECT_NEAR(position, 0.02, 0.001);
}

TEST(LitegripSystemTest, WriteBeyondTheRangeIsClampedNotRejected) {
  // This is what the clamping in width_to_rad() exists for. The model-layer
  // range is wider than the red lines permit, and the SDK REJECTS an
  // out-of-range target rather than clamping it — so without clamping here an
  // ordinary "open fully" command would be dropped whole, the gripper would not
  // move, and all that would be left behind is a fault code. The observable
  // proof that clamping happened is the ABSENCE of a fault.
  auto system = make_configured(make_info());
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  system->export_command_interfaces()[0].set_value(1.0);  // far beyond the range
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  sleep_ms(300);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);

  const auto states = system->export_state_interfaces();
  const double fault_code = states[6].get_value();
  EXPECT_EQ(fault_code, 0.0) << "an out-of-range command was rejected instead "
                                "of being clamped into the commandable range";
  const double age = states[7].get_value();
  EXPECT_TRUE(std::isfinite(age) || age < 0.0);
}

// ── activation must hold, not drive to the interface's default ────────────

TEST(LitegripSystemTest, ActivateIgnoresWhateverTheCommandInterfaceInitiallyHeld) {
  // ros2_control initialises command interfaces to 0.0, and 0.0 opening is a
  // fully closed gripper. Treating that as a goal makes the gripper travel to
  // the closed end the moment the stack comes up, from wherever it actually was.
  //
  // The requirement is that the ros2_control layer's target EQUALS the current
  // position, so the initial value is irrelevant. This checks that literally:
  // whatever is in the interface before activation gets overwritten.
  for (const double preloaded : {0.0, 0.087, -1.0, 42.0}) {
    auto system = make_configured(make_info());
    system->export_command_interfaces()[0].set_value(preloaded);
    const double measured_before =
        system->export_state_interfaces()[0].get_value();

    ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()),
              CallbackReturn::SUCCESS)
        << "preloaded command value " << preloaded;

    const double latched = system->export_command_interfaces()[0].get_value();
    EXPECT_NEAR(latched, measured_before, 1e-6)
        << "the command interface's initial value " << preloaded
        << " was treated as a goal instead of being latched to the measured "
        << "opening " << measured_before;
  }
}

TEST(LitegripSystemTest, ActivateHoldsInsteadOfDrivingTowardZero) {
  // The behavioural consequence of the above: after activation, with no goal
  // sent, the gripper must stay where it is.
  auto system = make_configured(make_info());
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  const double at_activation = system->export_state_interfaces()[0].get_value();

  sleep_ms(600);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  const double after = system->export_state_interfaces()[0].get_value();

  EXPECT_NEAR(after, at_activation, 0.001)
      << "the gripper moved after activation; with no goal sent it must hold "
         "position";
}
