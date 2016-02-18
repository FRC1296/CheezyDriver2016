#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include <functional>

#include "frc971/shifter_hall_effect.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

enum class ShifterType : int32_t {
  HALL_EFFECT_SHIFTER = 0,  // Detect when inbetween gears.
  SIMPLE_SHIFTER = 1,  // Switch gears without speedmatch logic.
};

struct DrivetrainConfig {
  // Shifting method we are using.
  ShifterType shifter_type;

  // Polydrivetrain functions returning various controller loops with plants.
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;
  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<7, 2, 3>()> make_kf_drivetrain_loop;

  double dt;  // Control loop time step in seconds
  double stall_torque;  // Stall torque in Newton meters.
  double stall_current;  // Stall current in amps.
  double free_speed_rpm;  // Free speed in rpm.
  double free_current;  // Free current in amps.
  double j;  // CIM moment of inertia in kg m^2.
  double mass;  // Mass of the robot.
  double robot_radius;  // Robot radius, in meters.
  double wheel_radius;  // Wheel radius, in meters.
  double r;  // Motor resistance.
  double v;  // Motor velocity constant.
  double t;  // Torque constant.

  double turn_width;  // Robot turn width, in meters.
  // Gear ratios, from encoder shaft to transmission output.
  double high_gear_ratio;
  double low_gear_ratio;

  // Hall effect constants. Unused if not applicable to shifter type.
  constants::ShifterHallEffect left_drive;
  constants::ShifterHallEffect right_drive;
  bool open_loop = false;
};


// Structs to carry information about the drivetrain.
struct DrivetrainGoal {
  // The steering wheel value.  Should be between -1 and 1.
  double steering;
  // The throttle stickvalue.  Should be between -1 and 1.
  double throttle;
  // If true, the robot should shift into high gear.
  bool highgear;
  // If true, the robot should enable quick-turn mode.  This will revert to an
  // arcade-like steering mode.
  bool quickturn;
  // If true, use the statespace closed loop position controller with the
  // goals below.
  bool control_loop_driving;
  // The left distance goal in meters from startup.
  double left_goal;
  // The right velocity goal in meters/second
  double left_velocity_goal;
  // The right distance goal in meters from startup.
  double right_goal;
  // The right velocity goal in meters/second
  double right_velocity_goal;
};

struct DrivetrainPosition {
  // The current left and right encoder positions in meters since startup.
  // Positive is forwards.
  double left_encoder;
  double right_encoder;
  // The gyro angle and velocity in radians and radians/second.  Positive is
  // according to the right hand rule around Z = straight up.
  double gyro_angle;
  double gyro_rate;
  // Battery voltage in Volts.
  double battery_voltage;
  // The measured shifter position of the left and right shifters.  False if it
  // is in low gear, true if it is in high gear.
  bool left_shifter_position;
  bool right_shifter_position;
};

struct DrivetrainOutput {
  // Voltage to send to the left and right motors in volts.
  double left_voltage;
  double right_voltage;
  // Solenoid positions to command for the two shifters.  true means shift into
  // high gear.
  bool left_high;
  bool right_high;
};

struct DrivetrainStatus {
  // Current robot speed in meters/second
  double robot_speed;
  // Current position of the left and right sides of the drivetrain in
  // meters
  double filtered_left_position;
  double filtered_right_position;
  // Current speed of the left and right sides of the drivetrain in
  // meters/second
  double filtered_left_velocity;
  double filtered_right_velocity;

  // The control loop request for the drivetrain position controller before +-
  // 12v battery limit is taken into effect.
  double uncapped_left_voltage;
  double uncapped_right_voltage;
  // True if the output was modified.
  bool output_was_capped;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
