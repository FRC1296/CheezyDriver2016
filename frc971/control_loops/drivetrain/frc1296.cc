#include <stdio.h>
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"


namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop *loop1296;
struct DrivetrainConfig config1296;

extern "C" void CheezyInit1296(void)
{
  config1296.shifter_type = ShifterType::SIMPLE_SHIFTER;

  config1296.make_drivetrain_loop = MakeDrivetrainLoop;
  config1296.make_v_drivetrain_loop = MakeVelocityDrivetrainLoop;
  config1296.make_kf_drivetrain_loop = MakeKFDrivetrainLoop;

  config1296.dt = kDt;                        // Control loop time step.
  config1296.stall_torque = kStallTorque;     // Stall torque in N m.
  config1296.stall_current = kStallCurrent;   // Stall current in amps.
  config1296.free_speed_rpm = kFreeSpeedRPM;  // Free speed in rpm.
  config1296.free_current = kFreeCurrent;     // Free current in amps.
  config1296.j = kJ;                       // Robot moment of inertia in kg m^2.
  config1296.mass = kMass;                 // Mass of the robot.
  config1296.robot_radius = kRobotRadius;  // Robot radius, in meters.
  config1296.wheel_radius = kWheelRadius;  // Wheel radius, in meters.
  config1296.r = kR;                       // Motor resistance.
  config1296.v = kV;                       // Motor velocity constant.
  config1296.t = kT;                       // Torque constant.

  config1296.turn_width = kRobotRadius * 2.0;  // Robot turn width, in meters.
  config1296.high_gear_ratio = kGHigh;  // Gear ratios
  config1296.low_gear_ratio = kGLow;

  config1296.open_loop = true;

  loop1296 = new DrivetrainLoop(config1296);
}

extern "C" void CheezyIterate1296(
    const DrivetrainGoal *goal,
    const DrivetrainPosition *position,
    DrivetrainOutput *output,
    DrivetrainStatus *status)
{
  loop1296->RunIteration(goal, position, output, status);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
