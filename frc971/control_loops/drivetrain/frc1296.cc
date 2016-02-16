#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop *loop1296;
struct DrivetrainConfig config1296;

extern "C" void CheezyInit1296(void)
{
  config1296.shifter_type = ShifterType::SIMPLE_SHIFTER;

  config1296.dt = 0.02;  // Control loop time step.
  config1296.stall_torque = 0.71;  // Stall torque in N m.
  config1296.stall_current = 134.0;  // Stall current in amps.
  config1296.free_speed_rpm = 18100.0;  // Free speed in rpm.
  config1296.free_current = 0.7;  // Free current in amps.
  config1296.j = 2.8;  // CIM moment of inertia in kg m^2.
  config1296.mass = 22.79;  // Mass of the robot.
  config1296.robot_radius = 0.558;  // Robot radius, in meters.
  config1296.wheel_radius = 0.152;  // Wheel radius, in meters.
  config1296.r = (12.0 / 134.0 / 2);  // Motor resistance.
  config1296.v = ((config1296.free_speed_rpm / 60.0 * 2.0 * 3.141519) / (12.0 - config1296.free_current * config1296.r));  // Motor velocity constant.
  config1296.t = (config1296.stall_torque / config1296.stall_current);  // Torque constant.

  config1296.turn_width = (0.558 * 4.0);  // Robot turn width, in meters.
  config1296.high_gear_ratio = 1.0;  // Gear ratios, from encoder shaft to transmission output.
  config1296.low_gear_ratio = 1.0;

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
