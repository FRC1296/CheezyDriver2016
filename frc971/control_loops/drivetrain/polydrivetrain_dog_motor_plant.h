#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2014 {
namespace control_loops {
namespace drivetrain {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController();

StateFeedbackPlant<2, 2, 2> MakeVelocityDrivetrainPlant();

StateFeedbackLoop<2, 2, 2> MakeVelocityDrivetrainLoop();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
