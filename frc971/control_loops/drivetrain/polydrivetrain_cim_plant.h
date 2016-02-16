#ifndef FRC971_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_
#define FRC971_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<1, 1, 1> MakeCIMPlantCoefficients();

StateFeedbackController<1, 1, 1> MakeCIMController();

StateFeedbackPlant<1, 1, 1> MakeCIMPlant();

StateFeedbackLoop<1, 1, 1> MakeCIMLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_
