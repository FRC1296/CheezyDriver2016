#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#ifdef INCLUDE_971_INFRASTRUCTURE
#include "aos/common/logging/queue_logging.h"
#endif //INCLUDE_971_INFRASTRUCTURE
#include "aos/common/logging/matrix_logging.h"

#ifdef INCLUDE_971_INFRASTRUCTURE
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#endif //INCLUDE_971_INFRASTRUCTURE
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#ifdef INCLUDE_971_INFRASTRUCTURE
#include "frc971/queues/gyro.q.h"
#include "frc971/shifter_hall_effect.h"

using frc971::sensors::gyro_reading;
#endif //INCLUDE_971_INFRASTRUCTURE

namespace frc971 {
namespace control_loops {
namespace drivetrain {

#ifdef INCLUDE_971_INFRASTRUCTURE
DrivetrainLoop::DrivetrainLoop(
    const DrivetrainConfig &dt_config,
    ::frc971::control_loops::DrivetrainQueue *my_drivetrain)
    : aos::controls::ControlLoop<::frc971::control_loops::DrivetrainQueue>(
          my_drivetrain),
      : dt_config_(dt_config),
        kf_(dt_config_.make_kf_drivetrain_loop()),
        dt_openloop_(dt_config_, &kf_),
        dt_closedloop_(dt_config_) {
  ::aos::controls::HPolytope<0>::Init();
}
#else //INCLUDE_971_INFRASTRUCTURE
DrivetrainLoop::DrivetrainLoop(const DrivetrainConfig &dt_config)
    : dt_config_(dt_config),
      kf_(dt_config_.make_kf_drivetrain_loop()),
      dt_openloop_(dt_config_, &kf_),
      dt_closedloop_(dt_config_) {
  ::aos::controls::HPolytope<0>::Init();
}
#endif //INCLUDE_971_INFRASTRUCTURE

#ifdef INCLUDE_971_INFRASTRUCTURE
void DrivetrainLoop::RunIteration(
    const ::frc971::control_loops::DrivetrainQueue::Goal *goal,
    const ::frc971::control_loops::DrivetrainQueue::Position *position,
    ::frc971::control_loops::DrivetrainQueue::Output *output,
    ::frc971::control_loops::DrivetrainQueue::Status *status) {
#else //INCLUDE_971_INFRASTRUCTURE
void DrivetrainLoop::RunIteration(
    const DrivetrainGoal *goal,
    const DrivetrainPosition *position,
    DrivetrainOutput *output,
    DrivetrainStatus *status) {
#endif //INCLUDE_971_INFRASTRUCTURE

  bool bad_pos = false;
  if (position == nullptr) {
#ifdef INCLUDE_971_INFRASTRUCTURE
    LOG_INTERVAL(no_position_);
    bad_pos = true;
#endif //INCLUDE_971_INFRASTRUCTURE
  }
#ifdef INCLUDE_971_INFRASTRUCTURE
  no_position_.Print();
#endif //INCLUDE_971_INFRASTRUCTURE

  kf_.set_controller_index(dt_openloop_.controller_index());

  {
    Eigen::Matrix<double, 3, 1> Y;
    Y << position->left_encoder, position->right_encoder, position->gyro_rate;
    kf_.Correct(Y);
    integrated_kf_heading_ += dt_config_.dt *
                              (kf_.X_hat(3, 0) - kf_.X_hat(1, 0)) /
                              (dt_config_.robot_radius * 2.0);
  }

  bool control_loop_driving = false;
  if (goal) {
    double wheel = goal->steering;
    double throttle = goal->throttle;
    bool quickturn = goal->quickturn;
    bool highgear = goal->highgear;

    control_loop_driving = goal->control_loop_driving;
    double left_goal = goal->left_goal;
    double right_goal = goal->right_goal;

    dt_closedloop_.SetGoal(left_goal, goal->left_velocity_goal, right_goal,
                           goal->right_velocity_goal);
    dt_openloop_.SetGoal(wheel, throttle, quickturn, highgear);
  }

  if (!bad_pos) {
    const double left_encoder = position->left_encoder;
    const double right_encoder = position->right_encoder;
#ifdef INCLUDE_971_INFRASTRUCTURE
    if (gyro_reading.FetchLatest()) {
      LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
      dt_closedloop_.SetPosition(left_encoder, right_encoder,
                                 gyro_reading->angle);
      last_gyro_heading_ = gyro_reading->angle;
      last_gyro_rate_ = gyro_reading->velocity;
    } else {
      dt_closedloop_.SetRawPosition(left_encoder, right_encoder);
    }
#else //INCLUDE_971_INFRASTRUCTURE
    //if () {
      dt_closedloop_.SetPosition(left_encoder, right_encoder,
                                 position->gyro_angle);
    //} else {
    //  dt_closedloop_.SetRawPosition(left_encoder, right_encoder);
    //}  TKB what to do if gyro reading is bad?
#endif //INCLUDE_971_INFRASTRUCTURE
  }
  dt_openloop_.SetPosition(position);
  dt_openloop_.Update();

  if (control_loop_driving) {
    dt_closedloop_.Update(output == NULL, true);
    dt_closedloop_.SendMotors(output);
  } else {
    dt_openloop_.SendMotors(output);
    if (output) {
      dt_closedloop_.SetExternalMotors(output->left_voltage,
                                       output->right_voltage);
    }
    dt_closedloop_.Update(output == NULL, false);
  }

  // set the output status of the control loop state
  if (status) {
    status->robot_speed = dt_closedloop_.GetEstimatedRobotSpeed();
    status->filtered_left_position = dt_closedloop_.GetEstimatedLeftEncoder();
    status->filtered_right_position = dt_closedloop_.GetEstimatedRightEncoder();

    status->filtered_left_velocity = dt_closedloop_.loop().X_hat(1, 0);
    status->filtered_right_velocity = dt_closedloop_.loop().X_hat(3, 0);
    status->output_was_capped = dt_closedloop_.OutputWasCapped();
    status->uncapped_left_voltage = dt_closedloop_.loop().U_uncapped(0, 0);
    status->uncapped_right_voltage = dt_closedloop_.loop().U_uncapped(1, 0);
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = output->left_voltage;
    right_voltage = output->right_voltage;
  }

#ifdef INCLUDE_971_INFRASTRUCTURE
  const double scalar = ::aos::robot_state->voltage_battery / 12.0;
#else //INCLUDE_971_INFRASTRUCTURE
  const double scalar = position->battery_voltage / 12.0; 
#endif //INCLUDE_971_INFRASTRUCTURE

  left_voltage *= scalar;
  right_voltage *= scalar;

  // To validate, look at the following:

  // Observed - dx/dt velocity for left, right.

  // Angular velocity error compared to the gyro
  // Gyro heading vs left-right
  // Voltage error.

  Eigen::Matrix<double, 2, 1> U;
  U << last_left_voltage_, last_right_voltage_;
  last_left_voltage_ = left_voltage;
  last_right_voltage_ = right_voltage;

  kf_.UpdateObserver(U);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
