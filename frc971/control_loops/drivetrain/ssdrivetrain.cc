#include "frc971/control_loops/drivetrain/ssdrivetrain.h"

#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/matrix_logging.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#ifdef INCLUDE_971_INFRASTRUCTURE
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#endif //INCLUDE_971_INFRASTRUCTURE
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

using ::frc971::control_loops::DoCoerceGoal;

DrivetrainMotorsSS::LimitedDrivetrainLoop::LimitedDrivetrainLoop(
    StateFeedbackLoop<4, 2, 2> &&loop)
    : StateFeedbackLoop<4, 2, 2>(::std::move(loop)),
      U_poly_((Eigen::Matrix<double, 4, 2>() << 1, 0, -1, 0, 0, 1, 0, -1)
                  .finished(),
              (Eigen::Matrix<double, 4, 1>() << 12.0, 12.0, 12.0, 12.0)
                  .finished()) {
  ::aos::controls::HPolytope<0>::Init();
  T_ << 1, 1, 1, -1;
  T_inverse_ = T_.inverse();
}

// This intentionally runs the U-capping code even when it's unnecessary to help
// make it more deterministic. Only running it when one or both sides want
// out-of-range voltages could lead to things like running out of CPU under
// certain situations, which would be bad.
void DrivetrainMotorsSS::LimitedDrivetrainLoop::CapU() {
  output_was_capped_ = ::std::abs(U(0, 0)) > 12.0 || ::std::abs(U(1, 0)) > 12.0;

  const Eigen::Matrix<double, 4, 1> error = R() - X_hat();

  LOG_MATRIX(DEBUG, "U at start", U());
  LOG_MATRIX(DEBUG, "R at start", R());
  LOG_MATRIX(DEBUG, "Xhat at start", X_hat());

  Eigen::Matrix<double, 2, 2> position_K;
  position_K << K(0, 0), K(0, 2), K(1, 0), K(1, 2);
  Eigen::Matrix<double, 2, 2> velocity_K;
  velocity_K << K(0, 1), K(0, 3), K(1, 1), K(1, 3);

  Eigen::Matrix<double, 2, 1> position_error;
  position_error << error(0, 0), error(2, 0);
  // drive_error = [total_distance_error, left_error - right_error]
  const auto drive_error = T_inverse_ * position_error;
  Eigen::Matrix<double, 2, 1> velocity_error;
  velocity_error << error(1, 0), error(3, 0);
  LOG_MATRIX(DEBUG, "error", error);

  const ::aos::controls::HPolytope<2> pos_poly(U_poly_, position_K * T_,
                                               -velocity_K * velocity_error);

  Eigen::Matrix<double, 2, 1> adjusted_pos_error;
  {
    const auto &P = drive_error;

    Eigen::Matrix<double, 1, 2> L45;
    L45 << ::aos::sign(P(1, 0)), -::aos::sign(P(0, 0));
    const double w45 = 0;

    Eigen::Matrix<double, 1, 2> LH;
    if (::std::abs(P(0, 0)) > ::std::abs(P(1, 0))) {
      LH << 0, 1;
    } else {
      LH << 1, 0;
    }
    const double wh = LH.dot(P);

    Eigen::Matrix<double, 2, 2> standard;
    standard << L45, LH;
    Eigen::Matrix<double, 2, 1> W;
    W << w45, wh;
    const Eigen::Matrix<double, 2, 1> intersection = standard.inverse() * W;

    bool is_inside_h, is_inside_45;
    const auto adjusted_pos_error_h =
        DoCoerceGoal(pos_poly, LH, wh, drive_error, &is_inside_h);
    const auto adjusted_pos_error_45 =
        DoCoerceGoal(pos_poly, L45, w45, intersection, &is_inside_45);
    if (pos_poly.IsInside(intersection)) {
      adjusted_pos_error = adjusted_pos_error_h;
    } else {
      if (is_inside_h) {
        if (adjusted_pos_error_h.norm() > adjusted_pos_error_45.norm() ||
            adjusted_pos_error_45.norm() > intersection.norm()) {
          adjusted_pos_error = adjusted_pos_error_h;
        } else {
          adjusted_pos_error = adjusted_pos_error_45;
        }
      } else {
        adjusted_pos_error = adjusted_pos_error_45;
      }
    }
  }

  mutable_U() =
      velocity_K * velocity_error + position_K * T_ * adjusted_pos_error;
  LOG_MATRIX(DEBUG, "U is now", U());

  if (!output_was_capped_) {
    if ((U() - U_uncapped()).norm() > 0.0001) {
      LOG(FATAL, "U unnecessarily capped\n");
    }
  }
}

DrivetrainMotorsSS::DrivetrainMotorsSS(const DrivetrainConfig &dt_config)
    : loop_(
          new LimitedDrivetrainLoop(dt_config.make_drivetrain_loop())),
      filtered_offset_(0.0),
      gyro_(0.0),
      left_goal_(0.0),
      right_goal_(0.0),
      raw_left_(0.0),
      raw_right_(0.0),
      dt_config_(dt_config) {
  // High gear on both.
  loop_->set_controller_index(3);
}

void DrivetrainMotorsSS::SetGoal(double left, double left_velocity,
                                 double right, double right_velocity) {
  left_goal_ = left;
  right_goal_ = right;
  loop_->mutable_R() << left, left_velocity, right, right_velocity;
}
void DrivetrainMotorsSS::SetRawPosition(double left, double right) {
  raw_right_ = right;
  raw_left_ = left;
  Eigen::Matrix<double, 2, 1> Y;
  Y << left + filtered_offset_, right - filtered_offset_;
  loop_->Correct(Y);
}
void DrivetrainMotorsSS::SetPosition(double left, double right, double gyro) {
  // Decay the offset quickly because this gyro is great.
  const double offset =
      (right - left - gyro * dt_config_.turn_width) / 2.0;
  filtered_offset_ = 0.25 * offset + 0.75 * filtered_offset_;
  gyro_ = gyro;
  SetRawPosition(left, right);
}

void DrivetrainMotorsSS::SetExternalMotors(double left_voltage,
                                           double right_voltage) {
  loop_->mutable_U() << left_voltage, right_voltage;
}

void DrivetrainMotorsSS::Update(bool stop_motors, bool enable_control_loop) {
  if (enable_control_loop) {
    loop_->Update(stop_motors);
  } else {
    if (stop_motors) {
      loop_->mutable_U().setZero();
      loop_->mutable_U_uncapped().setZero();
    }
    loop_->UpdateObserver(loop_->U());
  }
#ifdef INCLUDE_971_INFRASTRUCTURE
  ::Eigen::Matrix<double, 4, 1> E = loop_->R() - loop_->X_hat();
  LOG_MATRIX(DEBUG, "E", E);
#endif //INCLUDE_971_INFRASTRUCTURE
}

double DrivetrainMotorsSS::GetEstimatedRobotSpeed() const {
  // lets just call the average of left and right velocities close enough
  return (loop_->X_hat(1, 0) + loop_->X_hat(3, 0)) / 2;
}

#ifdef INCLUDE_971_INFRASTRUCTURE
void DrivetrainMotorsSS::SendMotors(
    ::frc971::control_loops::DrivetrainQueue::Output *output) const {
#else  //INCLUDE_971_INFRASTRUCTURE
void DrivetrainMotorsSS::SendMotors(DrivetrainOutput *output) {
#endif //INCLUDE_971_INFRASTRUCTURE
  if (output) {
    output->left_voltage = loop_->U(0, 0);
    output->right_voltage = loop_->U(1, 0);
    output->left_high = true;
    output->right_high = true;
  }
}


}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
