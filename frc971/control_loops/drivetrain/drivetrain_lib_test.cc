#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/network/team_number.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop_test.h"

#include "y2014/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "frc971/queues/gyro.q.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using ::y2014::control_loops::drivetrain::MakeDrivetrainPlant;

// TODO(Comran): Make one that doesn't depend on the actual values for a
// specific robot.
const DrivetrainConfig kDrivetrainConfig {
    ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,

    ::y2014::control_loops::drivetrain::MakeDrivetrainLoop,
    ::y2014::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
    ::y2014::control_loops::drivetrain::MakeKFDrivetrainLoop,

    ::y2014::control_loops::drivetrain::kDt,
    ::y2014::control_loops::drivetrain::kStallTorque,
    ::y2014::control_loops::drivetrain::kStallCurrent,
    ::y2014::control_loops::drivetrain::kFreeSpeedRPM,
    ::y2014::control_loops::drivetrain::kFreeCurrent,
    ::y2014::control_loops::drivetrain::kJ,
    ::y2014::control_loops::drivetrain::kMass,
    ::y2014::control_loops::drivetrain::kRobotRadius,
    ::y2014::control_loops::drivetrain::kWheelRadius,
    ::y2014::control_loops::drivetrain::kR,
    ::y2014::control_loops::drivetrain::kV,
    ::y2014::control_loops::drivetrain::kT,

    ::y2014::constants::GetValuesForTeam(971).turn_width,
    ::y2014::constants::GetValuesForTeam(971).high_gear_ratio,
    ::y2014::constants::GetValuesForTeam(971).low_gear_ratio,
    ::y2014::constants::GetValuesForTeam(971).left_drive,
    ::y2014::constants::GetValuesForTeam(971).right_drive
};

class Environment : public ::testing::Environment {
 public:
  virtual ~Environment() {}
  // how to set up the environment.
  virtual void SetUp() {
    aos::controls::HPolytope<0>::Init();
  }
};
::testing::Environment* const holder_env =
  ::testing::AddGlobalTestEnvironment(new Environment);

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(971); }
};

::testing::Environment* const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment);

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation()
      : drivetrain_plant_(
            new StateFeedbackPlant<4, 2, 2>(MakeDrivetrainPlant())),
        my_drivetrain_queue_(".frc971.control_loops.drivetrain",
                       0x8a8dde77, ".frc971.control_loops.drivetrain.goal",
                       ".frc971.control_loops.drivetrain.position",
                       ".frc971.control_loops.drivetrain.output",
                       ".frc971.control_loops.drivetrain.status") {
    Reinitialize();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->mutable_X(0, 0) = 0.0;
    drivetrain_plant_->mutable_X(1, 0) = 0.0;
    drivetrain_plant_->mutable_Y() =
        drivetrain_plant_->C() * drivetrain_plant_->X();
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_->Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_->Y(1, 0); }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Position>
        position = my_drivetrain_queue_.position.MakeMessage();
    position->left_encoder = left_encoder;
    position->right_encoder = right_encoder;
    position.Send();
  }

  // Simulates the drivetrain moving for one timestep.
  void Simulate() {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    EXPECT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    drivetrain_plant_->mutable_U() << my_drivetrain_queue_.output->left_voltage,
        my_drivetrain_queue_.output->right_voltage;
    drivetrain_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> drivetrain_plant_;
 private:
  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;
  double last_left_position_;
  double last_right_position_;
};

class DrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest() : my_drivetrain_queue_(".frc971.control_loops.drivetrain",
                               0x8a8dde77,
                               ".frc971.control_loops.drivetrain.goal",
                               ".frc971.control_loops.drivetrain.position",
                               ".frc971.control_loops.drivetrain.output",
                               ".frc971.control_loops.drivetrain.status"),
                drivetrain_motor_(kDrivetrainConfig, &my_drivetrain_queue_),
                drivetrain_motor_plant_() {
    ::frc971::sensors::gyro_reading.Clear();
  }

  void VerifyNearGoal() {
    my_drivetrain_queue_.goal.FetchLatest();
    my_drivetrain_queue_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_queue_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(),
                1e-2);
    EXPECT_NEAR(my_drivetrain_queue_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(),
                1e-2);
  }

  virtual ~DrivetrainTest() {
    ::frc971::sensors::gyro_reading.Clear();
  }
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  my_drivetrain_queue_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 200; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  my_drivetrain_queue_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    if (i > 20 && i < 200) {
      SimulateTimestep(false);
    } else {
      SimulateTimestep(true);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(DrivetrainTest, DriveStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(4.0)
      .right_goal(4.0)
      .Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_FLOAT_EQ(my_drivetrain_queue_.output->left_voltage,
                    my_drivetrain_queue_.output->right_voltage);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -3);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -3);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(4.0)
      .right_goal(3.9)
      .Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -3);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -3);
  }
  VerifyNearGoal();
}

::aos::controls::HPolytope<2> MakeBox(double x1_min, double x1_max,
                                      double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
            /*[*/-1.0, 0.0 /*]*/,
            /*[*/ 0.0, 1.0 /*]*/,
            /*[*/ 0.0,-1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
            /*[*/-x1_min /*]*/,
            /*[*/ x2_max /*]*/,
            /*[*/-x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return t_poly;
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_Intersect) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_no_Intersect) {
  ::aos::controls::HPolytope<2> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Middle_Of_Edge) {
  ::aos::controls::HPolytope<2> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
