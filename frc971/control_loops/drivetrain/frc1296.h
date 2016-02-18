#ifndef FRC1296_H_
#define FRC1296_H_

// Structs to carry information about the drivetrain.
struct DrivetrainGoal {
    double steering;
    double throttle;
    bool highgear;
    bool quickturn;
    bool control_loop_driving;
    double left_goal;
    double left_velocity_goal;
    double right_goal;
    double right_velocity_goal;
  };

struct DrivetrainPosition {
    double left_encoder;
    double right_encoder;
    double gyro_angle;
    double gyro_velocity;
    double battery_voltage;
    bool left_shifter_position;
    bool right_shifter_position; 
  };

struct DrivetrainOutput {
    double left_voltage;
    double right_voltage;
    bool left_high; 
    bool right_high;
  };

struct DrivetrainStatus {
    double robot_speed;
    double filtered_left_position;
    double filtered_right_position;
    double filtered_left_velocity;
    double filtered_right_velocity;

    double uncapped_left_voltage;
    double uncapped_right_voltage;
    bool output_was_capped;
  };

extern "C" void CheezyInit1296(void);

extern "C" void CheezyIterate1296(
    const DrivetrainGoal *goal,
    const DrivetrainPosition *position,
    DrivetrainOutput *output,
    DrivetrainStatus *status);

#endif  // FRC1296_H_
