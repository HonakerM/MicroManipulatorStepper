// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "utilities/logging.h"
#include "peripheral_control/peripheral_control.h"
#include "utilities/frequency_counter.h"
#include "hardware/MT6701_encoder.h"
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "servo_control/servo_controller.h"
#include "utilities/lookup_table.h"
#include "utilities/math_constants.h"

#include "motion_control/path_planner.h"
#include "motion_control/motion_controller.h"
#include "command_parser/command_parser.h"

constexpr int ENCODER_LUT_SIZE = 256; 

//*** CALSS *****************************************************************************

class Robot;

//--- PersistentData --------------------------------------------------------------------

struct PersistentRobotData {
  float encoder_lut[NUM_JOINTS][ENCODER_LUT_SIZE];
};

//--- SharedData ------------------------------------------------------------------------

enum class ERobotState {
  IDLE = 0,
  BUFFERING_PATH = 1,
  EXECUTING_PATH = 2,
  ERROR = 3
};

//--- SharedData ------------------------------------------------------------------------

// shared data used to communicte between CPU cores
struct SharedData {
  SharedData(int hw_spinlock_id=0){
    lock = spin_lock_instance(hw_spinlock_id);
  };

  volatile float joint_target_positions[NUM_JOINTS];
  volatile float joint_target_velocities[NUM_JOINTS];
  spin_lock_t* lock = nullptr;
};

//--- RobotJoint ------------------------------------------------------------------------

class RobotJoint {
  public:
    RobotJoint(MT6835Encoder* encoder, TB6612MotorDriver* motor_driver, int pole_pairs);
    ~RobotJoint();

    void init(int joint_idx);
    bool calibrate(bool print_measurements);
    void update(float dt, float one_over_dt);
    void update_target(float p, float v);
    bool load_calibration();
    bool store_calibration();

  private:
    std::string calib_data_filename(std::string data_name) const;

  public:
    int joint_idx = 0;
    bool is_homed = false;
    bool is_calibrated = false;

    float position;
    float velocity;

    MT6835Encoder* encoder;
    TB6612MotorDriver* motor_driver;
    ServoController* servo_controller;
};

//--- Robot -----------------------------------------------------------------------------

class Robot : public ICommandProcessor {
  public:
    Robot(float path_segment_time_step);
    ~Robot();

    void init();
    void calibrate();
    bool home(uint8_t joint_mask, float retract_angles[NUM_JOINTS]);
    bool calibrate_joint(int joint_idx, bool store_calibration, bool print_measurements);
    void enable_servo_control(bool enable);  // enables joint servo controll if homed and calibrated

    void update_command_parser();            // called from main loop
    void update_path_planner();              // called from main loop
    void update_servo_controllers(float dt); // called from seperate cpu-core

    void set_pose(const Pose6DF& pos);
    Pose6DF pose_from_joint_angles();

  public:
    void send_reply(const char* str) override;
    bool can_process_command(const GCodeCommand& cmd) override;
    void process_command(const GCodeCommand& cmd, std::string& reply) override;

    void process_motion_command(const GCodeCommand& cmd, std::string& reply);
    void process_set_pose_command(const GCodeCommand& cmd, std::string& reply);
    void process_dwell_command(const GCodeCommand& cmd, std::string& reply);
    void process_machine_command(const GCodeCommand& cmd, std::string& reply);
    void process_set_servo_parameter_command(const GCodeCommand& cmd, std::string& reply);
    void process_home_command(const GCodeCommand& cmd, std::string& reply);
    void process_calibrate_joint_command(const GCodeCommand& cmd, std::string& reply);
    void process_rot_command(const GCodeCommand& cmd, std::string& reply);
    void process_temp_command(const GCodeCommand& cmd, std::string& reply);

  protected:
    bool check_all_joints_ready();   // checks if all joints are homed and calibrated
    static bool update_motion_controller_isr(repeating_timer_t* timer); // called from update timer

  private:
    ERobotState state;
    uint32_t path_buffering_time_us;
    uint64_t path_buffering_start_time;

    bool all_joints_ready;
    RobotJoint* volatile joints[NUM_JOINTS];
    spin_lock_t* joints_spin_lock = nullptr;

    IKinematicModel* kinematic_model;
    PathPlanner path_planner;
    MotionController motion_controller;
    CommandParser command_parser;

    Pose6DF current_pose;
    LinearAngular max_acceleration;
    LinearAngular current_feedrate;

    SharedData shared_data;

    struct repeating_timer motion_controller_update_timer;
    uint64_t last_mc_update_time;

    FrequencyCounter servo_loop_frequency_counter;
    FrequencyCounter motion_controller_frequency_counter;
    Peripheral* peripheral;
};
