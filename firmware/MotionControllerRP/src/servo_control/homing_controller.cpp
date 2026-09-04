#include "homing_controller.h"
#include "utilities/math_constants.h"
#include "utilities/logging.h"
#include "pico/time.h"
#include <hw_config.h>

HomingController::HomingController() {
  retract_field_velocity = 100.0f; // rad per second
  retract_field_angle = Constants::TWO_PI_F*HOMING_RETRACTION_FIELD_ANGLE;
}

bool HomingController::run_blocking(ServoController* servo_controller, 
                                    float motor_velocity, 
                                    float search_range_angle, 
                                    float current, 
                                    float encoder_angle_to_motor_angle,
                                    float retract_angle_rad)
{
  start(servo_controller, motor_velocity, search_range_angle, current, 
        encoder_angle_to_motor_angle, retract_angle_rad);

  while(is_finished() == false) {
    update();
  }

  finalize();
  return is_successful();
}

void HomingController::start(ServoController* servo_controller, 
                            float velocity, 
                            float search_range_angle, 
                            float current, 
                            float encoder_angle_to_motor_angle,
                            float retract_angle_rad)
{
  float pole_pair_count = servo_controller->get_pole_pair_count();
  float field_angle_to_encoder_angle = 1.0f / pole_pair_count / encoder_angle_to_motor_angle;

  if(retract_angle_rad > 0.0f)
    HomingController::retract_field_angle = retract_angle_rad * pole_pair_count;
  //else if (servo_controller->get_encoder().cs_pin == PIN_ENCODER3_CS) {
  //  // for the 3rd motor, which is used in the z axis and has a different end stop, use a smaller retract angle to avoid hitting the end stop again
  //  HomingController::retract_field_angle = Constants::TWO_PI_F*HOMING_RETRACTION_Z_FIELD_ANGLE;
  //}

  // field_angle_to_rotor_angle = 1.0 / pole_pair_count
  // encoder_angle_to_rotor_angle = encoder_period_pitch/encoder_radius
  // field_angle_to_encoder_angle = field_angle_to_rotor_angle/encoder_angle_to_rotor_angle
  
  eval_field_angle_delta = Constants::TWO_PI_F*0.1f;
  expected_encoder_delta = eval_field_angle_delta * field_angle_to_encoder_angle;
  
  servo_ctrl = servo_controller;
  field_velocity = velocity * pole_pair_count;
  field_angle_search_range = search_range_angle * pole_pair_count;
  homing_current = current;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  // perform 'soft start'
  servo_ctrl->set_motor_enabled(true, false);

  initial_current = servo_ctrl->get_motor_driver().get_amplitude();
  servo_ctrl->get_motor_driver().set_amplitude_smooth(homing_current, 100);
  search_failed = false;

  // motor_driver.rotate_field(Constants::TWO_PI_F*0.5f * (field_velocity>0.0f ? -1.0f : 1.0f), 12.0f);
  start_field_angle = fmodf(motor_driver.get_field_angle(), Constants::TWO_PI_F);

  last_eval_encoder_angle = encoder.read_abs_angle();
  last_time = 0;

  state = State::Homing;
}

void HomingController::update() {
  if (state == State::Idle || state == State::Initializing)
    return;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  // The servo loop is suspended for the whole homing cycle, so this is the only place
  // reading the encoder. A joint that stops being read while it still moves (the joints
  // are mechanically coupled, so retracting one moves the others) loses track of its
  // encoder period and comes out of homing with a permanently wrong absolute position.
  // Keep polling it in every state.
  if (state == State::EndstopFound || state == State::Done) {
    encoder.read_abs_angle();
    return;
  }

  if (state == State::Retracting) {
    update_retract();
    return;
  }

  uint64_t time_us = time_us_64();
  if(last_time == 0) last_time = time_us;
  float dt = float(time_us - last_time) * 1e-6f;
  last_time = time_us;

  // Move motor and read encoder
  field_angle_offset += field_velocity * dt;
  motor_driver.set_field_angle(start_field_angle+field_angle_offset);

  float encoder_angle = encoder.read_abs_angle();

  if (fabs(last_eval_field_angle_offset - field_angle_offset) > eval_field_angle_delta) {
    float encoder_delta = encoder_angle - last_eval_encoder_angle;
    float encoder_velocity_ratio = encoder_delta / expected_encoder_delta;

    // LOG_DEBUG("encoder_delta=%f/ %f", encoder_delta, expected_encoder_delta);
    // LOG_DEBUG("encoder_velocity_ratio=%f", encoder_velocity_ratio);

    if (fabsf(encoder_velocity_ratio) < 0.05f) {
      LOG_DEBUG("End stop detected");
      on_endstop_detected();
      return;
    }

    last_eval_encoder_angle = encoder_angle;
    last_eval_field_angle_offset = field_angle_offset;
  }

  if (fabs(field_angle_offset) > field_angle_search_range) {
    LOG_INFO("End stop not detected");
    search_failed = true;
    state = State::Done;                      // add this
    servo_ctrl->set_motor_enabled(false, false);  // stop driving the coils
    return;
  }
}

void HomingController::on_endstop_detected() {
  state = State::EndstopFound;
  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  if(search_failed) {
    servo_ctrl->set_motor_enabled(false, false);
    return;
  }

  // reset encoder period, the remainder will provide a very repeatable position reference
  encoder.read_abs_angle();
  servo_ctrl->get_encoder().reset_abs_angle_period();

  // the motor is currently held against the end stop by the field, defining a geometric reference
  home_encoder_angle = encoder.read_abs_angle();
  LOG_DEBUG("home_encoder_angle=%f deg", home_encoder_angle*Constants::RAD2DEG);
  if(home_encoder_angle < Constants::TWO_PI_F*0.01 || home_encoder_angle > Constants::TWO_PI_F*0.99)
    LOG_WARNING("encoder angle for %d at home position close to wrap around point !", encoder.cs_pin);
}

void HomingController::start_retract() {
  if (state != State::EndstopFound)
    return;

  if (search_failed) {
    state = State::Done;
    return;
  }

  auto& motor_driver = servo_ctrl->get_motor_driver();

  retract_total_field_angle = retract_field_angle * (field_velocity>0.0f ? -1.0f : 1.0f);
  retract_start_field_angle = motor_driver.get_field_angle();
  retract_duration_us = (uint64_t)(fabsf(retract_total_field_angle/retract_field_velocity)*1e6f);
  retract_start_time = time_us_64();

  state = State::Retracting;
}

// Time based version of TB6612MotorDriver::rotate_field() so all joints can back off from
// their endstop at the same time instead of one after the other.
void HomingController::update_retract() {
  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  uint64_t elapsed_us = time_us_64() - retract_start_time;

  if (elapsed_us >= retract_duration_us) {
    motor_driver.set_field_angle(retract_start_field_angle + retract_total_field_angle);
    encoder.read_abs_angle();
    state = State::Done;
    return;
  }

  float t = retract_duration_us > 0 ? float(elapsed_us)/float(retract_duration_us) : 1.0f;
  motor_driver.set_field_angle(retract_start_field_angle + retract_total_field_angle*t);

  // update encoder so it doesnt miss a period
  encoder.read_abs_angle();
}

bool HomingController::is_retract_finished() const {
  return state == State::Done;
}

void HomingController::finalize() {
  auto& motor_driver = servo_ctrl->get_motor_driver();

  // back off from home position - already done when the caller drove the retraction of all
  // joints in parallel, otherwise run it here (blocking, used by run_blocking())
  if (state == State::EndstopFound)
    start_retract();
  while (state == State::Retracting)
    update();

  // restore previous motor current, keep the encoder alive during the ramp
  motor_driver.set_amplitude_smooth(initial_current, 100, [this](){
    servo_ctrl->get_encoder().read_abs_angle();
  });
}

bool HomingController::is_finished() const {
  return state == State::EndstopFound || state == State::Retracting || state == State::Done;
}

bool HomingController::is_successful() const {
  return is_finished() && !search_failed;
}

float HomingController::get_home_encoder_angle() const {
  return home_encoder_angle;
}


