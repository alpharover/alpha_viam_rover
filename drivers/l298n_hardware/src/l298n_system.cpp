// Implementation of L298N ros2_control SystemInterface using pigpio

#include "l298n_hardware/l298n_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <pigpiod_if2.h>

namespace l298n_hardware {

// ---------------- QuadEncoder -----------------
QuadEncoder::QuadEncoder() {}

void QuadEncoder::attach(int pi_handle, int pin_a, int pin_b, unsigned glitch_us) {
  detach();
  pi_ = pi_handle;
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  ticks_ = 0;
  last_state_ = 0;
  if (pin_a_ >= 0 && pin_b_ >= 0) {
    set_mode(pi_, pin_a_, PI_INPUT);
    set_mode(pi_, pin_b_, PI_INPUT);
    set_pull_up_down(pi_, pin_a_, PI_PUD_UP);
    set_pull_up_down(pi_, pin_b_, PI_PUD_UP);
    if (glitch_us > 0) {
      set_glitch_filter(pi_, pin_a_, glitch_us);
      set_glitch_filter(pi_, pin_b_, glitch_us);
    }
    int a = gpio_read(pi_, pin_a_);
    int b = gpio_read(pi_, pin_b_);
    last_state_ = ((a & 1) << 1) | (b & 1);
    cb_id_a_ = callback_ex(pi_, pin_a_, EITHER_EDGE, &QuadEncoder::cb_func_ex, this);
    cb_id_b_ = callback_ex(pi_, pin_b_, EITHER_EDGE, &QuadEncoder::cb_func_ex, this);
    attached_ = true;
  }
}

void QuadEncoder::detach() {
  if (attached_) {
    if (cb_id_a_) callback_cancel(cb_id_a_);
    if (cb_id_b_) callback_cancel(cb_id_b_);
  }
  attached_ = false;
}

void QuadEncoder::cb_func_ex(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void *userdata) {
  if (level == PI_TIMEOUT) return;
  auto *self = reinterpret_cast<QuadEncoder *>(userdata);
  if (self) self->handle_edge(pi, user_gpio, level, tick);
}

void QuadEncoder::handle_edge(int /*pi*/, unsigned /*user_gpio*/, unsigned level, uint32_t /*tick*/) {
  if (level < 0) return;
  int a = gpio_read(pi_, pin_a_);
  int b = gpio_read(pi_, pin_b_);
  int state = ((a & 1) << 1) | (b & 1);
  static const int8_t tab[4][4] = {
      {0, -1, 1, 0},
      {1, 0, 0, -1},
      {-1, 0, 0, 1},
      {0, 1, -1, 0},
  };
  int8_t delta = tab[last_state_ & 3][state & 3];
  ticks_ += delta;
  last_state_ = state;
}

// ---------------- L298NSystemHardware -----------------
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

CallbackReturn L298NSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parse parameters from URDF <ros2_control><hardware><param name=...>
  auto get_i = [&](const std::string &k, int def) {
    auto it = info_.hardware_parameters.find(k);
    return (it != info_.hardware_parameters.end()) ? std::stoi(it->second) : def;
  };
  auto get_d = [&](const std::string &k, double def) {
    auto it = info_.hardware_parameters.find(k);
    return (it != info_.hardware_parameters.end()) ? std::stod(it->second) : def;
  };
  auto get_b = [&](const std::string &k, bool def) {
    auto it = info_.hardware_parameters.find(k);
    if (it == info_.hardware_parameters.end()) return def;
    const auto &v = it->second;
    return (v == "true" || v == "1" || v == "True");
  };

  left_pwm_ = get_i("left_pwm", left_pwm_);
  left_in1_ = get_i("left_in1", left_in1_);
  left_in2_ = get_i("left_in2", left_in2_);
  right_pwm_ = get_i("right_pwm", right_pwm_);
  right_in3_ = get_i("right_in3", right_in3_);
  right_in4_ = get_i("right_in4", right_in4_);

  left_enc_.a = get_i("left_enc_a", -1);
  left_enc_.b = get_i("left_enc_b", -1);
  right_enc_.a = get_i("right_enc_a", -1);
  right_enc_.b = get_i("right_enc_b", -1);

  ticks_per_rev_ = get_d("ticks_per_rev", ticks_per_rev_);
  max_wheel_rad_s_ = get_d("max_wheel_rad_s", max_wheel_rad_s_);
  deadband_rad_s_ = get_d("deadband_rad_s", deadband_rad_s_);
  slew_duty_per_s_ = get_d("slew_duty_per_s", slew_duty_per_s_);
  pwm_freq_ = get_i("pwm_freq", pwm_freq_);
  pwm_range_ = get_i("pwm_range", pwm_range_);
  invert_left_ = get_b("invert_left", invert_left_);
  invert_right_ = get_b("invert_right", invert_right_);
  brake_on_zero_ = get_b("brake_on_zero", brake_on_zero_);
  enc_glitch_us_ = static_cast<unsigned>(get_i("encoder_glitch_us", static_cast<int>(enc_glitch_us_)));
  watchdog_s_ = get_d("watchdog_s", watchdog_s_);
  debug_ = get_b("debug", debug_);

  // Validate joints: expect exactly 2: left_wheel_joint, right_wheel_joint
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("l298n_hardware"), "Expected 2 joints, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto &j : info_.joints) {
    bool has_vel_cmd = false;
    bool has_pos = false, has_vel = false;
    for (const auto &ci : j.command_interfaces) has_vel_cmd |= (ci.name == HW_IF_VELOCITY);
    for (const auto &si : j.state_interfaces) {
      has_pos |= (si.name == HW_IF_POSITION);
      has_vel |= (si.name == HW_IF_VELOCITY);
    }
    if (!(has_vel_cmd && has_pos && has_vel)) {
      RCLCPP_ERROR(rclcpp::get_logger("l298n_hardware"),
                   "Joint %s missing required interfaces (vel cmd, pos+vel state)", j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  last_cmd_tp_ = std::chrono::steady_clock::now();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> L298NSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(info_.joints[0].name, HW_IF_POSITION, &pos_[0]);
  si.emplace_back(info_.joints[0].name, HW_IF_VELOCITY, &vel_[0]);
  si.emplace_back(info_.joints[1].name, HW_IF_POSITION, &pos_[1]);
  si.emplace_back(info_.joints[1].name, HW_IF_VELOCITY, &vel_[1]);
  return si;
}

std::vector<hardware_interface::CommandInterface> L298NSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(info_.joints[0].name, HW_IF_VELOCITY, &cmd_[0]);
  ci.emplace_back(info_.joints[1].name, HW_IF_VELOCITY, &cmd_[1]);
  return ci;
}

bool L298NSystemHardware::init_gpio() {
  pi_ = pigpio_start(nullptr, nullptr);
  if (pi_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("l298n_hardware"), "pigpio daemon connection failed");
    return false;
  }
  // PWM setup
  for (int pin : {left_pwm_, right_pwm_}) {
    set_mode(pi_, pin, PI_OUTPUT);
    set_PWM_frequency(pi_, pin, pwm_freq_);
    set_PWM_range(pi_, pin, pwm_range_);
    set_PWM_dutycycle(pi_, pin, 0);
  }
  // Direction pins
  for (int pin : {left_in1_, left_in2_, right_in3_, right_in4_}) {
    set_mode(pi_, pin, PI_OUTPUT);
    gpio_write(pi_, pin, 0);
  }
  // Encoders
  enc_left_.attach(pi_, left_enc_.a, left_enc_.b, enc_glitch_us_);
  enc_right_.attach(pi_, right_enc_.a, right_enc_.b, enc_glitch_us_);
  return true;
}

CallbackReturn L298NSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!init_gpio()) return CallbackReturn::ERROR;
  gpio_ready_ = true;
  ticks_prev_[0] = enc_left_.ticks();
  ticks_prev_[1] = enc_right_.ticks();
  pos_[0] = pos_[1] = 0.0;
  vel_[0] = vel_[1] = 0.0;
  cmd_[0] = cmd_[1] = 0.0;
  last_duty_[0] = last_duty_[1] = 0;
  last_cmd_tp_ = std::chrono::steady_clock::now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn L298NSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  stop_all(true);
  if (gpio_ready_ && pi_ >= 0) pigpio_stop(pi_);
  gpio_ready_ = false;
  enc_left_.detach();
  enc_right_.detach();
  return CallbackReturn::SUCCESS;
}

return_type L298NSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
  const double dt = period.seconds();
  if (ticks_per_rev_ > 0.0) {
    int64_t tl = enc_left_.ticks();
    int64_t tr = enc_right_.ticks();
    int64_t dl = tl - ticks_prev_[0];
    int64_t dr = tr - ticks_prev_[1];
    ticks_prev_[0] = tl;
    ticks_prev_[1] = tr;
    const double rad_per_tick = (2.0 * M_PI) / ticks_per_rev_;
    pos_[0] += dl * rad_per_tick;
    pos_[1] += dr * rad_per_tick;
    vel_[0] = dl * rad_per_tick / std::max(1e-6, dt);
    vel_[1] = dr * rad_per_tick / std::max(1e-6, dt);
  } else {
    // Fallback: integrate command (no encoders)
    pos_[0] += cmd_[0] * dt;
    pos_[1] += cmd_[1] * dt;
    vel_[0] = cmd_[0];
    vel_[1] = cmd_[1];
  }
  return return_type::OK;
}

void L298NSystemHardware::set_motor(int idx, double cmd_rad_s, double dt) {
  const bool invert = (idx == 0 ? invert_left_ : invert_right_);
  const int pwm_pin = (idx == 0 ? left_pwm_ : right_pwm_);
  const int in_a = (idx == 0 ? left_in1_ : right_in3_);
  const int in_b = (idx == 0 ? left_in2_ : right_in4_);

  double c = invert ? -cmd_rad_s : cmd_rad_s;
  // Deadband near zero to avoid buzz/stiction
  if (std::fabs(c) < deadband_rad_s_) {
    if (brake_on_zero_) {
      gpio_write(pi_, in_a, 1);
      gpio_write(pi_, in_b, 1);
    } else {
      gpio_write(pi_, in_a, 0);
      gpio_write(pi_, in_b, 0);
    }
    set_PWM_dutycycle(pi_, pwm_pin, 0);
    last_duty_[idx] = 0;
    return;
  }

  double mag = std::min(std::fabs(c) / std::max(1e-6, max_wheel_rad_s_), 1.0);
  int target_duty = static_cast<int>(std::round(mag * pwm_range_));

  // Slew rate limit on duty change (duty units per second)
  int max_step = static_cast<int>(std::round(std::max(0.0, slew_duty_per_s_) * std::max(1e-3, dt)));
  int duty = target_duty;
  if (max_step > 0) {
    int prev = last_duty_[idx];
    if (duty > prev + max_step) duty = prev + max_step;
    if (duty < prev - max_step) duty = prev - max_step;
  }

  // Direction
  if (c >= 0) {
    gpio_write(pi_, in_a, 1);
    gpio_write(pi_, in_b, 0);
  } else {
    gpio_write(pi_, in_a, 0);
    gpio_write(pi_, in_b, 1);
  }
  set_PWM_dutycycle(pi_, pwm_pin, duty);
  if (debug_ && duty != last_duty_[idx]) {
    RCLCPP_INFO(rclcpp::get_logger("l298n_hardware"),
                "motor[%d] cmd=%.3f dir=%s duty=%d/%d (target=%d)",
                idx, cmd_rad_s, (c >= 0 ? "fwd" : "rev"), duty, pwm_range_, target_duty);
  }
  last_duty_[idx] = duty;
}

void L298NSystemHardware::stop_all(bool brake) {
  if (brake) {
    gpio_write(pi_, left_in1_, 1);
    gpio_write(pi_, left_in2_, 1);
    gpio_write(pi_, right_in3_, 1);
    gpio_write(pi_, right_in4_, 1);
  } else {
    gpio_write(pi_, left_in1_, 0);
    gpio_write(pi_, left_in2_, 0);
    gpio_write(pi_, right_in3_, 0);
    gpio_write(pi_, right_in4_, 0);
  }
  set_PWM_dutycycle(pi_, left_pwm_, 0);
  set_PWM_dutycycle(pi_, right_pwm_, 0);
}

return_type L298NSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration & period) {
  const double dt = std::max(1e-3, period.seconds());
  // Update watchdog timestamp when commands are non-zero or changed
  if (std::fabs(cmd_[0]) > 1e-6 || std::fabs(cmd_[1]) > 1e-6 ||
      std::fabs(cmd_[0] - cmd_prev_[0]) > 1e-6 || std::fabs(cmd_[1] - cmd_prev_[1]) > 1e-6) {
    last_cmd_tp_ = std::chrono::steady_clock::now();
    cmd_prev_ = cmd_;
  }

  // Watchdog: if no recent commands within watchdog_s_, stop motors
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - last_cmd_tp_).count();
  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  if (elapsed > watchdog_s_) {
    if (debug_) {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("l298n_hardware"), steady_clock, 2000,
                           "watchdog tripped (%.3fs > %.3fs); braking", elapsed, watchdog_s_);
    }
    stop_all(true);
    return return_type::OK;
  }
  if (debug_) {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("l298n_hardware"), steady_clock, 500,
                         "write dt=%.3f cmdL=%.3f cmdR=%.3f", dt, cmd_[0], cmd_[1]);
  }
  set_motor(0, cmd_[0], dt);
  set_motor(1, cmd_[1], dt);
  return return_type::OK;
}

}  // namespace l298n_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(l298n_hardware::L298NSystemHardware, hardware_interface::SystemInterface)
