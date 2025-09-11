// Minimal ros2_control hardware interface for L298N + encoders using pigpio
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

namespace l298n_hardware {

struct EncoderPins {
  int a{-1};
  int b{-1};
};

class QuadEncoder {
 public:
  QuadEncoder();
  void attach(int pi_handle, int pin_a, int pin_b, unsigned glitch_us);
  void detach();
  int64_t ticks() const { return ticks_; }
  void reset_ticks(int64_t v = 0) { ticks_ = v; }

 private:
  static void cb_func_ex(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void *userdata);
  void handle_edge(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

  int pi_ = -1;
  int pin_a_ = -1;
  int pin_b_ = -1;
  volatile int64_t ticks_ = 0;
  int last_state_ = 0;
  bool attached_ = false;
  unsigned cb_id_a_ = 0;
  unsigned cb_id_b_ = 0;
};

class L298NSystemHardware : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  // Helpers
  bool init_gpio();
  void set_motor(int idx, double cmd_rad_s);
  void set_motor(int idx, double cmd_rad_s, double dt);
  void stop_all(bool brake);

  // Parameters
  int left_pwm_{26};
  int left_in1_{19};
  int left_in2_{13};
  int right_pwm_{22};
  int right_in3_{6};
  int right_in4_{5};
  EncoderPins left_enc_{};
  EncoderPins right_enc_{};
  double ticks_per_rev_{20.0};
  double max_wheel_rad_s_{20.0};
  double deadband_rad_s_{0.1};
  double slew_duty_per_s_{50.0};
  int pwm_freq_{20000};
  int pwm_range_{255};
  bool invert_left_{false};
  bool invert_right_{false};
  bool brake_on_zero_{false};
  unsigned enc_glitch_us_{100};
  double watchdog_s_{0.5};
  bool debug_{false};

  // State/command
  std::array<double, 2> pos_{0.0, 0.0};
  std::array<double, 2> vel_{0.0, 0.0};
  std::array<double, 2> cmd_{0.0, 0.0};
  std::array<double, 2> cmd_prev_{0.0, 0.0};
  std::array<int, 2> last_duty_{0, 0};
  std::array<int64_t, 2> ticks_prev_{0, 0};

  QuadEncoder enc_left_{};
  QuadEncoder enc_right_{};
  int pi_ = -1;  // pigpio daemon handle
  std::chrono::steady_clock::time_point last_cmd_tp_{};
  bool gpio_ready_{false};
};

}  // namespace l298n_hardware
