#!/usr/bin/env bash
set -euo pipefail

# Guard: refuse to run if controller_manager is active
if ros2 service list 2>/dev/null | grep -q "/controller_manager"; then
  echo "[motor_test] controller_manager is running; stop ROS before manual motor test." >&2
  exit 2
fi

L_PWM=${L_PWM:-26}
L_IN1=${L_IN1:-19}
L_IN2=${L_IN2:-13}
R_PWM=${R_PWM:-22}
R_IN3=${R_IN3:-6}
R_IN4=${R_IN4:-5}

sudo -n true 2>/dev/null || { echo "[motor_test] sudo without password required" >&2; exit 3; }

if ! pigs t >/dev/null 2>&1; then
  sudo /usr/local/bin/pigpiod >/dev/null 2>&1 || sudo pigpiod >/dev/null 2>&1 || true
  sleep 1
fi

# 20 kHz, 0..255
pigs pfs $L_PWM 20000 >/dev/null || true
pigs pfs $R_PWM 20000 >/dev/null || true
pigs prs $L_PWM 255   >/dev/null || true
pigs prs $R_PWM 255   >/dev/null || true

# Stop
pigs p $L_PWM 0 >/dev/null || true; pigs p $R_PWM 0 >/dev/null || true
pigs w $L_IN1 0 >/dev/null || true; pigs w $L_IN2 0 >/dev/null || true
pigs w $R_IN3 0 >/dev/null || true; pigs w $R_IN4 0 >/dev/null || true
sleep 0.2

echo "[motor_test] Forward both 2.5s @100%"
pigs w $L_IN1 1 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 1 >/dev/null; pigs w $R_IN4 0 >/dev/null
pigs p $L_PWM 255 >/dev/null; pigs p $R_PWM 255 >/dev/null
sleep 2.5

echo "[motor_test] Stop"
pigs p $L_PWM 0 >/dev/null; pigs p $R_PWM 0 >/dev/null
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 0 >/dev/null
sleep 0.8

echo "[motor_test] Reverse both 2.0s @100%"
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 1 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 1 >/dev/null
pigs p $L_PWM 255 >/dev/null; pigs p $R_PWM 255 >/dev/null
sleep 2.0

echo "[motor_test] Final stop"
pigs p $L_PWM 0 >/dev/null; pigs p $R_PWM 0 >/dev/null
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 0 >/dev/null

echo "[motor_test] DONE"

