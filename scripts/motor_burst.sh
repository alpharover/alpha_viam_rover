#!/usr/bin/env bash
set -euo pipefail

# Safe off-ground motor burst using pigpio only (ROS stopped).
# Usage: scripts/motor_burst.sh [duty_percent]

DUTY_PCT=${1:-80}
L_PWM=${L_PWM:-26}
L_IN1=${L_IN1:-19}
L_IN2=${L_IN2:-13}
R_PWM=${R_PWM:-22}
R_IN3=${R_IN3:-6}
R_IN4=${R_IN4:-5}

if ros2 service list 2>/dev/null | grep -q "/controller_manager"; then
  echo "[motor_burst] controller_manager running; aborting." >&2
  exit 2
fi

sudo -n true 2>/dev/null || { echo "[motor_burst] sudo without password required" >&2; exit 3; }

if ! pigs t >/dev/null 2>&1; then
  sudo /usr/local/bin/pigpiod >/dev/null 2>&1 || sudo pigpiod >/dev/null 2>&1 || true
  sleep 1
fi

RATE=255
DUTY=$(( RATE * DUTY_PCT / 100 ))
echo "[motor_burst] duty=${DUTY} (${DUTY_PCT}%), forward 2.5s, reverse 2.5s"

pigs pfs $L_PWM 20000 >/dev/null || true
pigs pfs $R_PWM 20000 >/dev/null || true
pigs prs $L_PWM $RATE   >/dev/null || true
pigs prs $R_PWM $RATE   >/dev/null || true

# Stop
pigs p $L_PWM 0 >/dev/null || true; pigs p $R_PWM 0 >/dev/null || true
pigs w $L_IN1 0 >/dev/null || true; pigs w $L_IN2 0 >/dev/null || true
pigs w $R_IN3 0 >/dev/null || true; pigs w $R_IN4 0 >/dev/null || true
sleep 0.3

# Forward
pigs w $L_IN1 1 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 1 >/dev/null; pigs w $R_IN4 0 >/dev/null
pigs p $L_PWM $DUTY >/dev/null; pigs p $R_PWM $DUTY >/dev/null
sleep 2.5

# Stop
pigs p $L_PWM 0 >/dev/null; pigs p $R_PWM 0 >/dev/null
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 0 >/dev/null
sleep 0.5

# Reverse
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 1 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 1 >/dev/null
pigs p $L_PWM $DUTY >/dev/null; pigs p $R_PWM $DUTY >/dev/null
sleep 2.5

# Final stop
pigs p $L_PWM 0 >/dev/null; pigs p $R_PWM 0 >/dev/null
pigs w $L_IN1 0 >/dev/null; pigs w $L_IN2 0 >/dev/null
pigs w $R_IN3 0 >/dev/null; pigs w $R_IN4 0 >/dev/null

echo "[motor_burst] DONE"

