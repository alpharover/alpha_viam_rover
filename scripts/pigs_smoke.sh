#!/usr/bin/env bash
set -euo pipefail

# Simple H-bridge smoke test using pigpio's `pigs` CLI.
# Drives each wheel forward and reverse briefly. Off-ground only.
# Uses BCM pins from hw/pinmap.yaml:
#  Left  ENA=26, IN1=19, IN2=13
#  Right ENB=22, IN3=6,  IN4=5
#
# Usage: scripts/pigs_smoke.sh [duty 0-255] [duration_s]

DUTY=${1:-200}
DUR=${2:-1.5}

LPWM=${LPWM:-26}; LIN1=${LIN1:-19}; LIN2=${LIN2:-13}
RPWM=${RPWM:-22}; RIN3=${RIN3:-6};  RIN4=${RIN4:-5}
FREQ=${FREQ:-10000}; RANGE=${RANGE:-255}

echo "[pigs_smoke] Verifying pigpio daemon..."
if ! pigs hwver >/dev/null 2>&1; then
  echo "[pigs_smoke] pigpio daemon not responding. Start with: sudo systemctl start pigpiod" >&2
  exit 1
fi

stop_all() {
  pigs w $LIN1 0 || true; pigs w $LIN2 0 || true; pigs p $LPWM 0 || true
  pigs w $RIN3 0 || true; pigs w $RIN4 0 || true; pigs p $RPWM 0 || true
}
trap stop_all EXIT INT TERM

echo "[pigs_smoke] Configure modes and PWM..."
for p in $LIN1 $LIN2 $RIN3 $RIN4; do pigs m $p 1; done
for p in $LPWM $RPWM; do pigs m $p 1; pigs pfs $p $FREQ; pigs prg $p $RANGE; pigs p $p 0; done

echo "[pigs_smoke] LEFT forward ${DUR}s"
pigs w $LIN1 1; pigs w $LIN2 0; pigs p $LPWM $DUTY; sleep $DUR; pigs p $LPWM 0
sleep 0.3
echo "[pigs_smoke] LEFT reverse ${DUR}s"
pigs w $LIN1 0; pigs w $LIN2 1; pigs p $LPWM $DUTY; sleep $DUR; pigs p $LPWM 0

sleep 0.5
echo "[pigs_smoke] RIGHT forward ${DUR}s"
pigs w $RIN3 1; pigs w $RIN4 0; pigs p $RPWM $DUTY; sleep $DUR; pigs p $RPWM 0
sleep 0.3
echo "[pigs_smoke] RIGHT reverse ${DUR}s"
pigs w $RIN3 0; pigs w $RIN4 1; pigs p $RPWM $DUTY; sleep $DUR; pigs p $RPWM 0

echo "[pigs_smoke] Done. Motors should have spun fwd/rev on each side."

