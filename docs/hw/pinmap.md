# Pin Map (Hardware)

Source of truth for GPIO, I²C/SPI, PWM, and encoder pins. Keep in sync with `hw/pinmap.yaml`.

Current draft is in `hw/pinmap.yaml`. Update both files when finalizing.

- PWM (suggested): 12 (PWM0), 13 (PWM1), 18 (PWM0), 19 (PWM1)
- I²C: bus 1, IMU 0x68, INA219 0x40
- SPI: `/dev/spidev0.*` (LiDAR may be UART/USB; SPI for future)
- Encoders (single-channel): Left=GPIO20 (P38), Right=GPIO21 (P40)

Revisions

- v0: initial draft with placeholders

