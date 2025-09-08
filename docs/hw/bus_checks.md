# Bus Verification Checks (Phase 1)

Collect quick evidence that I²C/SPI/UART are alive.

Commands

- I²C devices: `i2cdetect -y 1`
- SPI devices: `ls -l /dev/spidev*`
- UART: loopback or console check (e.g., `ls -l /dev/serial*` and a short loopback with a USB‑UART)

Artifacts

Save outputs/screenshots under `docs/hw/artifacts/phase1/` and link them here.
