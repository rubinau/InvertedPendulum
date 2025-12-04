# InvertedPendulum

Collection of Arduino sketches and test utilities for an inverted pendulum control experiment.

## Overview

- **Purpose**: Provide simple sketches to test hardware (motor and encoders) and a controller sketch for running inverted-pendulum experiments.
- **Contents**: The repository contains a main `control.ino` and a `Kode/` folder with helper and test sketches.

## Repository structure

- `control.ino` : Top-level Arduino sketch (project entrypoint / main controller).
- `Kode/` : Supporting sketches and checks
  - `cekMotor/cekMotor.ino` : Motor test / motor driver check sketch.
  - `cek1Encoder/cek1Encoder.ino` : Single encoder test sketch.
  - `cek2Encoder/cek2Encoder.ino` : Dual encoder test sketch.
  - `control/control.ino` : Alternative or modular copy of the controller sketch (kept for organization).

## Hardware & Software Requirements

- Arduino Uno (or compatible) or the board referenced in the sketches.
- Motor driver compatible with your motor (e.g. L298, TB6612, or an appropriate ESC depending on motor type).
- Rotary encoder(s) for angle/position feedback.
- Power supply sized for the motor and driver.
- Arduino IDE (recommended) or PlatformIO to compile and upload sketches.

## Quick Start (Arduino IDE)

1. Open the Arduino IDE and open the sketch you want to run (for example `control.ino` or any sketch inside `Kode/`).
2. Select the correct board from `Tools > Board` and the correct serial port from `Tools > Port`.
3. Verify the sketch with the verify (check) button.
4. Upload the sketch to your board with the upload (right arrow) button.

## Usage suggestions

- Start with the check sketches:
  - Upload `cekMotor/cekMotor.ino` and verify motor rotation and driver wiring.
  - Upload `cek1Encoder/cek1Encoder.ino` or `cek2Encoder/cek2Encoder.ino` to confirm encoder signals and counting.
- When hardware checks pass, upload `control.ino` to run the controller. Use serial printouts (if available) to observe sensor readings and controller state.

## Wiring and Safety

- Ensure power to motors is disconnected while wiring.
- Double-check encoder wiring and motor driver connections before applying power.
- Use appropriate current limiting and fuses where relevant.

## Notes for contributors

- If you update or improve any sketch, keep a short note at the top of the sketch with what changed and when.
- If you add a new hardware-specific wiring diagram, add it to the `docs/` folder (create it) or update this README with wiring details.

## License & Contact

This repository does not include a license file. Add a `LICENSE` file if you want to define reuse terms.

For questions, contact the repository owner.
