# ESC V0.3 Firmware

Firmware for a custom three-phase brushless DC motor controller built around an **STM32G431RBT6** microcontroller and **DRV8353** three-phase gate driver.

The project is an experimental motor-control platform used to develop and test low-level inverter control, gate-driver communication, phase-current sensing, six-step commutation, and open-loop space-vector PWM (SVPWM).

> [!WARNING]
> This firmware controls high-current power electronics and is under active development. It is not production-ready and should not be used without appropriate current limiting, fusing, isolation, thermal monitoring, and an emergency shutdown method.

## Current Status

The firmware currently supports open-loop motor operation and hardware bring-up. It does **not** yet implement closed-loop field-oriented control.

Implemented:

* STM32G431 peripheral initialization using STM32 HAL
* Six complementary TIM1 PWM outputs for a three-phase inverter
* DRV8353 SPI register read/write
* DRV8353 fault-register polling and status indication
* Open-loop six-step commutation
* Open-loop SVPWM with electrical-frequency ramping
* Three-channel phase-current measurement through ADC1
* Current-sense offset calibration utilities
* DWT cycle-counter timing for microsecond delays
* UART debug output
* Configured FDCAN, SPI3, I2C3, and USB peripherals for continued development

Planned:

* Encoder-based rotor-angle feedback
* Closed-loop current control
* Field-oriented control
* Torque and velocity control
* CAN command and telemetry protocol
* Hardware-backed overcurrent and thermal shutdown
* Configuration storage and calibration persistence

## Hardware Target

| Component                   | Configuration             |
| --------------------------- | ------------------------- |
| Microcontroller             | STM32G431RBT6             |
| Package                     | LQFP64                    |
| Gate driver                 | DRV8353                   |
| Inverter interface          | Six-PWM mode              |
| Phase PWM timer             | TIM1                      |
| Current sensing             | ADC1 channels 6, 7, and 8 |
| Gate-driver interface       | SPI1, 16-bit              |
| Additional sensor interface | SPI3                      |
| Communications              | FDCAN1, USART1, USB       |
| System clock                | 112 MHz                   |
| Debug interface             | SWD                       |

The code is designed specifically for the matching ESC V0.3 hardware. Pin assignments, current-sense scaling, PWM polarity, and gate-driver settings must be reviewed before using it with another board.

## Motor-Control Modes

### Open-Loop Six-Step Commutation

The six-step implementation energizes two phases at a time while leaving the third phase disconnected. Electrical frequency is converted into a delay between the six commutation states.

Available development routines include:

* Fixed-frequency six-step commutation
* Six-step commutation with phase-current logging
* Filtered current measurements
* Frequency ramp testing

### Open-Loop SVPWM

The SVPWM implementation generates a rotating voltage vector without encoder feedback:

1. The electrical angle is advanced using the commanded electrical frequency.
2. An inverse Park transform converts the requested `Vd` and `Vq` values into the stationary reference frame.
3. Three phase-voltage references are generated.
4. Common-mode offset injection centers the phase commands within the available DC-bus range.
5. TIM1 compare registers are updated with the resulting phase duties.

TIM1 is changed to center-aligned mode for SVPWM. With the current 112 MHz timer clock, prescaler of `0`, and period of `2799`, the PWM carrier is nominally **20 kHz** in center-aligned mode.

Because this mode is open loop, the generated electrical angle is not synchronized to the actual rotor position. Excessive acceleration, insufficient voltage, or load disturbances can cause the motor to lose synchronization.

## Important Configuration

Motor-control constants are currently located near the top of `Core/Src/main.c`.

| Constant                     | Current value | Description                                    |
| ---------------------------- | ------------: | ---------------------------------------------- |
| `MOTOR_POLE_PAIRS`           |          `12` | Motor pole-pair count used for RPM calculation |
| `MOTOR_START_ELECTRICAL_HZ`  |         `5.0` | Initial electrical frequency                   |
| `MOTOR_HIGH_ELECTRICAL_HZ`   |       `500.0` | Ramp target                                    |
| `MOTOR_LOW_ELECTRICAL_HZ`    |        `10.0` | Low-speed target used by test routines         |
| `MOTOR_RAMP_EHZ_PER_SEC`     |        `15.0` | Open-loop electrical acceleration              |
| `MOTOR_SVPWM_UPDATE_US`      |         `250` | SVPWM reference update interval                |
| `MOTOR_OPEN_LOOP_MODULATION` |        `0.08` | Open-loop voltage command                      |
| `MOTOR_SVPWM_MAX_INDEX`      |     `0.57735` | Maximum permitted modulation command           |
| `MOTOR_TIM1_DEADTIME_TICKS`  |           `0` | MCU timer dead time                            |

The current-sense conversion is defined by:

```c
#define ADC_VREF      3.3f
#define CURRENT_GAIN  (0.0015f * 20.0f)
```

`CURRENT_GAIN` must equal the actual shunt resistance multiplied by the DRV8353 current-shunt-amplifier gain. Update this value if the hardware shunt or amplifier gain differs.

> [!CAUTION]
> `MOTOR_TIM1_DEADTIME_TICKS` is currently set to zero. Confirm that adequate dead time is configured in the DRV8353 before enabling the power stage. Never assume that the existing settings are safe for a different MOSFET, gate resistance, bus voltage, or PCB layout.

## Repository Structure

```text
ESC-V0.3-Firmware/
├── Core/
│   ├── Inc/                    # Application and interrupt headers
│   ├── Src/
│   │   └── main.c             # Motor-control and application logic
│   └── Startup/               # STM32 startup assembly
├── Drivers/
│   ├── CMSIS/                 # ARM and STM32 device support
│   └── STM32G4xx_HAL_Driver/  # STM32 HAL drivers
├── Debug/                     # Generated debug-build files
├── ESC-V0.3-Firmware.ioc      # STM32CubeMX hardware configuration
├── STM32G431RBTX_FLASH.ld     # Linker script
├── .project                   # STM32CubeIDE project definition
└── .cproject                  # Eclipse/CDT build configuration
```

## Getting Started

### Requirements

* STM32CubeIDE
* ST-LINK programmer/debugger
* ESC V0.3 controller hardware
* Current-limited DC power supply
* Compatible three-phase BLDC or PMSM motor
* Oscilloscope for verifying gate and phase behavior
* Appropriate fuse and emergency power-disconnect method

### Clone the Repository

```bash
git clone https://github.com/steedas/ESC-V0.3-Firmware.git
cd ESC-V0.3-Firmware
```

### Import and Build

1. Open STM32CubeIDE.
2. Select **File → Import**.
3. Choose **Existing Projects into Workspace**.
4. Select the cloned repository.
5. Build the `Debug` configuration.
6. Connect an ST-LINK programmer through SWD.
7. Flash and debug the firmware.

The `.ioc` file can be opened in STM32CubeMX or STM32CubeIDE to inspect peripheral and pin configuration. Regenerating code may overwrite changes made outside the STM32 `USER CODE` sections.

## Initial Bring-Up Procedure

Before connecting a motor:

1. Power the logic section from a current-limited supply.
2. Confirm that the 3.3 V rail is stable.
3. Verify the STM32 clock and SWD connection.
4. Confirm that `DRV_ENABLE` remains in a safe state during startup.
5. Read the DRV8353 fault registers over SPI.
6. Verify all six PWM signals without the MOSFET power bus energized.
7. Confirm complementary polarity and dead time at the gate-driver inputs.
8. Energize the power stage at reduced bus voltage with a strict current limit.
9. Test with no mechanical load before increasing voltage, modulation, frequency, or acceleration.

Immediately disable power if the MOSFETs heat rapidly, the supply enters current limit, the motor loses synchronization, or the DRV8353 reports a fault.

## Debugging

The firmware exposes two status outputs:

* `STATUS_1` is used for SPI and DRV8353 fault indication.
* `STATUS_2` is toggled during SVPWM updates as a basic activity indicator.

Useful runtime values include:

* Target electrical frequency
* Actual commanded electrical frequency
* Estimated mechanical RPM
* PWM frequency
* Modulation index
* Phase-current measurements
* DRV8353 fault-status registers

Mechanical speed is estimated from electrical frequency:

```text
mechanical RPM = electrical frequency × 60 / pole pairs
```

This is only a commanded-speed estimate during open-loop operation; it is not a measured rotor speed.



## License

This repository does not currently include a project-level license. Add a license before distributing or permitting reuse of the project.

STM32 HAL, CMSIS, and generated STMicroelectronics files remain subject to their respective license terms.
