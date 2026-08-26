# ESC V0.3 Firmware

Firmware for a custom three-phase BLDC/PMSM controller built around an
STM32G431RBT6 and a DRV8353S three-phase gate driver.

The project is an active motor-control development platform. It now includes
hardware bring-up tools, synchronized phase-current sensing, encoder feedback,
open-loop commutation, field-oriented control (FOC), experimental SVPWM
schedulers, and an initial CAN control-plane prototype.

> [!WARNING]
> This firmware controls high-current power electronics and is not
> production-ready. Use a current-limited supply, suitable fusing, an unloaded
> motor or secured test fixture, thermal monitoring, and an immediate hardware
> power-disconnect method. Confirm PWM polarity and dead time at the gate-driver
> inputs before energizing the power bus.

## Current Build

The repository is currently configured to run the encoder-based FOC demo. The
experimental Will SVPWM path is disabled but preserved in the source.

| Selector | Value | Effect |
| --- | ---: | --- |
| `WILL_SVPWM_EXPERIMENT_ENABLE` | `0` | Excludes the standalone SVPWM probe from startup |
| `MOTOR_FOC_DEMO_ENABLE` | `1` | Selects `Motor_FOC_Demo()` |
| `FOC_POSITION_DEMO_ENABLE` | `1` | Enables the geared position/velocity/torque sequence |
| `FOC_TORQUE_ONLY_DEMO_ENABLE` | `0` | Does not bypass position and velocity stages |
| `FOC_IMPEDANCE_DEMO_ENABLE` | `0` | Impedance-only profile disabled |
| `FOC_COMPOSITE_DEMO_ENABLE` | `0` | Five-stage composite profile disabled |
| `FOC_VELOCITY_HEAT_TEST_ENABLE` | `0` | Long 5000 rpm thermal profile disabled |
| `FOC_CURRENT_STEP_TEST_ENABLE` | `0` | Current-step diagnostic disabled |
| `FOC_LOW_SPEED_VELOCITY_TEST_ENABLE` | `0` | Low-speed diagnostic disabled |
| `CAN_FIRMWARE_ROLE` | `0` | CAN role prototype disabled |

The selected FOC run performs automatic rotor alignment and then exercises the
geared output-position PID, ramped velocity PI, and Kt-based torque-control
stages. FOC starts with 56 TIM1 dead-time ticks, approximately 500 ns at the
112 MHz timer clock.

## Development Progress

### Hardware and gate-driver bring-up

- Six complementary TIM1 outputs for a three-phase inverter
- Explicit PWM off/on sequencing and main-output-enable handling
- DRV8353S register reads, writes, verification, and raw fault dumps
- Bit-banged SPI diagnostics for the board's crossed PA6/PA7 data routing
- DRV `nFAULT` monitoring with immediate PWM and gate-driver shutdown
- UART1 diagnostics on PC4 at 9600 baud, 8-N-1
- DWT cycle-counter timing for short deterministic delays
- Status GPIOs for timing and test-window observation

The normal startup currently preserves the verified DRV8353S reset
configuration rather than applying the full experimental driver configuration.
Manual CSA calibration is enabled during current-offset calibration and then
removed before motor operation.

### Phase-current sensing

- Three ADC1 current channels with PWM-synchronized sampling
- 512-sample, PWM-off offset calibration for each channel
- 1 mOhm shunt and verified 20 V/V CSA conversion
- Channel-specific offsets and raw ADC saturation checks
- Sampling-window validity tracking
- Dynamic two-shunt reconstruction when one phase is not measurable
- Phase-B PWM-dependent bias correction used by the FOC path
- Moving-average and diagnostic current helpers
- Software current and d/q fault thresholds for the applicable demos
- Optional pre-fault and CSV telemetry capture

Recent zero-current measurements were centered near half scale with roughly
10-12 ADC counts peak-to-peak per channel. These are board observations, not
universal calibration constants; calibration is repeated at startup.

### Encoder and field-oriented control

- AS5048A magnetic encoder support over SPI3 with parity/error handling
- Automatic electrical-zero alignment
- Verified encoder direction and current polarity settings
- Angle prediction and bounded observer correction for sensor latency
- 20 kHz d/q current loop with Clarke/Park and inverse transforms
- Sine lookup table with interpolation for fast trigonometry
- PI d/q current regulators with saturation handling
- Space-vector/common-mode voltage limiting
- Encoder-staleness, innovation, overspeed, overcurrent, and DRV fault handling
- Geared output-position PID with trajectory generation and anti-windup
- Output-velocity PI with acceleration/deceleration ramps
- Kt-based positive and negative torque commands

Additional selectable FOC profiles remain available in `Core/Src/main.c`:

- Torque-only demonstration
- Fixed-angle impedance control
- Composite impedance, position, velocity, open-loop SVPWM, and six-step demo
- Long velocity thermal run
- Encoder-locked current-step test
- Low-speed velocity test

### Open-loop six-step and SVPWM

- Six-step commutation with complementary PWM and a floating phase
- Rotor-alignment and frequency/duty ramp helpers
- Center-aligned open-loop SVPWM with modulation and speed ramps
- Static stator-field orientation test
- Finite and continuous probe modes
- Current logging and driver-fault shutdown paths

At 112 MHz with TIM1 prescaler `0` and ARR `2799`, center-aligned PWM is
nominally 20 kHz:

```text
112,000,000 / (2 x (2799 + 1)) = 20,000 Hz
```

Electrical and mechanical rotation are related by the motor's pole-pair count:

```text
mechanical RPM = electrical Hz x 60 / pole pairs
```

The configured motor has 14 pole pairs, so one mechanical revolution requires
14 electrical revolutions.

### SVPWM continuity and acoustic-noise investigation

The standalone Will SVPWM work has been retained for controlled A/B testing:

- `W_SVPWM_Start_Efficient()` preserves the original direct implementation.
- `W_SVPWM_Start_Efficient_Ramp()` preserves the user-readable ramp version.
- `W_SVPWM_Start_Efficient_Copilot()` is the separate aligned and time-based
  ramp implementation.
- `W_SVPWM_Start_Efficient_Ramp_Counter()` copies the readable ramp while
  replacing its one-bit update flag with the monotonic timer-event counter.

The code uses float constants (`PI_F` and `TWO_PI_F`) instead of double-typed
`M_PI` expressions in the real-time modulation math. The timer callback keeps
both the original update flag and a monotonic `svpwm_update_count`, allowing the
implementations to be compared without deleting either approach.

The counter-ramp clone applies every elapsed timer event to the original
`+0.0005f` rad/s-per-event velocity ramp and angle accumulator. At the end of a
normal probe it reports the number of backlogged updates and the largest batch
seen by the foreground loop. A zero backlog with a maximum batch of one means
the loop did not observe multiple pending timer events.

Hardware observations so far:

- The measured SVPWM waveform appeared continuous and correctly shaped on the
  oscilloscope; small apparent jumps may still depend on probe triggering and
  sample depth.
- A low-modulation, higher-electrical-frequency test ran without the earlier
  high-pitched noise.
- The readable ramp improved the sound but remained noisier than the separate
  Copilot implementation, motivating the counter-only comparison.
- A 1 electrical Hz, 0.5 modulation test drew approximately 125 W and asserted
  DRV `nFAULT`; firmware captured raw status values `0x480` and `0x040` and
  inhibited PWM. That operating point should not be repeated without strict
  current limiting and further diagnosis.
- TIM1 dead time for the standalone comparison was reduced to 11 ticks,
  approximately 98 ns before any timing added by the DRV8353S. The restored FOC
  demo continues to request the more conservative 56-tick/500 ns setting.

The disabled probe constants are currently 100 electrical Hz, 120 seconds, and
0.05 modulation. Re-enabling the probe requires deliberately changing
`WILL_SVPWM_EXPERIMENT_ENABLE`; do not enable it merely to build the project.

### CAN progress

An initial classic-CAN control-plane protocol is present but disabled by
default. It includes:

- 1 Mbit/s FDCAN configuration
- Node heartbeat and status flags
- Global emergency-stop identifier
- State-command parsing and validation
- Fault-latched safe states and forced PWM inhibition
- A second-device sender/test-controller role

This is currently a safe control-plane increment, not a completed motor command
or telemetry interface.

## Hardware Target

| Component | Configuration |
| --- | --- |
| Microcontroller | STM32G431RBT6, LQFP64 |
| Gate driver | DRV8353S |
| Inverter interface | Six-PWM mode |
| System/TIM1 clock | 112 MHz |
| PWM/current-loop rate | 20 kHz |
| Phase PWM timer | TIM1 |
| Current sensing | ADC1 channels 6, 7, and 8 |
| Current shunts / CSA | 1 mOhm / verified 20 V/V |
| Rotor encoder | AS5048A over SPI3 |
| Motor pole pairs | 14 |
| Motor/output gear ratio | 11:1 |
| Gate-driver diagnostics | SPI1/bit-banged PA4-PA7 path |
| Communications | USART1, FDCAN1, USB |
| Debug interface | SWD |

The pin assignments, shunt scaling, PWM polarity, current polarity, encoder
direction, pole-pair count, and gear ratio are hardware-specific and must be
reviewed before using this firmware on another board or motor.

## Important Source Locations

Most test selectors, gains, limits, and motor constants are near the top of
`Core/Src/main.c`. The standalone SVPWM experiment selector is in
`Core/Inc/main.h` because it also controls timer interrupt setup in the HAL MSP
and interrupt source files.

```text
ESC-V0.3-Firmware/
|-- Core/
|   |-- Inc/main.h                 hardware definitions and SVPWM selector
|   |-- Src/main.c                 control code, tests, and configuration
|   |-- Src/stm32g4xx_hal_msp.c    peripheral and interrupt setup
|   `-- Src/stm32g4xx_it.c         interrupt handlers
|-- Drivers/                       STM32 HAL and CMSIS
|-- Debug/                         generated debug-build output
|-- ESC-V0.3-Firmware.ioc          STM32CubeMX configuration
|-- STM32G431RBTX_FLASH.ld         linker script
|-- .project                       STM32CubeIDE project
`-- .cproject                      Eclipse/CDT build configuration
```

Regenerating code from the `.ioc` file may overwrite changes made outside STM32
`USER CODE` sections. Review generated diffs before accepting them.

## Build and Flash

1. Install STM32CubeIDE with the STM32G4 GNU Arm toolchain.
2. Import the repository as an existing project.
3. Select the `Debug` build configuration.
4. Build and flash through ST-LINK/SWD.
5. Observe the complete UART startup diagnostics before enabling bus power.

The command-line build uses the generated makefiles in `Debug/` and
`mingw32-make` supplied with STM32CubeIDE.

## Hardware Test Checklist

1. Keep the power stage unenergized and verify logic power, clocks, UART, and
   SWD access.
2. Confirm the DRV8353S register dump and `nFAULT` state.
3. Verify all six PWM inputs, complementary polarity, and dead time with an
   oscilloscope.
4. Confirm current offsets and noise while PWM is inhibited.
5. Check encoder direction, angle continuity, and magnet diagnostics.
6. Energize at reduced bus voltage with a strict supply-current limit.
7. Keep the shaft unloaded for automatic FOC alignment and speed tests.
8. Use a secured fixture for torque or impedance tests; do not restrain the
   shaft by hand.
9. Stop immediately for a DRV fault, unexpected current, rapid heating, loss of
   synchronization, or incorrect encoder motion.

## License

This repository does not currently include a project-level license. Add one
before distributing the project or granting reuse. STM32 HAL, CMSIS, and
generated STMicroelectronics files remain subject to their respective terms.
