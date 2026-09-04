# ESC V0.3 Firmware

Firmware for a custom three-phase BLDC/PMSM controller built around an
STM32G431RBT6 and a DRV8353S three-phase gate driver.

The project is an active motor-control development platform. It now includes
hardware bring-up tools, synchronized phase-current sensing, encoder feedback,
open-loop commutation, field-oriented control (FOC), experimental SVPWM
schedulers, and two-board CAN position control.

> [!WARNING]
> This firmware controls high-current power electronics and is not
> production-ready. Use a current-limited supply, suitable fusing, an unloaded
> motor or secured test fixture, thermal monitoring, and an immediate hardware
> power-disconnect method. Confirm PWM polarity and dead time at the gate-driver
> inputs before energizing the power bus.

## Current Build

The default local build runs one guarded four-stage FOC test through the new
7-pole-pair motor and 19:1 gearbox. After alignment and a five-second PWM-off
countdown, it executes a 25-target bidirectional position choreography. It
progresses through mirrored +/-90, +/-180, and +/-245 degree excursions, sweeps
continuously from +360 to -360 and back, then finishes with asymmetric
cross-zero moves (120, -240, 300, -60, 0 degrees). The position trajectory
permits 180 output rpm and uses 600 output-rpm/s acceleration and deceleration
with a 750 ms dwell at each endpoint. The position and velocity controllers
may command up to 50 A Iq; D/Q and phase-current trips are 75 A and 78 A.

The second stage commands output-speed plateaus of 25, 50, 100, 150, and
approximately 394.7 rpm with 50 output-rpm/s acceleration and 250 output-rpm/s
deceleration. Each initial step lasts three seconds. The final command is
capped at exactly 7500 motor rpm through the 19:1 ratio and is held for roughly
four seconds after its ramp completes. The reference then ramps to zero and
must remain inside the stop-speed tolerance continuously for one second. The
normal profiled position controller then returns the output to the nearest
360-degree equivalent of the angle recorded at the start of position control.
It never unwinds accumulated velocity revolutions. Only after that position
has settled does the third stage engage its 0.20 N m/output-degree virtual
spring for 30 seconds. Stiffness is ramped to zero over three seconds before
PWM is removed. Its overspeed cutoff is 10000 motor rpm, equivalent to
approximately 526.3 output rpm at 19:1.

For the high-inertia tube, velocity PI is reduced to 0.075 A/output-rpm
proportional and 0.01 A/(output-rpm*s) integral, with a 5 A integral clamp and
6x reverse-error unwind. The zero-speed handoff window is eight seconds.

The current PI uses a two-sample D/Q moving average while current protection
continues to use raw samples. Position and velocity operation use a 50 A Iq
command clamp, 75 A abnormal d/q-current trip, 78 A raw phase-current trip,
and 8250 motor-rpm overspeed limit. The compliant stiffness stage uses a 13 A
Iq clamp, 20 A d/q trip, and 22 A phase trip. Use a guarded mechanism, external
bus monitoring, and a hardware power disconnect.

| Selector | Value | Effect |
| --- | ---: | --- |
| `WILL_SVPWM_EXPERIMENT_ENABLE` | `0` | Excludes the standalone SVPWM probe from startup |
| `MOTOR_FOC_DEMO_ENABLE` | `1` | Selects `Motor_FOC_Demo()` |
| `FOC_POSITION_DEMO_ENABLE` | `1` | Enables the shared geared-output FOC demo infrastructure |
| `FOC_ALL_IN_ONE_TEST_ENABLE` | Derived `1` | Runs position, stepped velocity, profiled return, then stiffness locally |
| `FOC_MASS_POWER_DEMO_ENABLE` | `0` | Rotating-mass power demo disabled |
| `FOC_TORQUE_ONLY_DEMO_ENABLE` | `0` | Does not bypass position and velocity stages |
| `FOC_IMPEDANCE_DEMO_ENABLE` | `0` | Impedance-only profile disabled |
| `FOC_PRE_POSITION_IMPEDANCE_ENABLE` | Derived `0` | Restores the stiffness/position/velocity demo when the mass selector is `0` |
| `FOC_CAN_POSITION_DEMO_ENABLE` | Role-derived | Selects live remote position control on device 1 |
| `FOC_FORCE_SCALE_TEST_CONFIG_ENABLE` | `0` | Bounded force-scale test disabled |
| `FOC_COMPOSITE_DEMO_ENABLE` | `0` | Five-stage composite profile disabled |
| `FOC_VELOCITY_HEAT_TEST_ENABLE` | `0` | Long 5000 rpm thermal profile disabled |
| `FOC_CURRENT_STEP_TEST_ENABLE` | `0` | Current-step diagnostic disabled |
| `FOC_LOW_SPEED_VELOCITY_TEST_ENABLE` | `0` | Low-speed diagnostic disabled |
| `CAN_FIRMWARE_ROLE` | `0` | Local demo build; CAN actuator/controller roles disabled |

When the CAN role is disabled, the preserved FOC force-scale run performs
automatic rotor alignment, a five-second PWM-off
arming pause, a slow current-limited move toward the CCW scale, and a
single-direction torque-current ramp after rigid contact is detected. Contact
requires at least 0.25 output degree of travel followed by 500 ms below 0.25
output rpm while the 6 A approach command is nearly saturated and substantial
travel remains. This addresses the August 27 scale run, where the bar stopped
at about -2.75 degrees, and the subsequent run where the reassembled fixture
contacted at about -0.377 degrees. The old -45-degree position-settle
requirement—and then the original 1-degree travel gate—kept the load stage from
starting.

The preserved load profile allows seven seconds to reach a 70 A command at
10 A/s, holds for at least 0.25 second, and ramps back to zero over seven
seconds. The 6 A
position-controlled approach has separate 12 A phase and 10 A d/q thresholds
so it can overcome the measured drivetrain breakaway friction before declaring
contact. The load stage has a 72 A abnormal d/q threshold and 75 A hard phase
threshold. A 120 motor-rpm approach limit and 150 motor-rpm torque-mode
overspeed shutdown, DRV `nFAULT` monitoring, and the
PWM-off completion path remain active. At the configured 11:1 gear ratio,
estimated 0.02984 N m/A motor Kt, and configured 238.1 mm measured lever arm, the
40 A, 50 A, 60 A, and 66 A runs produced 12.443 N m, 15.944 N m, 18.182 N m,
and 19.074 N m. The validated 70 A run produced 20.034 N m from an 8577 g scale
reading, meeting the 20 N m target. FOC
starts with 56 TIM1 dead-time ticks,
approximately 500 ns at the 112 MHz timer clock.

The contact-to-load handoff preserves the 6 A approach torque as the initial
torque-ramp state. An earlier handoff reset Iq to zero for one control interval,
allowing the compressed scale fixture to rebound and produce a misleading
opposite-direction overspeed fault before the load ramp had started.

The 60 A attempt reached approximately 55.5 A before the DRV8353S asserted
`nFAULT`. The FOC shutdown path now removes PWM, reads both latched DRV fault
registers while the driver is still enabled, prints their raw values, and
decodes VDS overcurrent, gate-drive, undervoltage, thermal, CSA overcurrent, and
phase-specific VDS/VGS flags before disabling the driver.

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

The configured motor has 7 pole pairs, so one mechanical revolution requires
7 electrical revolutions.

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

Classic-CAN protocol revision 5 connects the remote position packets to the
proven encoder-based FOC position loop. The current default build selects the
Device 1 actuator role, with both matched role images preserved separately. It
includes:

- 1 Mbit/s FDCAN configuration
- Node heartbeat and status flags
- Global emergency-stop identifier
- State-command parsing and validation
- Signed millidegree position commands with bounded output-speed fields
- Exact position-command receipt acknowledgement before each scripted move
- Measured output position, speed, Iq, Iq reference, and target-error feedback
- `AT_TARGET` only after a continuous 300 ms position/speed settle dwell
- A single remote position PID with 0.25 A/degree proportional, 0.05
  A/(degree s) integral, and 0.20 A/rpm derivative gains, a 1 A integral
  clamp, standard conditional anti-windup, and no disturbance boost or nested
  velocity controller
- A bounded position-reference profile with the commanded speed as its ceiling
  and 100 rpm/s output acceleration and deceleration
- A 15-second absolute actuator settle timeout that intermittent stable samples
  cannot extend
- Device 2 measured-completion validation with a 16-second supervisory timeout
- Immediate E-stop plus heartbeat removal on sender timeout
- Idempotent duplicate handling and stale/conflicting sequence rejection
- Persistent nonblocking Device 1 service across DISABLED, E-stop, and clear
- A nonblocking 2048-byte Device 2 UART log queue so 9600-baud text cannot
  delay CAN heartbeat, retry, or completion processing
- Controller heartbeat at 100 ms and a 350 ms actuator watchdog
- A position-ready interlock after encoder alignment
- Current-limited live FOC with a 6 A Iq command ceiling
- Fault-latched safe states and forced PWM inhibition
- A second-device sender/test-controller role

The sender scripts `0`, `+45`, `+90`, `+45`, `0`, `-45`, `-90`, `-45`, and `0`
output-degree targets at a 100 rpm trajectory limit. Protocol revision 5 uses
0.1-rpm units for command and feedback speed, so the two-byte value `1000`
means 100.0 output rpm. Device 1 validates `+/-180` output-degree and
0.1--400.0 output-rpm bounds, echoes each accepted packet, and executes the
small sweep only after
alignment, a valid controller heartbeat, `ARMED`, and `ACTIVE`. The live demo
uses 6 A Iq, 12 A hard phase-current, and 10 A abnormal d/q-current limits. The
100 output-rpm command corresponds to approximately 1100 motor rpm through the
configured 11:1 gearbox; the overspeed shutdown is 5000 motor rpm. Loss of the
controller heartbeat while ACTIVE immediately inhibits
PWM and the gate driver and latches CAN fault 2. Device 2 does not treat the
receipt echo as motion completion: it requires the matching applied sequence,
`AT_TARGET`, no more than 0.25 output degree of error, and no more than 0.2
output rpm. See `CAN_DEVICE2_DEMO.md` for the exact frames, flash order, and
test procedure.

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
| Motor pole pairs | 7 |
| Motor/output gear ratio | 19:1 |
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

Matched role-specific CAN images are also generated under `Debug/Device1` and
`Debug/Device2`; use those files for the two-board position demonstration.

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
7. Keep the shaft unloaded for automatic FOC alignment. For the rotating-mass
   demo, attach the secured load only during the PWM-off countdown, let it hang
   vertically down for zero capture, then clear and guard the swept volume.
8. Use a secured fixture for torque or impedance tests; do not restrain the
   shaft by hand. Use external DC-bus voltage monitoring and an energy-absorbing
   bus path for any test that commands regenerative torque.
9. Stop immediately for a DRV fault, unexpected current, rapid heating, loss of
   synchronization, or incorrect encoder motion.

## License

This repository does not currently include a project-level license. Add one
before distributing the project or granting reuse. STM32 HAL, CMSIS, and
generated STMicroelectronics files remain subject to their respective terms.
