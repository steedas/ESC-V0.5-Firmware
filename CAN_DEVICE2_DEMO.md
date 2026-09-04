# Two-board CAN position-control demo

This test uses two identical ESC boards with different firmware roles:

- Device 1 is the actuator. It aligns the motor, owns the drive state machine,
  receives position commands, runs FOC, and transmits heartbeat `0x081`.
- Device 2 is the sender/test controller. Its PWM and gate driver remain
  inhibited. It sends the command heartbeat and scripted position sequence.

Both images use Classical CAN at 1 Mbit/s and protocol revision 5. This is a
live motor test: Device 1 executes `0`, `+45`, `+90`, `+45`, `0`, `-45`,
`-90`, `-45`, `0` output degrees at a 100 rpm output-speed limit.

> [!WARNING]
> Keep the output shaft unloaded and the complete travel area clear. Use a
> current-limited supply, suitable fusing, and an immediate hardware
> power-disconnect. Do not hold or restrain the shaft by hand. Stop immediately
> for an unexpected direction, violent alignment, abnormal current, heating,
> noise, or a driver fault.

## Matched firmware images

- Device 1 actuator:
  `Debug/Device1/ESC-V0.3-Device1-CAN-Actuator.elf`
- Device 2 sender:
  `Debug/Device2/ESC-V0.3-Device2-CAN-Sender.elf`

Equivalent `.hex` and raw `.bin` files are in the same directories. Prefer the
ELF or HEX image because it carries the flash address. Both boards must use the
revision-5 images from the same build; revision-4 and earlier images are not
compatible with the measured-feedback frames or sequence rules.

The normal/default CubeIDE build is currently Device 1. Change
`CAN_FIRMWARE_ROLE` in `Core/Src/main.c` to
`CAN_FIRMWARE_ROLE_DEMO_CONTROLLER` when a fresh Device 2 sender build is
needed.

## Wiring and test setup

1. Mechanically unload Device 1's output and make sure its full motion range is
   clear. Connect its motor phases and AS5048A encoder normally.
2. Leave Device 2's motor disconnected. Its firmware also keeps its PWM and
   gate-driver enable inhibited.
3. Connect CAN-H to CAN-H, CAN-L to CAN-L, and logic ground to logic ground.
4. Both boards require powered CAN transceivers; do not connect the STM32 FDCAN
   TX/RX pins directly to one another.
5. Terminate the two physical ends with 120 ohms. With power off, CAN-H to
   CAN-L should measure approximately 60 ohms.
6. Connect UART adapters as needed at 9600 baud, 8-N-1. Device 2 reports command
   receipts and measured completion. Device 1 limits UART output to startup so
   its persistent control loop is not delayed by blocking text transmission.
   Device 2 routes runtime text through a 2048-byte nonblocking transmit queue,
   keeping heartbeat and acknowledgement processing independent of UART speed.
7. Begin with a reduced bus voltage and a strict supply-current limit.

## Flashing and startup order

1. Flash and start Device 2 with the sender image. Wait for:

   `Device 2 CAN command demo ready; waiting for device 1 heartbeat 0x081`

2. Flash or reset Device 1 using the actuator image.
3. Device 1 calibrates current sensing and performs its normal rotor alignment.
   The shaft can move during alignment.
4. Device 1 inhibits PWM, advertises `POSITION_READY`, and waits DISABLED.
5. Device 2 validates that safe heartbeat, then begins after 500 ms.

Starting Device 2 first provides CAN acknowledgement as soon as Device 1 begins
transmitting and supplies the controller heartbeat before Device 1 can accept
`ACTIVE`.

## Safety interlocks and limits

Device 1 energizes only when all of these are true:

- Encoder/current-sense startup and FOC alignment completed.
- Drive state progressed from DISABLED to ARMED to ACTIVE.
- A valid revision-5 Device 2 heartbeat was received recently.
- No local FOC, DRV, E-stop, or CAN watchdog fault is present.

The test uses these deliberately conservative motion limits:

| Limit | Value |
| --- | ---: |
| Scripted travel | `+/-90` output degrees in 45-degree steps |
| Scripted output speed limit | `100 rpm` (`1100 motor rpm` at 11:1) |
| Trajectory acceleration/deceleration | `100 rpm/s` output |
| FOC Iq command | `6 A` |
| Hard phase-current trip | `12 A` |
| Abnormal d/q-current trip | `10 A` |
| Motor overspeed trip | `5000 rpm` |
| Controller heartbeat period | `100 ms` |
| Device 1 heartbeat watchdog | `350 ms` |
| Measured-feedback period | `50 ms` |
| Required completion dwell | `300 ms` |
| Device 1 position timeout | `15 s` |
| Device 2 motion timeout | `16 s` |

If Device 2, its CAN wiring, or its heartbeat disappears while ACTIVE, Device 1
immediately disables PWM and the gate driver and latches CAN fault 2. A local
FOC fault is reported as CAN fault `0x80 | local_fault`.

## CAN frames

| Identifier | Direction | Purpose |
| --- | --- | --- |
| `0x000` | Device 2 -> all | Global E-stop |
| `0x080` | Device 2 -> Device 1 | Controller heartbeat |
| `0x081` | Device 1 -> Device 2 | Actuator state heartbeat |
| `0x101` | Device 2 -> Device 1 | State command |
| `0x181` | Device 1 -> Device 2 | Position-command status |
| `0x201` | Device 2 -> Device 1 | Position command |
| `0x281` | Device 1 -> Device 2 | Measured position/speed feedback |
| `0x301` | Device 1 -> Device 2 | Measured current/error feedback |

The `0x201` position payload is:

| Bytes | Field |
| --- | --- |
| 0--3 | Signed output-position target in millidegrees, little-endian |
| 4--5 | Unsigned output-speed limit in 0.1-rpm units, little-endian |
| 6 | Command sequence |
| 7 | Flags; must be zero |

This remains closed-loop position control. The speed field limits the internal
position trajectory; it is not a separate velocity-mode request. Short moves
can peak below the requested limit because the trajectory also observes its
acceleration and braking limits.

Device 1 uses one output-frame PID to track that reference:
`Iq = Kp*position_error + Ki*integral(position_error) + Kd*d(position_error)/dt`.
For the profiled reference, the derivative is calculated directly as trajectory
speed minus measured output speed. The CAN profile has no disturbance boost,
breakaway injection, gain scheduling, or nested velocity controller. Its gains
are `Kp=0.25 A/degree`, `Ki=0.05 A/(degree s)`, and `Kd=0.20 A/rpm`; the
integral is limited to 1 A and uses conditional anti-windup.

Device 1 accepts targets from -180000 through +180000 mdeg and speed-limit
values from 1 through 4000 (0.1 through 400.0 output rpm) only while ACTIVE.
It replies on `0x181` with bytes
0--6 echoed exactly and byte 7 replaced by the command-result code. This is a
receipt acknowledgement only; it does not claim that motion has completed.

The `0x281` measured position payload is:

| Bytes | Field |
| --- | --- |
| 0--3 | Signed measured output position in mdeg, little-endian |
| 4--5 | Signed measured output speed in 0.1-rpm units, little-endian |
| 6 | Sequence of the target actually applied by the FOC loop |
| 7 | Feedback flags |

Feedback flag bit 0 means a target is valid, bit 1 means `AT_TARGET`, bit 2
means the motor output is enabled, bit 3 reports Iq saturation, and bit 4 means
the position loop is initialized. `AT_TARGET` is asserted only after measured
position is within 250 mdeg and measured speed is within 0.2 rpm continuously
for 300 ms.

The `0x301` current payload contains signed measured Iq in bytes 0--1, signed Iq
reference in bytes 2--3, and signed final target error in mdeg in bytes 4--7.

## Replay-safe sequence rules

State and position commands use independent 8-bit sequence streams. Device 1
accepts the next sequence value modulo 256. If it receives an exact duplicate
of the last accepted command, it returns `ACCEPTED` without repeating the
transition, resetting an integrator, or restarting the trajectory. Reusing a
sequence with different content returns result 8 (`SEQUENCE_CONFLICT`), and an
out-of-order sequence returns result 9 (`STALE_SEQUENCE`). `ARMED` begins a new
position-command stream. An `ARMED` request with state sequence 1 is also the
explicit new-session marker, so `CLEAR FAULT` sequence 4 remains safely
replayable if its acknowledgement is lost while delayed older state commands
remain rejectable.

## Automatic sequence

Device 2 separates transport acknowledgement from physical completion:

1. Request `ARMED`, sequence 1.
2. Request `ACTIVE`, sequence 2. Device 1 enables FOC after resetting its
   current-loop and trajectory state.
3. Send `0`, `+45000`, `+90000`, `+45000`, `0`, `-45000`, `-90000`, `-45000`,
   and `0` mdeg with speed-limit value 1000 (100.0 output rpm). For every target,
   first require the exact `0x181` receipt,
   then wait for matching applied sequence, `AT_TARGET`, position error no more
   than 250 mdeg, and speed no more than 0.2 rpm in `0x281`. Device 1 allows
   15 seconds to settle; Device 2 reports a supervisory timeout after 16
   seconds.
4. Request `DISABLED`, sequence 3. Device 1 immediately inhibits PWM and the
   gate driver.
5. Send global E-stop and confirm the latched fault.
6. Clear the E-stop fault, sequence 4.

Each command receipt is retried up to three times if no matching acknowledgement
arrives within 500 ms. Duplicate retries are safe and do not restart motion. A
successful Device 2 UART log includes one measured-completion line per target
and ends with:

Any command or motion timeout sends global E-stop immediately and stops the
controller heartbeat, providing the actuator watchdog as an independent
shutdown path. The timeout log includes measured/reference Iq and final error.

`Device 2 CAN position demo PASSED: motion commands and fault transitions completed; both motor outputs are now inhibited`

After completion, Device 2 continues transmitting its controller heartbeat but
does not repeat the motion sequence. Device 1 remains in its continuous service
loop, DISABLED with PWM and the gate driver inhibited, and can accept a fresh
ARMED/ACTIVE session without reset.
