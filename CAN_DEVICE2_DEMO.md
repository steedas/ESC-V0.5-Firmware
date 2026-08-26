# Two-board CAN control-plane demo

This demo uses two identical ESC boards with different firmware roles:

- Device 1: target actuator. It owns the drive state machine, receives CAN
  commands, and transmits heartbeat `0x081`.
- Device 2: sender/test controller. Its own motor outputs remain disabled. It
  receives device 1's heartbeat and sends the test commands.

Both images use Classical CAN at 1 Mbit/s.

## Firmware images

- Device 1:
  `Debug/Device1/ESC-V0.3-Device1-CAN-Actuator.elf`
- Device 2:
  `Debug/Device2/ESC-V0.3-Device2-CAN-Sender.elf`

Equivalent `.hex` and raw `.bin` files are in the same directories. The ELF
or HEX image is preferred because it carries the flash address.

The normal/default CubeIDE build currently produces the device 2 sender role,
so the ordinary Build/Run workflow can be used to program device 2. Before
rebuilding device 1 later, change `CAN_FIRMWARE_ROLE` in `Core/Src/main.c` back
to `CAN_FIRMWARE_ROLE_ESC`.

## Wiring and safe setup

1. Leave the motor phase wires disconnected and the 24 V motor bus off for
   this control-plane test.
2. Connect CAN-H to CAN-H, CAN-L to CAN-L, and logic ground to logic ground.
3. Both boards require powered CAN transceivers; do not connect STM32 FDCAN TX
   and RX pins directly to one another.
4. Terminate the two physical ends of the bus with 120 ohms. With power off,
   the resistance between CAN-H and CAN-L should be approximately 60 ohms.
5. Connect the UART adapter to device 2 so its progress is visible at 9600
   baud, 8-N-1.

## Flashing and startup order

1. Flash the device 2 sender image first.
2. Start device 2 and wait for:

   `Device 2 CAN command demo ready; waiting for device 1 heartbeat 0x081`

3. Flash or reset device 1 using the actuator image.
4. Device 2 will validate the initial disabled heartbeat and run the test
   automatically.

Starting device 2 first is important: it provides CAN acknowledgement as soon
as device 1 begins transmitting heartbeats and avoids leaving device 1 in
bus-off after transmitting onto an unacknowledged bus.

## Automatic test sequence

Device 2 performs one pass and advances only after the next heartbeat confirms
the expected state:

1. Request `ARMED`, sequence 1.
2. Request `ACTIVE`, sequence 2.
3. Request `DISABLED`, sequence 3.
4. Send global E-stop.
5. Clear the E-stop fault, sequence 4.

Each command is retried up to three times if no matching heartbeat arrives
within 500 ms. The test ends with device 1 disabled and both boards' motor
outputs inhibited.

A successful device 2 UART log ends with:

`Device 2 CAN demo PASSED: all state and fault transitions confirmed; both motor outputs remain inhibited`
