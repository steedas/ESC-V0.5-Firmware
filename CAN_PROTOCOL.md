# ESC CAN protocol

This document describes revision 1 of the ESC's small external-control
protocol. Revision 1 implements only the control plane: state commands,
emergency stop, and heartbeat feedback. Motor outputs remain inhibited even
when the reported state is `ACTIVE`. Real-time setpoints will be added in a
later revision.

Two-board firmware roles and their flashing order are documented in
`CAN_DEVICE2_DEMO.md`.

## Physical and data-link configuration

- Classical CAN 2.0, standard 11-bit identifiers
- 1 Mbit/s
- Node ID 1 by default
- FDCAN1 RX on PB8 and TX on PB9
- External CAN transceiver required
- 120 ohm termination at each physical end of the bus

## Message identifiers

| Identifier | Direction | Period | Purpose |
|---|---|---:|---|
| `0x000` | Host to ESC | Event | Global emergency stop |
| `0x081` | ESC to host | 100 ms | Node 1 heartbeat |
| `0x101` | Host to ESC | Event | Node 1 state command |

The node-specific identifiers are `0x080 + node_id` for heartbeat and
`0x100 + node_id` for state commands.

## State command (`0x101`)

The data length must be exactly two bytes.

| Byte | Name | Meaning |
|---:|---|---|
| 0 | Request | `1` disabled, `2` armed, `3` active, `4` clear fault |
| 1 | Sequence | Host-provided rolling sequence number, echoed in heartbeat |

State commands are idempotent. Repeating `ARMED` while already armed or
`ACTIVE` while already active is accepted. The valid progression is:

`DISABLED -> ARMED -> ACTIVE`

`DISABLED` can be requested from any non-fault state. Emergency stop or a
local controller fault enters `FAULT`. A fault must be cleared before arming
again; a local fault cannot be cleared while its cause remains present.

## Global emergency stop (`0x000`)

Any standard data frame received at ID `0x000` immediately inhibits PWM,
drives the gate-driver enable low, and latches the state to `FAULT`. The frame
does not require a payload.

This communication E-stop is a supervisory feature. It is not a substitute
for a hardware safety circuit.

## Heartbeat (`0x081`)

The ESC transmits one eight-byte heartbeat every 100 ms.

| Byte | Name | Meaning |
|---:|---|---|
| 0 | Protocol version | `1` |
| 1 | State | `0` boot, `1` disabled, `2` armed, `3` active, `4` fault |
| 2 | Fault | `0` none, `1` E-stop, `0x80 + local_fault` local fault |
| 3 | Status flags | Bit field described below |
| 4 | Last sequence | Sequence from last valid-length state command |
| 5 | Command result | Result described below |
| 6 | TX drops | Saturating count of heartbeats not queued |
| 7 | Invalid RX | Saturating count of invalid commands/transitions |

Status flags:

| Bit | Meaning |
|---:|---|
| 0 | CAN peripheral started |
| 1 | At least one command was received |
| 2 | Motor output enabled; always zero in revision 1 |
| 3 | Last command was invalid |
| 4 | Communication E-stop is latched |
| 5 | Control-plane-only revision; set in revision 1 |

Command results:

| Value | Meaning |
|---:|---|
| 0 | No command processed |
| 1 | Accepted |
| 2 | Invalid data length |
| 3 | Unknown request |
| 4 | Invalid state transition |
| 5 | Local fault is still present |
| 6 | E-stop received |

## Basic bench check

With a SocketCAN-compatible adapter configured for 1 Mbit/s, the initial
heartbeat should report protocol 1 and state `DISABLED`:

```text
candump can0,081:7FF
```

Example state requests for node 1:

```text
cansend can0 101#0201    # ARMED, sequence 1
cansend can0 101#0302    # ACTIVE, sequence 2
cansend can0 101#0103    # DISABLED, sequence 3
cansend can0 000#        # global E-stop
cansend can0 101#0404    # clear fault, sequence 4
```

After each command, verify state, sequence, and result in the next heartbeat.
The motor and gate driver should remain off throughout this revision-1 test.
