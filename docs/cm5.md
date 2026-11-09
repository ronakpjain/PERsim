# CM5 CAN bridge {#cm5_bridge}

@tableofcontents

The virtualized CM5 torque-vector application talks to PERsim over a raw CAN
link. The Yoctocoupler image already provides SocketCAN `vcan0`; the supplied
`guest/vcan_bridge.c` forwards `PF_CAN/SOCK_RAW/CAN_RAW` frames over TCP to
PERsim's in-process FIL bus. No kernel driver is needed.

This is not emulation of the carrier board's MCP251XFD SPI hardware. An
application that depends on device registers, interrupts, device-tree layout,
or hardware timestamping needs a different device model.

## Host mode

```sh
./build/persim --external-tv
./build/persim-headless --external-tv --duration 30 --scenario acceleration
```

The listener binds **127.0.0.1:29500**. `--tv-port PORT` selects another port and
implies external-TV mode. It accepts one TCP peer and has no authentication;
keep it local. The STM32 torque-vector board is removed, while dashboard, main
module, and both drivelines still run in FIL. The external application owns its
`vcu_settings` and `vcu_torque_request`; the main module still emits AMK SET
messages. **Never run two TV controllers on the same VCAN.**

The bridge publishes plant/firmware CAN traffic and admits guest traffic at the
next control boundary. A guest must not publish competing pedal, BMS, AMK
feedback, or other plant-owned reports. This is a raw CAN link, not a
transmitter allowlist.

## Timing and limits

The guest and QEMU run in wall time; PERsim paces external-TV sessions to wall
time. TCP and guest scheduling add nondeterministic latency. Guest frames are
applied at the next firmware boundary, not at a guest timestamp.

There is **no VM/FIL instruction lockstep**, virtual-clock negotiation, replay
guarantee, or hardware timing validation. Pause/resume or reset of PERsim does
not reset Linux or the TV application.

On disconnect, queued transport bytes are discarded and status becomes visible.
The firmware's stale-TV policy may fall back to its direct pedal map; disconnect
is not inherently an emergency stop. AMK watchdogs still protect against stale
main-module commands.

## Guest client

The client uses only Linux/POSIX APIs:

```sh
$CC -std=c11 -O2 -Wall -Wextra -Wpedantic guest/vcan_bridge.c -o persim-vcan
# On Linux, this project also provides:
cmake --build build --target persim-vcan
```

Cross-build with the image's Yocto SDK for AArch64; a macOS binary cannot run in
the guest. Map VCAN, optionally MCAN and SCAN:

```sh
./persim-vcan 192.168.50.1 29500 vcan0:0
./persim-vcan 192.168.50.1 29500 vcan0:0 vcan1:1 vcan3:3
```

The client never creates interfaces. Unmapped bus traffic is ignored, and
SocketCAN does not echo a socket's own transmissions. Only standard classic
data frames are supported; extended, RTR, error, and FD frames are rejected.

## QEMU networking

Use the independent image profile and its matching kernel, as described by the
[Yoctocoupler project](https://github.com/ronakpjain/yoctocoupler). With
`YOCTOCOUPLER` pointing at a local checkout:

```sh
"$YOCTOCOUPLER/scripts/run-image-in-qemu.sh" \
  --ssh-forward 2222 --kernel /path/to/Image /path/to/image.wic.bz2
```

The runner adds a user-network IGB NIC on `192.168.50.0/24`; guest
`192.168.50.2` reaches the host at `192.168.50.1`. Without networking the client
cannot reach the listener. No image, kernel, SSH configuration, or Yoctocoupler
source is modified by PERsim. No TV algorithm is bundled; deploy the actual CM5
application separately.

`guest/yocto/` contains an optional build-only Scarthgap layer with a
`persim-vcan` recipe. Add it to a development image's `IMAGE_INSTALL`; it does
not auto-start a service. The recipe is marked CLOSED because no redistribution
license has been assigned, and it has not been validated by a full BitBake build
in this macOS workspace.

## PCAN wire format

Each TCP record is exactly **20 bytes**, with no native struct padding:

| Offset | Bytes | Meaning |
|---:|---:|---|
| 0 | 4 | ASCII `PCAN` |
| 4 | 1 | Version, exactly 1 |
| 5 | 1 | PER bus: VCAN=0, MCAN=1, SCAN=3 |
| 6 | 1 | Data length 0..8 |
| 7 | 1 | Flags, must be zero |
| 8 | 4 | Standard arbitration ID, little-endian |
| 12 | 8 | Data bytes, zero padded |

Receivers must accumulate partial records and handle concatenated records.
Unknown magic/version/flags/bus, an ID above 0x7ff, or length above 8 is a hard
error, not a resynchronization point. The host buffers at most 1 MiB of outgoing
data; the guest buffers 64 KiB of TCP output and 256 frames per interface.
Backpressure overflow disconnects explicitly. A new connection starts without
historical replay. This PCAN transport is not FIL's FILN stdio protocol: PERsim
links directly to FIL's C++ world and attaches virtual CAN nodes.
