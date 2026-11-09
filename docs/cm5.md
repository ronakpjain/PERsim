# Virtualized CM5 TV and the CAN gateway {#cm5_bridge}

@tableofcontents

## Why a userspace client, not a kernel driver

The Yoctocoupler QEMU profile already enables Linux SocketCAN and brings up
`vcan0`. The TV application can use ordinary `PF_CAN/SOCK_RAW/CAN_RAW` sockets.
The supplied `guest/vcan_bridge.c` forwards those frames over TCP to PERsim's
in-process FIL CAN bus. No custom kernel driver is necessary for this
software-only interface.

This is not emulation of the carrier's MCP251XFD SPI CAN hardware. A program
that depends on that device's registers, IRQ behavior, device-tree layout, or
hardware timestamping needs a different device-model approach. A SocketCAN
application generally does not need those implementation details.

## Host mode and single-controller rule

```sh
./build/persim --external-tv
# or, paced headless:
./build/persim-headless --external-tv --duration 30 --scenario acceleration
```

The listener binds **127.0.0.1:29500** by default. `--tv-port PORT` chooses a
different port and implies external-TV mode. It accepts one TCP peer. There
is no authentication; keep it local and never expose it on a public interface.

The STM32 torque-vector board is removed in this mode. Dashboard, main module,
and both drivelines still run in FIL. The external application owns its
`vcu_settings` and `vcu_torque_request` output; the main module still owns the
AMK SET messages. **Do not run two TV controllers on the same VCAN.**

The bridge publishes plant and firmware CAN traffic to the guest and admits
guest traffic into the matching virtual bus at the next control boundary.
A guest must not publish competing pedal, BMS, AMK feedback, or other
plant-owned sensor reports. This is a raw simulated CAN link, not a sandbox
enforcing a transmitter allowlist.

## Timing contract

The Linux guest and QEMU run in wall time. PERsim paces external-TV sessions
to wall time, but a slow firmware/host can still fall behind. TCP transport
and guest scheduling add nondeterministic latency. Guest requests are applied
at the next firmware control boundary, not at a guest-provided timestamp.

There is **no VM/FIL instruction lockstep**, virtual-clock negotiation,
cross-VM replay guarantee, or real-hardware timing validation. Pause/resume
or reset of PERsim does not reset Linux or the TV application; reset that app
separately when its controller state must restart.

When the peer disconnects, its queued transport bytes are discarded and
connection status becomes visible. The main firmware's own stale-TV policy
may use its direct pedal mapping instead of TV; disconnect is not inherently
an emergency stop. AMK command watchdogs independently protect against
stale main-module inverter commands.

## Build the Linux guest client

The C client uses only Linux/POSIX APIs and libc. On a Linux host or inside a
suitable AArch64 SDK environment:

```sh
$CC -std=c11 -O2 -Wall -Wextra -Wpedantic guest/vcan_bridge.c -o persim-vcan
# Or, on Linux through this project's CMake:
cmake --build build --target persim-vcan
```

For an AArch64 Yoctocoupler image, use the image's Yocto SDK compiler/sysroot.
A macOS executable cannot be copied into the guest. The minimal production
image does not contain a compiler, Python, or CAN diagnostic utilities.
Cross-build or include the optional recipe below.

Run the client in the guest, mapping VCAN only:

```sh
./persim-vcan 192.168.50.1 29500 vcan0:0
```

Optional additional interfaces can map MCAN and SCAN on the same connection:

```sh
ip link add dev vcan1 type vcan
ip link set vcan1 up
ip link add dev vcan3 type vcan
ip link set vcan3 up
./persim-vcan 192.168.50.1 29500 vcan0:0 vcan1:1 vcan3:3
```

The client never creates/configures interfaces itself. Those optional `ip`
commands require appropriate privileges and the `iproute2` utility, which is
not assumed to be installed in the minimal image. Unmapped bus traffic is
intentionally ignored. SocketCAN does not return a socket's own transmissions
by default, preventing normal bridge echo.

Only standard classic data frames are supported. Extended, RTR, CAN error, and
FD frames are rejected; the gateway must not reinterpret them as standard data.

## Yoctocoupler QEMU networking

Use the independent QEMU image profile, its matching kernel, and an authorized
root SSH key, as described by `../yoctocoupler/README.md`:

```sh
../yoctocoupler/scripts/run-image-in-qemu.sh \
  --ssh-forward 2222 --kernel /path/to/qemu-profile/Image /path/to/qemu-profile/image.wic.bz2
```

That runner adds a user-network IGB NIC on `192.168.50.0/24`; guest
`192.168.50.2` reaches the emulated host at `192.168.50.1`.
The QEMU profile includes the IGB and VCAN modules. The runner's
`--ssh-forward` also enables that NIC; without networking the client cannot
reach the host listener. Copy a target-built client over the forwarded SSH
port using the image's configured key access.

No image, SSH configuration, kernel, or Yoctocoupler source is modified by
PERsim. The bridge and VM do not bundle a torque-vectoring algorithm: deploy
the actual CM5 application separately.

## Optional Yocto layer

`guest/yocto/` is a small optional Scarthgap layer containing a build-only
`persim-vcan` recipe. Add that layer to the **development/QEMU** build and add
`persim-vcan` to its `IMAGE_INSTALL`. It compiles the same C file with the
target compiler; it does not auto-start a service or change the production
image. The recipe is marked CLOSED because no redistribution license has
been assigned to this prototype. It has not been validated by a full
BitBake/image build in this macOS workspace.

## PCAN wire format

Each TCP record is exactly **20 bytes**, with no native struct padding:

| Offset | Bytes | Meaning |
|---:|---:|---|
| 0 | 4 | ASCII `PCAN` |
| 4 | 1 | Version, exactly 1 |
| 5 | 1 | PER bus ID: VCAN=0, MCAN=1, SCAN=3 |
| 6 | 1 | Data length 0..8 |
| 7 | 1 | Flags, must be zero |
| 8 | 4 | Standard arbitration ID, unsigned little-endian |
| 12 | 8 | Data bytes, unused bytes padded |

A receiver must accumulate partial records and handle multiple concatenated
records. Unknown magic/version/flags/bus, an ID above 0x7ff, or length above 8
is an error, not an opportunity to resynchronize. The host buffers at most
1 MiB of outgoing data; the guest buffers 64 KiB of TCP output and 256 CAN
frames per mapped interface. Backpressure overflow disconnects explicitly
rather than silently dropping queued CAN frames. A new connection starts
without historical frame replay.

This PCAN transport is **not FIL's FILN stdio service protocol**. PERsim links
directly to FIL's C++ world and attaches virtual CAN nodes.
