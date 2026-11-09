# Architecture and timing

## Ownership

```text
keyboard / scripted driver
        +-> dashboard ADC + start GPIO -> real dashboard firmware
        +-> steering actuator -> LWS CAN
        +-> mechanical brakes
vehicle / inverter / battery sensors -> FIL firmware world
        -> main-module AMK SET CAN -> timed inverter response
        -> 14-DOF physical plant -> next sensor observation
```

`Session` owns one `Vehicle`, one `Powertrain`, and (unless `--standalone`) one
`FilBridge`. Headless steps the session directly. The raylib viewer uses one
POSIX worker with an **8 MiB stack** because FIL's memory map can exceed macOS's
512 KiB pthread default. The render thread exchanges controls and `Snapshot`
values under a mutex; it never calls FIL or touches physics.

Pause stops simulated time. Reset rebuilds plant and firmware. A slow renderer
does not discard physics time, and a slow firmware backend slows the simulation
instead of silently jumping forward.

## Causal control interval

1. Sample the physical state at shared time T.
2. Apply ADC/GPIO inputs and queue due external-device CAN reports.
3. Advance FIL for a bounded 5 ms shared-time interval.
4. Collect AMK frames at **CAN EOF/delivery timestamps**.
5. Advance the plant to each delivery using the previous command.
6. Apply that command to its inverter; continue through the interval.
7. Publish the observation for the next input sample.

Inputs are zero-order held across the slice, so feedback coupling adds at most
one control interval plus bus, task, and device delays. Commands are never
applied retroactively. Physics substeps never exceed 0.5 ms.

The firmware shared clock may overshoot a deadline; PERsim rejects an overrun
above 100 µs and advances physics to the actual end time. FIL's parallel
scheduler uses bounded-skew owner slices, not cycle-exact multi-board lockstep.
MMIO is strict and ADC decimation is 1. Guarded RAM-capsule replay is disabled.
FIL is not cycle-accurate hardware.

## Wiring and firmware ownership

`tools/generate_network.py` reads the monorepo's
`generators/configs/system/bus_configs.json` and current node files, then writes
build-local FIL board/network JSON pointing at the real ELF images and FIL's
STM32G474 MCU definition. It does not reuse FIL's older example assignments.

| Board | Controller connections |
|---|---|
| Main module | FDCAN1 → VCAN (0), FDCAN2 → MCAN (1) |
| Dashboard | FDCAN2 → VCAN (0), FDCAN3 → SCAN (3) |
| Torque vector | FDCAN2 → VCAN (0), FDCAN1 → SCAN (3) |
| Front / rear driveline | FDCAN1 → SCAN (3) |

The build generates signal descriptors from the versioned SuperDBC JSON,
including Intel and Motorola bit order. Select a specific file with
`PERSIM_SUPERDBC` if the directory does not contain exactly one.

The default network has dashboard, main module, TV, and both drivelines.
**a_box is absent** because the plant owns pack/BMS reports. CCAN and charging
hardware are not modeled, and configs containing a_box or timed stimuli are
rejected. External-TV mode removes only the STM32 torque-vector board.

## Devices

| Quantity | Interface | Period |
|---|---|---:|
| APPS 1 / APPS 2 | dashboard ADC1 ch 8 / 7 | 5 ms |
| Regen / brake (harness-swapped) | dashboard ADC1 ch 2 / 9 | 5 ms |
| Start button | dashboard PB14, active-low | 5 ms |
| SDC mux output | main-module PB9 high, all selections closed | constant |
| Precharge incomplete | main-module PC4 until 95% HV bus | 5 ms |
| Shockpots | source shock lengths → ADC3 ch5 / ADC4 ch3 | 5 ms |
| Steering angle / validity | LWS_Standard, VCAN | 10 ms |
| Pack voltage / current | pack_analog, VCAN | 10 ms |
| Gyro / accelerometer | IZZE angular rate / acceleration, VCAN | 50 ms |
| Pack summary | pack_bms, VCAN | 200 ms |
| Rotor speed / torque | AMK CRIT, MCAN | 15 ms |
| Inverter status / DC / temps | AMK INFO/TEMPS, MCAN | 500 ms |
| GPS NAV-PVT + RELPOSNED | TV USART3 | 200 ms |
| TV driver request | vcu_driver_request, AUTOCROSS, 50% gains | 200 ms |

These are nominal report periods, not oscillator-exact device clocks.

Pedal encoding reproduces `dashboard/pedals/pedals.c`: APPS1 200..710,
inverted APPS2 3320..3470, regen 2550..3000, brake 450..1300. Brake demand
drives both the mechanical brakes and the firmware regen/brake inputs. This is
an input mapping, not a hydraulic calibration.

The plant is left-positive. LWS reports positive degrees for a physical left
turn, and `control_loop.c` negates it into the VCU's right-positive `ST_RAW`.
The VCU also negates body-Z gyro for right-positive yaw rate.

Source IMU samples use the firmware's exact mounting rotation. The source model
produces conventional specific force `a-g`; the IZZE encoder negates only the
accelerometer so an upright stationary car reports **-1 g body Z** after firmware
transformation. Gyro polarity, plant truth, and CSV `ax/ay/az` remain
conventional +9.81 m/s² specific force. Optional noise is applied before CAN
quantization. See [historical captures](data.md).

GPS uses mounted rover position/velocity and base-to-rover baseline with an
optional seeded-noise model. UBX encoding uses a fixed synthetic georeference
(world X north, Y west) and UTC epoch. The 100-byte NAV-PVT and 72-byte
NAV-RELPOSNED packets match the firmware DMA buffer. Satellites, RF,
ambiguity resolution, and PPS are not modeled.

## Startup and failure

The pack precharges, then AMK status follows real main-module SET flags.
Automatic startup holds the brake, pulses PB14 at 1.5..1.65 s, and lets the
vehicle FSM run its buzzer before R2D. Throttle is suppressed before R2D; no
firmware variables are patched.

Missing images, unsupported MMIO/instructions, non-monotonic events, queue
failures, invalid dynamics, and time overruns stop the session visibly. There is
no automatic standalone fallback. A guest TCP disconnect is visible, but the
firmware's stale-TV policy may fall back to its direct pedal map; do not treat a
missing TV connection as an emergency stop.
