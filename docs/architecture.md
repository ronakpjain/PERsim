# Architecture, firmware devices, and timing {#timing_model}

@tableofcontents

## Ownership and data flow

```text
keyboard / scripted driver
        |
        +-> dashboard ADC + start GPIO -> real dashboard firmware
        +-> physical steering actuator -> LWS CAN
        +-> physical mechanical brakes
                                              |
vehicle / inverter / battery sensors --------> FIL firmware world
                                              |
                     main-module AMK SET CAN --+
                              |
                 timed inverter device response
                              |
                    14-DOF physical plant
                              |
                     next sensor observation
```

Session owns one Vehicle, one Powertrain, and (unless explicitly standalone)
one FilBridge. The headless frontend steps the session directly. The raylib
frontend uses one POSIX worker with an **8 MiB stack**; FIL's memory-map
construction can exceed macOS's default 512 KiB pthread stack. The render
thread publishes controls and copies Snapshot values under a mutex. It never
calls FIL or modifies physics.

Pause stops simulated time. Reset reconstructs both plant and firmware;
the viewer creates a numbered CSV file for each subsequent run. A slow
renderer does not discard physics time. A slow firmware backend slows the
simulation rather than silently jumping forward.

## Causal control intervals

1. Sample the physical state at shared time T.
2. Apply ADC/GPIO inputs and queue due external-device CAN reports.
3. Advance FIL for a bounded 5 ms shared-time interval.
4. Collect AMK frames at **CAN EOF/delivery timestamps**, not admission or SOF.
5. Advance the physical plant up to each delivery with the old command.
6. Apply that command to its inverter; continue through the remaining interval.
7. Publish the observation and use it for the next input sample.

Inputs are zero-order held across the firmware slice. The resulting maximum
feedback coupling delay is one control interval, in addition to modeled bus,
firmware task, and device delays. Commands are not applied retroactively to
the start of a slice. Physics substeps never exceed 0.5 ms.

The firmware shared clock may overshoot a deadline within a bounded window;
PERsim rejects an overrun above 100 µs and advances physics to the actual end
time. FIL's current parallel scheduler itself uses bounded-skew owner slices,
not cycle-exact multi-board lockstep. Strict MMIO and ADC decimation 1 are
used. Guarded RAM-capsule replay is disabled because cycle-exact real-firmware
network replay is not established. This does not make FIL cycle-accurate hardware.

## Wiring generation and firmware ownership

`tools/generate_network.py` reads the monorepo's
`generators/configs/system/bus_configs.json` and current node files. It writes
build-local FIL board/network JSON referencing the actual monorepo ELF images
and FIL's STM32G474 MCU definition. It does **not** reuse FIL's older example
PER controller assignments.

| Board | Current controller connections |
|---|---|
| Main module | FDCAN1 → VCAN (0), FDCAN2 → MCAN (1) |
| Dashboard | FDCAN2 → VCAN (0), FDCAN3 → SCAN (3) |
| Torque vector | FDCAN2 → VCAN (0), FDCAN1 → SCAN (3) |
| Front / rear driveline | FDCAN1 → SCAN (3) |

The build consumes the versioned SuperDBC JSON to generate selected signal
descriptors. It implements both Intel and Motorola bit order; there is no
second hand-maintained list of signal offsets. If there is not exactly one
versioned JSON, select it with `PERSIM_SUPERDBC`.

The default network has dashboard, main module, TV, and both drivelines.
**a_box is deliberately absent**: the plant owns the pack/BMS CAN reports,
rather than competing with an emulated battery-monitor firmware/SPI chain.
CCAN and charging hardware are not modeled. User network configs containing
a_box or timed stimulus scripts are rejected. In external-TV mode only the
STM32 torque-vector board is removed; main-module safety and AMK command
generation remain real firmware.

## Sensor/device map

| Physical quantity | Interface | Nominal update |
|---|---|---:|
| APPS 1 / APPS 2 | dashboard ADC1 channels 8 / 7 | each 5 ms slice |
| Regen / brake (harness-swapped firmware inputs) | dashboard ADC1 channels 2 / 9 | 5 ms |
| Start button | dashboard PB14, active-low | 5 ms |
| SDC mux output | main-module PB9 held high, all mux selections closed | initial constant |
| Precharge incomplete | main-module PC4, high until modeled HV bus reaches 95% terminal voltage | 5 ms |
| Shockpot channels | source linkage shock lengths → front/rear ADC3 ch5, ADC4 ch3; front harness swap included | 5 ms |
| Steering-wheel angle / validity | LWS_Standard on VCAN | 10 ms |
| Pack voltage/current | pack_analog on VCAN | 10 ms |
| Gyro / accelerometer | IZZE_angular_rate / IZZE_acceleration, VCAN | 50 ms |
| Pack summary | pack_bms on VCAN | 200 ms |
| Actual rotor speed/torque | AMK CRIT on MCAN | 15 ms |
| Inverter status/DC voltage/temperatures | AMK INFO/TEMPS on MCAN | 500 ms |
| GPS NAV-PVT + RELPOSNED | TV USART3 byte stream | 200 ms |
| User TV request | vcu_driver_request, AUTOCROSS mode, 50% gains | 200 ms |

Scheduling occurs at control boundaries; these are nominal report periods,
not a promise of oscillator-exact external-device clocks. The actual physical
steering angle, not an instantaneous key press, drives the LWS report.
The plant uses left-positive steering. LWS therefore reports positive degrees
for a physical left turn: `control_loop.c` negates that signal to produce the
VCU's **right-positive** `ST_RAW`, as specified by `vcu/vcu_interface.md`.
The VCU also negates body-Z gyro to obtain right-positive yaw rate.

Pedal calibration reproduces `dashboard/pedals/pedals.c`: APPS1 200..710,
inverted APPS2 3320..3470, regen 2550..3000, and brake 450..1300. A brake
demand drives both the mechanical brakes and the two firmware brake/regen
inputs. This is a simulator input mapping, not a calibrated pedal hydraulic model.

The source IMU samples use a RigidMount3D with the exact mounting rotation in
`torque_vector/sensors/imu.c`. Encoded gyro is degrees/s; acceleration is g.
The source model evaluates conventional specific force `a-g`. The IZZE CAN
encoder negates only this accelerometer result to match the captured device's
`g-a` convention: an upright stationary car reports **-1 g on body Z** after
firmware transformation. Gyro is not negated by this adaptation. Plant truth
and CSV `ax/ay/az` remain conventional body specific force, with +g on upright
stationary Z in m/s²; they are not literal IZZE report values. Optional noise
is applied by the source sensor model before wire quantization. See
@ref captured_data for the sign evidence and historical report-rate differences.

The source GPS supplies mounted rover position/velocity and base-to-rover
baseline, with optional seeded noise. UBX encoding uses a fixed synthetic
georeference and UTC epoch: world X maps north and Y maps west. The 100-byte
NAV-PVT and 72-byte NAV-RELPOSNED packets match the firmware's cyclic DMA buffer.
Satellites, RF, ambiguity resolution, and PPS are not modeled.

## Startup and failure behavior

The modeled pack precharges its HV bus, then AMK status responds to real
main-module SET flags. Automatic startup holds the brake, pulses PB14 at
1.5..1.65 s, and lets the real vehicle FSM run its buzzer before entering R2D.
Driver throttle is suppressed before R2D; no firmware variables are patched.

Missing images, unsupported MMIO/instructions surfaced by FIL, non-monotonic
actuator events, queue admission failure, invalid dynamics, and time overruns
stop the session visibly. There is no automatic standalone fallback. A guest
TCP disconnect is separately visible; firmware's own stale-TV behavior may
fall back to its direct pedal map. Do not mistake a missing TV connection for
a guaranteed all-vehicle shutdown.
