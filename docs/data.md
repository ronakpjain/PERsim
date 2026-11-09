# Historical captures

`data/PER26/` holds historical PER26 logs from June and September 2026. They
corroborate device conventions and expose implausible assumptions; they do **not**
establish PER27 mass, inertia, tire, aero, suspension, steering, battery, or
thermal parameters. Raw captures are large and are not tracked by git.

The initial review streamed five individual parsed captures: about
**1.72 GB / 4.18M millisecond-grid rows / 69.6 min**, including stationary
periods. Raw logs, `parsed.old/`, corrected exports, and combined exports are
not independent experiments and must not be pooled as extra runs.

| Label | Capture (under `data/PER26/`) | DAQ span |
|---|---|---:|
| June torque | `2026-06-14/out_005_2026_06_14__09_51_49_torque_window.csv` | 521.916 s |
| September laps | `2026-09-05/endurance_attempt/parsed/couple_laps_004_2026-09-05T22-14-14.893Z.csv` | 523.381 s |
| No-cooling label | `2026-09-06/1_no_cooling/parsed/no_cooling_000_2026-09-06T18-34-51.252Z.csv` | 1804.113 s |
| Regen | `2026-09-06/2_regen/parsed/regen_002_2026-09-06T20-21-44.349Z.csv` | 544.039 s |
| Later laps, corrected | `2026-09-06/3_happy_laps_endurance_regen/parsed/vbatt_fixed_014_2026-09-07T00-53-20.966Z.csv` | 781.586 s |

The supplied `amk_actual_torque.png` is in **percent**, not Nm.

## Corroborated conventions

- **AMK report rates.** CRIT had a 15 ms median; INFO/TEMPS 500 ms. These match
  the simulator's device schedules. June main-module SET frames arrived about
  every 16 ms, but the simulator runs real firmware and does not impose that
  period itself.
- **Torque is nominal-torque percent.** June inverter C reached 95.6%, about
  9.37 Nm at the firmware's 9.8 Nm nominal rating. September rear channels
  reached about 132%, so 100% is not a hardware ceiling. The decoded positive
  limit is usually 214 (CSV unit label says Nm) but means 214% ≈ 20.97 Nm.
- **Mounting and accelerometer sign.** The firmware rotation
  `Rz(-pi/2) Rx(2.2689280276)` reproduces processed IMU vectors from raw ones
  (RMS differences near 0.0055 g and 0.055°/s). Stationary processed body Z is
  about **-1 g**, and lateral acceleration correlates at **-0.978 / -0.989**
  with `speed * yaw_rate / g`. The IZZE device therefore reports `g-a`.
  PERsim negates only the accelerometer at the CAN boundary; gyro, source
  models, physical forces, and CSV specific force keep conventional `a-g`.
- **Speed units.** Logged `wheel_speeds` follow rotor rpm, not wheel rpm. With a
  0.2 m assumed radius, near-straight samples give a ratio of 12.41–12.59,
  compatible with the firmware's **12.51** reduction. Spread is too large to
  identify the ratio precisely; keep the firmware value.
- **Regeneration.** Of 3,079 pack-current samples with fresh motor data and
  estimated shaft power below -1 kW, **98.8%** had negative pack current
  (10th/50th/90th percentiles about -8.8 / -6.4 / -3.5 A). This supports
  positive-discharge/negative-charge polarity, not an efficiency fit.
- **Operating points.** June GPS speed reached 23.497 m/s (84.6 km/h); the
  no-cooling-labelled run reached 96.0 °C on inverter A. The label alone is not
  a verified cooling command.

## Data-quality limits

- CSVs have **seven header rows**: message names row 3, signal names row 5,
  units row 7, data from row 8. Most rows are sparse asynchronous updates; a
  1 ms grid is not a 1 kHz sample. Never zero-fill or forward-fill across
  arbitrary gaps.
- Enum cells can be labels such as `ready2drive (5)`; decode their numeric value.
- `2026_09_05_endurance_attempt_combined.csv` has backward DAQ-time jumps and
  is rejected for temporal alignment. Use individual captures; sorting
  overlapping timestamps would mix physical times.
- The later corrected capture's steering angle stays near 38.1° despite real
  yaw motion, so it cannot identify steering ratio, center, or lock.
- Rear-right shock telemetry has a pronounced **145 mm** floor, often its
  median. Clipping cannot be separated from mechanics, so do not infer preload
  or damping from it.
- `vbatt_fixed_*` values sit on a roughly 400–500 scale under an `ADC` label;
  preserve export provenance. Some pack reports drop out or disagree with
  inverter DC-link voltage.
- June inverter A reached **519 V**, above PERsim's estimated 380–480 V OCV
  curve. This limits battery-model transferability; do not copy PER26 limits.
- Raw/processed IMU reports were about 4 ms apart, faster than the current
  50 ms IZZE publication. Historical GPS and pack reports also differ. Report
  intervals are not necessarily physical sample periods.
- Spikes, unknown road/cooling conditions, unmatched trajectories, and missing
  corner loads make peak-based friction, aero, inertia, and thermal fitting
  underdetermined. No parameter defaults were tuned from these extrema.

After the wire-polarity adaptation, a temporary native check recovered about
**-1.001 g** on stationary body Z while source specific force stayed
**+9.81 m/s²**. The check program was removed; it is not a test target.

## Reproduce

`tools/analyze_capture.py` streams selected numeric columns with pandas, keeps
original DAQ timestamps, decodes numeric enums, and reports units, counts,
ranges, quantiles, and update intervals. Comparisons use bounded-age backward
as-of alignment, so stale or absent channels remain missing (50 ms for AMK,
60 ms raw IMU, 2 ms processed-axis skew, 15 ms for kinematic sign checks).
These are analysis gates, not identified physical delays.

Requires NumPy, SciPy, and pandas. Pandas is not needed by the simulator, build
generators, or real-time plant.

```sh
python3 tools/analyze_capture.py \
  data/PER26/2026-06-14/out_005_2026_06_14__09_51_49_torque_window.csv \
  data/PER26/2026-09-06/2_regen/parsed/regen_002_2026-09-06T20-21-44.349Z.csv \
  --output build/capture-review.json
```

The utility is read-only, refuses to overwrite an input, does not fit
parameters, and does not replay commands into FIL. It is offline analysis, not
an automated simulator test suite. See [scope and limitations](validation.md).
