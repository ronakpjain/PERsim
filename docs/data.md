# Historical telemetry evidence {#captured_data}

@tableofcontents

## Scope and provenance

`data/PER26/` contains historical PER26 captures dated June and September 2026.
PERsim uses the sibling repository's **PER27** presets. These logs can
corroborate device conventions and expose implausible assumptions, but do not
establish PER27 mass, inertia, tire coefficients, aero, suspension calibration,
steering ratio, usable battery energy, or thermal constants.

The initial review streamed five individual parsed captures, approximately
**1.72 GB / 4,175,040 millisecond-grid rows / 69.6 minutes** including stationary
periods. This is not an exhaustive review of every raw/parsed file. Raw logs,
`parsed.old/`, corrected exports, and combined exports are not independent
experiments and must not be pooled as additional vehicle runs.

Paths below are relative to `data/PER26/`:

| Label | Capture | DAQ span |
|---|---|---:|
| June torque | `2026-06-14/out_005_2026_06_14__09_51_49_torque_window.csv` | 521.916 s |
| September laps | `2026-09-05/endurance_attempt/parsed/couple_laps_004_2026-09-05T22-14-14.893Z.csv` | 523.381 s |
| No-cooling-labelled run | `2026-09-06/1_no_cooling/parsed/no_cooling_000_2026-09-06T18-34-51.252Z.csv` | 1804.113 s |
| Regen | `2026-09-06/2_regen/parsed/regen_002_2026-09-06T20-21-44.349Z.csv` | 544.039 s |
| Later laps, corrected export | `2026-09-06/3_happy_laps_endurance_regen/parsed/vbatt_fixed_014_2026-09-07T00-53-20.966Z.csv` | 781.586 s |

The supplied `amk_actual_torque.png` depicts the June capture in **percent**, not
Nm. Its ordinate must not be used directly as physical shaft torque.

## Corroborated interfaces

### AMK feedback and torque representation

The captures' AMK CRIT messages have a median update interval of **15 ms**.
INFO and TEMPS have **500 ms** medians. Those match PERsim's device schedules.
The June main-module SET reports arrive about every **16 ms**; the simulator
runs the actual firmware rather than imposing that command period itself.

Actual torque metadata explicitly says nominal-torque percent. June inverter C
reaches **95.6%**, approximately **9.37 Nm** using the firmware's 9.8 Nm nominal
rating, not 95.6 Nm. The September-laps rear channels reach approximately 132%,
so 100% is also not an absolute hardware torque ceiling.

The decoded positive-limit word is usually **214**, despite its CSV `Nm` unit
label. The existing AMK/firmware interpretation remains nominal-torque percent:
214% corresponds to about 20.97 Nm, and the negative limit -50 to -4.90 Nm.
These observations support retaining that interpretation; logs alone do not
independently identify the motor's rated torque or speed envelope.

### Mounting rotation and accelerometer polarity

Applying the existing firmware rotation `Rz(-pi/2) Rx(2.2689280276)` to captured
raw IZZE vectors reproduces the corresponding processed IMU vectors. In the
June capture, per-axis RMS differences are approximately **0.0055–0.0058 g**
and **0.052–0.058 deg/s**, near the observed 0.01 g / 0.1 deg/s quantization.
This corroborates the mounting transform, not a sensor-noise calibration.

The hardware acceleration convention differs from the sibling's conventional
specific-force output:

- With reported GPS speed below 0.1 m/s, processed body Z is approximately
  **-1 g**, not +1 g, in both June and regen captures.
- In moving samples with speed above 5 m/s and absolute yaw rate below
  100 deg/s, the correlation of processed lateral acceleration with
  `speed * yaw_rate / g` is **-0.978** in June and **-0.989** in the regen run.
- A separate smoothed longitudinal comparison also has negative correlation
  with the delayed GPS speed derivative. It is not used to identify latency,
  since GPS filtering, pitch, and sideslip confound that comparison.

Together these support an IZZE accelerometer **`g-a`** report convention. The
source sensor still evaluates specific force **`a-g`**; `sensor_frames()`
negates the accelerometer at the CAN boundary. **Gyro polarity, the mounting
rotation, source models, gravity, physical forces, and CSV specific-force
columns are unchanged.** A stationary upright car therefore retains about
+9.81 m/s² in CSV `az`, while firmware receives about -1 g on processed body Z.
This is a device-interface correction, not a change in vehicle dynamics.

### Speed units and gearing

Despite their historical `wheel_speeds` name, the logged values are on the
rotor-speed scale and closely follow AMK ActualSpeed, not wheel rpm. With an
assumed rolling radius of 0.2 m, near-straight June samples give median
`rpm * 2*pi*radius / (60*GPS_speed)` of **12.41–12.59** across four corners.
That is compatible with the firmware's 12.51 reduction and inconsistent with
treating those numbers as wheel speed.

The spread is substantial and changes between runs. Radius, slip, latency,
position of antennas, and remaining turn geometry confound this ratio. Keep
12.51 from firmware; do not claim that the captures independently measure an
exact reduction or calibrate the tire radius.

### Regeneration and operating ranges

The selected regen run has **3,079** pack-current observations with fresh
four-motor speed/torque reports and estimated summed shaft power below -1 kW.
**98.8%** have negative pack current; current's 10th/50th/90th percentiles are
approximately **-8.8 / -6.4 / -3.5 A**. The capture's minimum reported pack
current is -13.8 A. This supports positive-discharge/negative-charge polarity,
not a fit of inverter efficiency or pack resistance.

June reaches reported GPS speed **23.497 m/s (84.6 km/h)**. In the no-cooling-
labelled run, inverter A's reported motor temperature reaches **96.0 C**.
These are observed operating points, not maximum design capability or thermal
parameter identification. The label alone is not a verified cooling command.

## Data-quality and transfer limits

- CSVs have **seven header rows**; message name, signal name, and units are
  separate rows. Most numeric rows are sparse asynchronous updates. A 1 ms
  row grid does not mean every sensor samples at 1 kHz. Never zero-fill absent
  measurements or forward-fill across arbitrary missing intervals.
- Enum cells can contain labels such as `ready2drive (5)`. Numeric decoding
  must extract their encoded values rather than discard them as missing.
- `2026_09_05_endurance_attempt_combined.csv` contains backward DAQ-time jumps.
  The analysis utility rejects it for temporal alignment. Use individual
  captures, or explicitly reconstruct a trustworthy segmented clock first;
  sorting overlapping DAQ timestamps would mix different physical times.
- The later corrected capture's steering angle stays near **38.1 degrees**
  despite substantial yaw and vehicle motion. It cannot establish steering
  ratio, center, or lock. Earlier steering-wheel excursions also must not be
  substituted for road-wheel angles or extrapolate PER27 linkage splines.
- Rear-right shock telemetry has a pronounced **145 mm** floor, often its
  median. Sensor/calibration clipping cannot be distinguished from mechanical
  behavior from these traces alone. Do not infer spring preload, motion ratio,
  or tire/suspension damping from that floor. Historic sensor length is not
  automatically the PER27 linkage's geometric shock length.
- `vbatt_fixed_*` changes values in a column still labelled `vbatt_adc` / `ADC`;
  values are then on a roughly 400–500 scale rather than raw 12-bit counts.
  Preserve export provenance instead of trusting that unit label blindly.
  Some pack reports have dropouts or disagree with contemporaneous inverter
  DC-link voltage. Neither is a stand-alone OCV/SOC/resistance calibration.
- June inverter A DC-link voltage reaches **519 V**, above PERsim's estimated
  380–480 V OCV curve. This exposes the simplified battery model's limited
  transferability; it does not justify copying PER26 voltage limits into PER27.
- Raw/processed IMU reports are observed around **4 ms** apart, faster than
  PERsim's current **50 ms** IZZE publication. Historical GPS/pack summary
  reports also differ from current interfaces, and may retransmit cached data.
  Report intervals do not necessarily identify physical sampling periods.
  No exact hardware-latency reproduction is claimed.
- Isolated acceleration/yaw spikes, unknown road/cooling conditions, unmatched
  driver trajectories, and absence of independently measured corner loads make
  peak-based friction, aero, inertia, and thermal fitting underdetermined.

No physical parameter defaults were tuned from these historical extrema.
PERsim remains an uncalibrated engineering prototype; see @ref validation.

## Manual simulator follow-up

After the wire-polarity adaptation, a temporary native inspection of emitted
CAN frames recovered approximately **-1.001 g** on stationary body Z using the
firmware mounting matrix, while source specific force remained **+9.81 m/s²**.
A nonzero angular-rate inspection retained gyro polarity within CAN quantization.
The temporary inspection program was removed, not added as a test target.

Real-FIL acceleration (6 s), circle (8 s), and braking (10 s) scenarios completed
their existing optional runtime checks. Acceleration still reached R2D at
4.050 s and 41.579 km/h peak; braking returned the car to rest. The viewer,
headless executable, and Doxygen reference rebuilt successfully. These bounded
manual observations check the integration, not correlation to a recorded lap.

## Reproduce a review

`tools/analyze_capture.py` streams selected numeric columns using pandas,
retains original DAQ timestamps, decodes numeric enums, and reports per-signal
units, event counts, ranges, quantiles, and update intervals. Comparisons use
bounded-age backward-asof alignment; stale or absent channels remain missing.
AMK comparisons allow 50 ms age, raw/processed IMU matching allows 60 ms raw
age and 2 ms processed-axis skew, and kinematic sign checks allow 15 ms IMU
age. These are analysis gates, not identified physical delays.

Requires Python with **NumPy, SciPy, and pandas**. Pandas is optional for offline
capture analysis; it is not needed by the simulator, build generators, or
real-time plant. For example:

```sh
python3 tools/analyze_capture.py \
  data/PER26/2026-06-14/out_005_2026_06_14__09_51_49_torque_window.csv \
  data/PER26/2026-09-06/2_regen/parsed/regen_002_2026-09-06T20-21-44.349Z.csv \
  --output build/capture-review.json
```

The utility reads captures without modifying them and refuses to overwrite an
input with its output. It neither fits/writes Parameters nor replays captured
commands into FIL. It is an offline analysis tool, **not an automated simulator
test suite**. Keep raw and corrected files separate when interpreting results.
