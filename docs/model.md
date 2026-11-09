# Physical model and parameter provenance {#vehicle_model}

@tableofcontents

## Coordinates, state, and units

ISO 8855: **+X forward, +Y left, +Z up**. Dynamics use SI metres, seconds,
kilograms, newtons, radians, and newton-metres. Orientation maps body to world;
translation velocity is world-frame and angular velocity is body-frame. Raylib
uses the separate handedness-preserving conversion `(x,y,z) → (x,z,-y)`.

There are **14 mechanical degrees of freedom**: six chassis coordinates,
four source suspension bump coordinates, and four wheel spins. A unit quaternion
does not add a fifteenth DOF. Steering rack motion is a prescribed, rate-limited
actuator. Wheel order is **FR/FL/RL/RR**, unlike the sibling's FR/FL/RR/RL.

## Source linkage and coupled mass matrix

Let \f$p,R,\omega\f$ be the body reference position, rotation, and body angular
velocity. The source suspension splines provide \f$r_i(q_i,u)\f$, the wheel
centre relative to the body, for bump \f$q_i\f$ and rack offset \f$u\f$.

\f[
p_i=p+Rr_i,\quad
v_i=v+R(\omega\times r_i+r_{i,q}\dot q_i+r_{i,u}\dot u).
\f]

For generalized velocity
\f$\nu=[v,\omega,\dot q_1..\dot q_4,\dot\theta_1..\dot\theta_4]\f$,
the hub Jacobian is

\f[
J_i=[I,\;-R[r_i]_\times,\;Rr_{i,q}e_i^T,\;0].
\f]

Its known acceleration bias is

\f[
b_i=R\left(
\omega\times(\omega\times r_i)
+2\omega\times(r_{i,q}\dot q_i+r_{i,u}\dot u)
+r_{i,qq}\dot q_i^2+2r_{i,qu}\dot q_i\dot u
+r_{i,uu}\dot u^2+r_{i,u}\ddot u
\right).
\f]

The mass matrix sums sprung-body translation/inertia, unsprung point masses
\f$m_iJ_i^TJ_i\f$, and wheel axial-inertia outer products. A positive-definite
LDLT solve computes acceleration. Wheel axis direction comes from source
toe/camber and its derivatives; gyroscopic and axis-rate terms enter the same
coupled solve. Motor/brake torque acts on relative spin, with body reaction
through this coupling rather than a separately invented body torque.

Only yaw inertia is sourced. Estimated total roll/pitch inertia is reduced by
nominal unsprung mass and axial wheel-spin contributions to obtain sprung
inertia. This is not a full unsprung rigid-body inertia model. Upright axial
twist and full transverse wheel/upright rotational inertia are not modeled.

## Actual shock motion ratios

The source SpringDamper and FRONT_COILOVER/REAR_COILOVER presets evaluate force
along increasing shock length \f$L_i(q_i,u)\f$:

\f[
F_{L_i}=-k_i(L_i-L_{ref,i})-c_i\dot L_i,\qquad
\dot L_i=L_{i,q}\dot q_i+L_{i,u}\dot u,\qquad
Q_{q_i}=L_{i,q}F_{L_i}.
\f]

The source front/rear reference lengths are 0.1955/0.1970 m. Extension and
compression stops use 0.200 m maximum length and 0.057 m stroke. Additional
soft travel stops bound the available spline domain. Reset runs a damped
physical relaxation before time zero; body ride height is determined by the
actual shock/contact system, not fixed to the nominal CG-height parameter.

## Contact and source tire forces

Contact uses the road normal projected into the wheel plane, so camber affects
the effective rolling radius and rim contact offset. Source VerticalTire
evaluates unilateral normal force from compression and compression velocity.
Its explicit parameter override is 180 kN/m / 650 N·s/m; the sibling prototype
defaults are 15 kN/m / zero damping.

Longitudinal slip compares source Wheel::surfaceSpeed with the contact-base
velocity. Its denominator is regularized to at least **1 m/s**. Lateral slip is
`atan2(-v_lateral, max(abs(v_forward), 1 m/s))`. This keeps reverse/near-zero
operation defined rather than directly using the planar RigidCorner slip
formula.

The default tangential model is the actual source ReducedMFTire:

\f[
F=\mu F_z\sin\left(C\arctan\frac{Ks}{C\mu F_z}\right).
\f]

Source Tire::getForce saturates **longitudinal force first**, then clips lateral
force to the remaining friction-circle capacity:

\f[
|F_x|\le\mu F_z,\qquad |F_y|\le\sqrt{\max(0,(\mu F_z)^2-F_x^2)}.
\f]

The source TanhTire and LinearTire are alternatives selected by
`--tire tanh` / `--tire linear`; they use the same source combined-slip policy.
The road multiplier sets the source tire's protected coefficient through a
small derived adapter; it does not scale the normal contact force.

There is no tire relaxation length, aligning moment, temperature/wear model,
or measured load sensitivity. Saturating drive torque can consume lateral
grip: there is no hidden traction force or stability correction in the plant.

Ground forces act through hub-path Jacobians and ground-level moment arms.
Axle-direction changes also map contact moments into suspension generalized
force. Mechanical brakes oppose existing spin; rolling resistance opposes
wheel spin. These brake/resistance laws remain PERsim estimates because the
source mechanical-brake model is empty.

## Steering and frontend controls

Nominal mean front-wheel steering maps monotonically to source rack offset.
Source wheel orientation supplies actual steering, bump steer, and camber;
there is no separate synthetic Ackermann approximation on top of the linkage.
Steering lock is limited to the source rack table, currently about ±7.9°.

Keyboard driving additionally ramps input and bounds its angle with the
estimated kinematic envelope

\f[
\delta_{key}=\min\left(
\delta_{max},
\arctan\frac{L\,0.85\mu(g+F_{aero}/m)}{\max(v_{XY}^2,1\;\mathrm{m^2/s^2})}
\right).
\f]

Keyboard throttle rises at 1.6/s and releases at 5/s. Normalized steering
slews at 2.5/s under input and recenters at 3.5/s; braking is immediate.
Gamepad axes and scripted/headless inputs bypass this frontend aid. The
physical tire coefficient remains the source default 1.0.

## Aero, integration, and terrain

Actual source Aero::getForce and COP determine aerodynamic loads. As in that
model, reverse forward-speed input is clamped to zero. Four additional
unilateral belly points prevent chassis ground penetration; they are not a
general collision engine.

Vehicle::step uses normalized quaternion updates and semi-implicit Euler.
The maximum substep is **0.5 ms**. Session splits each 5 ms control interval
at delivered AMK command times. Physics, sensor shock reports, and rendered
wheel poses share source linkage geometry.

The proving ground contains:
- a 65 mm smooth bump at X=18..22 m;
- a 0.4 m ramp/crest at X=35..53 m;
- ripples at X=58..80 m;
- all three in the smoothly edged Y=-14..-6 m strip;
- a 0.45 friction multiplier at X=25..45, Y=6..14 m.

Cones, paint, and procedural body geometry are not collision objects.

## Source parameters and remaining estimates

Parameters reads live source presets: PER27, PER27_WHEEL, DEFAULT_TIRE,
FRONT_COILOVER/REAR_COILOVER, AE27, AMK_DD5, and DT27 efficiency.
The gearbox ratio remains **12.51** to match firmware, not DT27's 13.0.

Estimates include roll/pitch inertia (38/110 kg·m²), total unsprung mass (32 kg),
vertical tire override, soft-stop stiffness, brake capacity/bias, steering slew
and sensor ratio, rolling resistance, battery/inverter thermal/electrical
parameters, sensor mount locations, and optional noise levels. Source data
itself is not a claim of hardware calibration.

See @ref reused_models for an exact source-to-runtime inventory and
@ref validation before interpreting results.
