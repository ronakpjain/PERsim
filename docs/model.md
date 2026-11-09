# Physical model {#vehicle_model}

@tableofcontents

PERsim is an uncalibrated engineering prototype. Source presets supply geometry
and component behavior; values marked *estimate* are not measured PER27 data.

## Coordinates, state, and units

ISO 8855: **+X forward, +Y left, +Z up**. Dynamics use SI metres, seconds,
kilograms, newtons, radians, and newton-metres. Orientation maps body to world;
translation velocity is world-frame, angular velocity is body-frame. The raylib
frontend converts `(x,y,z) → (x,z,-y)` to preserve handedness.

The plant has **14 mechanical DOF**: six chassis coordinates, four source
suspension bump coordinates, and four wheel spins. The orientation quaternion
does not add a fifteenth DOF. Steering rack motion is a prescribed, rate-limited
actuator. Wheel order is **FR/FL/RL/RR**.

## Coupled mass matrix

For body position \f$p\f$, rotation \f$R\f$, and body rate \f$\omega\f$, source
suspension splines give the wheel-centre offset \f$r_i(q_i,u)\f$ for bump
\f$q_i\f$ and rack \f$u\f$:

\f[
p_i=p+Rr_i,\qquad
v_i=v+R(\omega\times r_i+r_{i,q}\dot q_i+r_{i,u}\dot u).
\f]

With generalized velocity
\f$\nu=[v,\omega,\dot q_1..\dot q_4,\dot\theta_1..\dot\theta_4]\f$, each hub
Jacobian is \f$J_i=[I,\;-R[r_i]_\times,\;Rr_{i,q}e_i^T,\;0]\f$. The mass matrix
sums sprung-body translation/inertia, unsprung point masses \f$m_iJ_i^TJ_i\f$,
and wheel axial-inertia outer products. A positive-definite LDLT solve yields
the accelerations. The known acceleration bias includes centripetal, Coriolis,
path-curvature, mixed, and prescribed-rack terms; see `src/vehicle.cpp`.

Only yaw inertia is sourced. Estimated roll/pitch inertia is reduced by nominal
unsprung mass and axial wheel-spin contributions. Upright axial twist and full
transverse wheel/upright inertia are not modeled.

## Suspension

Source `SpringDamper` and coilover presets evaluate force along increasing
shock length \f$L_i(q_i,u)\f$:

\f[
F_{L_i}=-k_i(L_i-L_{ref,i})-c_i\dot L_i,\qquad
Q_{q_i}=L_{i,q}F_{L_i}.
\f]

Reference lengths are 0.1955/0.1970 m; stops use 0.200 m maximum and 0.057 m
stroke. Soft stops bound the spline domain. Reset runs a damped relaxation, so
ride height comes from the actual shock/contact system rather than a fixed CG
offset.

## Contact and tires

Contact uses the road normal projected into the wheel plane, so camber changes
the effective rolling radius and rim contact offset. Source `VerticalTire`
evaluates unilateral normal force from compression and compression velocity at
an explicit **180 kN/m / 650 N·s/m** override; the sibling prototype uses
15 kN/m and zero damping.

Longitudinal slip compares source tread speed with contact-base velocity, with
a 1 m/s regularized denominator. Lateral slip is
`atan2(-v_lateral, max(|v_forward|, 1 m/s))`. The default source `ReducedMFTire`
uses

\f[
F=\mu F_z\sin\left(C\arctan\frac{Ks}{C\mu F_z}\right),
\f]

then saturates longitudinal force first and limits lateral force to the
remaining friction circle. `tanh` and `linear` are registered alternatives with
the same combined-slip policy. The road multiplier sets the source coefficient
through a small adapter; it does not scale the normal load.

There is no relaxation length, aligning moment, temperature/wear model, or
measured load sensitivity. Saturating drive torque consumes lateral grip; the
plant adds no hidden traction or stability correction. Mechanical brakes and
rolling resistance are PERsim estimates because the source brake model is empty.

## Steering, aero, and integration

Nominal mean front-wheel steering maps monotonically to source rack offset.
Source wheel orientation supplies actual steering, bump steer, and camber; no
synthetic Ackermann is added. Lock is limited to the source table, about ±7.9°.

Keyboard input ramps throttle at 1.6/s and release at 5/s; steering slews at
2.5/s and recenters at 3.5/s. Keyboard steering is also bounded by the
kinematic envelope \f$\delta_{key}=\min(\delta_{max},
\arctan(L\,0.85\mu(g+F_{aero}/m)/\max(v_{XY}^2,1)))\f$. This is an input aid,
not extra grip. Gamepad and headless inputs are direct.

Source `Aero::getForce` and COP provide loads; reverse forward speed is clamped
to zero as in the source. Four unilateral belly points prevent chassis ground
penetration. `Vehicle::step` uses normalized quaternions and semi-implicit Euler
with a **0.5 ms** maximum substep. `Session` splits each 5 ms control interval
at delivered AMK command times.

The proving ground contains a 65 mm bump at X=18..22 m, a 0.4 m
ramp at X=35..53 m, ripples at X=58..80 m (all in the Y=-14..-6 m strip), and a
0.45 grip patch at X=25..45, Y=6..14 m. Cones and paint are not colliders.

## Parameters

`Parameters` reads live `PER27`, `PER27_WHEEL`, `DEFAULT_TIRE`,
`FRONT_COILOVER`/`REAR_COILOVER`, `AE27`, `AMK_DD5`, and DT27 efficiency. The
gearbox ratio stays **12.51** for current firmware, not DT27's 13.0.

Estimates include roll/pitch inertia (38/110 kg·m²), total unsprung mass (32 kg),
vertical tire override, stop stiffness, brake capacity/bias, steering slew and
sensor ratio, rolling resistance, battery/inverter electrical and thermal
parameters, sensor mounts, and noise levels. See [reused models](models.md),
[powertrain](powertrain.md), and [scope](validation.md).
