<h1 align="center">monte carlo localization</h1>

<p align="center">
  particle-filter localization and competition robot control for VEX V5.
</p>

<p align="center">
  built with <a href="https://pros.cs.purdue.edu/v5/">PROS</a>,
  <a href="https://lemlib.readthedocs.io/">LemLib</a> and
  <a href="https://docs.lvgl.io/">LVGL</a>.
</p>

<details open>
<summary><strong>overview</strong></summary>

This project explores Monte Carlo localization on a VEX V5 competition robot. It combines
odometry with field-facing distance sensors to maintain a probabilistic estimate of the robot's
global position and heading.

- 7,500-particle localization filter
- odometry-based motion prediction
- distance-sensor measurement correction
- weighted resampling and filtered pose estimation
- autonomous selection and competition-ready driver control

<p align="center">
  <img src="include/mcl.png" width="900" alt="Monte Carlo localization update cycle">
</p>

</details>

<details open>
<summary><strong>localization</strong></summary>

Wheel odometry is fast and locally accurate, but error accumulates through wheel slip, sensor
noise and field inconsistencies. The particle filter limits that drift by repeatedly comparing
possible robot poses against distance measurements from the field.

Each update follows four stages:

1. **prediction** — move every particle using the latest odometry delta
2. **correction** — compare predicted and measured distance-sensor readings
3. **resampling** — replace unlikely particles with samples from higher-weight regions
4. **estimation** — calculate a filtered pose and feed it back into the chassis

The implementation lives in [`src/monte.cpp`](src/monte.cpp).

</details>

<details open>
<summary><strong>setup</strong></summary>

1. install the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/)
2. clone this repository
3. build the project with `pros m`
4. connect a VEX V5 Brain and upload with `pros u`

The configured robot uses a six-motor tank drivetrain, an inertial sensor, a horizontal tracking
wheel and four distance sensors. Motor, sensor and pneumatic ports are defined in
[`src/globals.cpp`](src/globals.cpp).

</details>

<details open>
<summary><strong>usage</strong></summary>

Select an autonomous routine on the V5 Brain before the match:

- `redNeg`
- `redPos`
- `blueNeg`
- `bluePos`
- `skills`

Driver controls:

- left and right joysticks control the tank drivetrain
- `L1` runs the intake forward
- `L2` runs the intake in reverse
- `R1` advances the Lady Brown mechanism through idle, primed and scored positions
- `R2` toggles the mobile-goal clamp

</details>

<details>
<summary><strong>project structure</strong></summary>

- [`src/main.cpp`](src/main.cpp) — competition lifecycle, autonomous selector and driver control
- [`src/monte.cpp`](src/monte.cpp) — particle-filter implementation
- [`src/auto.cpp`](src/auto.cpp) — match autonomous routines
- [`src/skills.cpp`](src/skills.cpp) — skills autonomous routine
- [`src/globals.cpp`](src/globals.cpp) — chassis, motors, sensors and pneumatics
- [`include/robot/`](include/robot/) — robot-specific interfaces
- [`project.pros`](project.pros) — PROS project configuration

</details>

<details>
<summary><strong>development</strong></summary>

- `pros m` builds the project
- `pros u` builds and uploads it to the connected V5 Brain
- `pros mu` performs both steps

The project targets C++20 and includes its PROS, LemLib, LVGL and Robodash dependencies in the
repository.

</details>

<details>
<summary><strong>license</strong></summary>

This project is currently unlicensed.

</details>
