<div align="center">
  <img src="/branding/logoV2/fulllogo_nobuffer.png" alt="Spyder Logo">
</div>

The Spyder drone, developed by [Nishant Kumar](https://github.com/nishantkumar201) and [Jai Willems](https://github.com/JaiWillems), is an x-configuration quad-rotor developed from the ground up, using arduino class sensors and compute.

| Parameter | Value | Unit |
| --------- | :---: | :--: |
| Weight | 1.04100 | kg |
| Motor Moment Arm | 0.27250 | m |
| Moment of Inertia, $I_{xb}$ | 0.01468 | kg $\cdot$ m$^2$ |
| Moment of Inertia, $I_{yb}$ | 0.01492 | kg $\cdot$ m$^2$ |
| Moment of Inertia, $I_{zb}$ | 0.02800 | kg $\cdot$ m$^2$ |
| Max Thrust / Motor | 12.139 | N |
| Max Torque / Motor | 0.208 | N $\cdot$ m |
| Thrust / Weight | 4.75470 | N / kg |


# Demo

TODO: Photos of the finished build, a hover video, CAD render.

# Overview

The Spyder drone is an interest project to explore mechatronic concepts applied to aerial systems. The goal was to develop a quadrotor capapable of controlled flight with extensibility for capable payloads including autonomous operations. To align with this learning objective, the platform was developed from the ground up with a fully custom airframe, bespoke controller design and implementation, and arduino class hardware. The result is a crude implementation with personality, quirks, and bruises.

# System Architecture

A System Block Diagram (SBD) is illustrated in [Figure 1](#fig-sbd) for the Spyder system defining segment and sub-system interactions and interfacing with external systems. The system decomposition is further explored in the System Hierarchy Diagram (SHD) illustrated in [Figure 2](#fig-shd).

<a id="fig-sbd"></a>
<div align="center">
  <img src="/design/Systems/System Block Diagram.svg" alt="System Block Diagram">
  <br>
  <em>Figure 1: System Block Diagram</em>
</div>

<br>

<a id="fig-shd"></a>
<div align="center">
  <img src="/design/Systems/System Hierarchy Diagram.svg" alt="System Hierarchy Diagram">
  <br>
  <em>Figure 2: System Hierarchy Diagram</em>
</div>

# Hardware

## Bill of Materials

TODO: Parts list.

## Airframe and Mechanical Design

TODO: The seven parts, design rationale, the artifacts and where to find them (maybe put in a table with links).

## Electrical

The electrical schematics for the Air and Ground Segments are provided in [Figure 3](#fig-as-schematic) and [Figure 4](#fig-gs-schematic), respectively.

<a id="fig-as-schematic"></a>
<div align="center">
  <img src="/design/Wiring Diagram/Drone/Spyder Wiring Diagram.svg" alt="Air Segment Electrical Schematic">
  <br>
  <em>Figure 3: Air Segment Electrical Schematic</em>
</div>

<a id="fig-gs-schematic"></a>
<div align="center">
  <img src="/design/Wiring Diagram/Ground Station/Ground Station Wiring Diagram.svg" alt="Ground Segment Electrical Schematic">
  <br>
  <em>Figure 4: Ground Segment Electrical Schematic</em>
</div>

# Modelling and Control Design

TODO: The quadcopter dynamics, coordinate axes, propellor spin directions, motor naming, control architecture, Simulink modeling. Plant model, limitations (no air resistance), minimal hardware limitations. Show model performance. Perhaps discuss the motor saturation problem.

# Motor and Propellor Characterization

TODO: Motor thrust test stand, measurement process, analysis calculations to arrive at final results. Add raw data and show plots with best fit lines and regression models.

# State Estimation

TODO: IMU library, gryo and accel calibration, magnetometer calibration, tilt compensation.

# Ground Station: Calibration and Signal Mapping

TODO: Startup center finding (calibration) (documented overflow ceiling), piecewise linear calibration mapping each half, cubic mapping, control authority limits, arm-disarm, packet encoding (why it is kept minimal).

# Build and Flash Instructions

TODO: Target boards, required libraries, how to compile each of the three sketches.

# License, Authors, and Acknowlegements

TODO
