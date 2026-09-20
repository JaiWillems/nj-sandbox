# <a href="https://github.com/JaiWillems/nj-sandbox"><img alt="Spyder" src="/branding/logoV2/fulllogo_nobuffer.png" height="150"></a>

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

TODO: System diagrams including FFBD, SBD, etc.

# Hardware

## Bill of Materials

TODO: Parts list.

## Airframe and Mechanical Design

TODO: The seven parts, design rationale, the artifacts and where to find them (maybe put in a table with links).

## Electrical

TODO: Electrical schematics.

# Modelling and Control Design

TODO: The quadcopter dynamics, coordinate axes, propellor spin directions, motor naming, control architecture, Simulink modeling. Plant model, limitations (no air resistance), minimal hardware limitations). Show model performance. Perhaps discuss the motor saturation problem.

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
