
g = 9.81; % [m / s^2], gravitational acceleration.

m = 0.547; % [kg], drone mass.
L = 0.17; % [m], arm length.

Ixb = 0.0033; % [kg m^2], body x moment of inertia.
Iyb = 0.0033; % [kg m^2], body y moment of inertia.
Izb = 0.0058; % [kg m^2], body z moment of inertia.

kF = 0.00000015; % [N / RPM^2], motor force coefficient.
kM = 0.000000375; % [Nm / RPM^2], motor torque coefficient.

z_Kp = 1;
z_Ki = 1;
z_Kd = 1;

roll_Kp = 1;
roll_Ki = 1;
roll_Kd = 1;

pitch_Kp = 1;
pitch_Ki = 1;
pitch_Kd = 1;

yaw_Kp = 1;
yaw_Ki = 1;
yaw_Kd = 1;
