
g = 9.81; % [m / s^2], gravitational acceleration.

m = 1; % [kg], drone mass.
L = 0.27305; % [m], arm length.

Ixb = 0.0033; % [kg m^2], body x moment of inertia.
Iyb = 0.0033; % [kg m^2], body y moment of inertia.
Izb = 0.0058; % [kg m^2], body z moment of inertia. 

r = 2; % Thrust to weight ratio.
kF = r * m * g / (4 * 100); % [N / PWM], motor force coefficient.

I = 30; % [A], ESC current.
kV = 1000; % Motor kv rating.
kM = I / kV; % [Nm / PWM], motor torque coefficient.

max_voltage = 4; % [V], maximum motor voltage.
rpm_per_voltage = 1000; % [RPM / V], motor KV rating.

z_Kp = 25;
z_Ki = 50;
z_Kd = 0;

roll_Kp = 6;
roll_Ki = 0.01;
roll_Kd = 0.25;

pitch_Kp = roll_Kp;
pitch_Ki = roll_Ki;
pitch_Kd = roll_Kd;

yaw_Kp = 1;
yaw_Ki = 0;
yaw_Kd = 0;
