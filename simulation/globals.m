% Ground Station

MIN_INPUT = 0;
MAX_INPUT = 1023;

Z_DOT_AUTHORITY = 1; % [m / s].
MIN_Z_DOT = -Z_DOT_AUTHORITY;
MAX_Z_DOT = Z_DOT_AUTHORITY;

ROLL_AUTHORITY = 0.17453; % [RAD], equivalent to 30 degrees.
MIN_ROLL = -ROLL_AUTHORITY;
MAX_ROLL = ROLL_AUTHORITY;

PITCH_AUTHORITY = 0.17453; % [RAD], equivalent to 30 degrees.
MIN_PITCH = -PITCH_AUTHORITY;
MAX_PITCH = PITCH_AUTHORITY;

YAW_RATE_AUTHORITY = 0.6; %1.25664; % [RAD / s], equivalent to 72 deg / s.
MIN_YAW_RATE = -YAW_RATE_AUTHORITY;
MAX_YAW_RATE = YAW_RATE_AUTHORITY;

% Drone

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
yaw_Ki = 0.01;
yaw_Kd = 0;
