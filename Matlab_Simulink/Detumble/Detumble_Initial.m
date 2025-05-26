%% Detumble Simulink Initialization
clc
clear

% During the detumble phase, the reference frame is in LVLH

%% Initial Conditions
sc_mass = 640; % kg
sc_cm = [0;0;0]; % m
sc_d = 2; % m

sc_J = 1/6*sc_mass*sc_d^2 * eye(3); % kg/m^2

mu_earth = 398600; % km^3/s^2

omega_b_ECI = [-0.05; 0.03; 0.2]; % rad/s

% Orbit COEs
h = 53335.2; % km^2/s
ecc = 0;
RAAN = deg2rad(0); % rad
inc = deg2rad(98.43); % rad
arg_peri = deg2rad(0); % rad
True_Anomoly = deg2rad(0); % rad

% Initial Quaternion Relating the Body Frame to LVLH
epsilon = [0; 0; 0];
eta = 1;

% Initial Torque
Tc = [0;0;0];

%% Body to LVLH
epsilonx = [0,-epsilon(3),epsilon(2); epsilon(3),0,-epsilon(1); -epsilon(2), epsilon(1),0];

% Calculating the rotation matrix for body to LVLH
C_b_LVLH = (2*eta^2 - 1)*eye(3) + 2*(epsilon*epsilon') - 2*eta*epsilonx;


%% LVLH to ECI
[rECI, vECI] = curtis_COEstoRV(h, ecc, RAAN, inc, arg_peri, True_Anomoly, mu_earth); % Function gotten from H. Curtis "Orbital Mechanics for Engineering Students" in Appendix D
z_LVLH = -rECI/norm(rECI);
y_LVLH = -cross(rECI, vECI)/norm(cross(rECI, vECI));
x_LVLH = cross(y_LVLH, z_LVLH);

C_LVLH_ECI = [x_LVLH; y_LVLH; z_LVLH];

%% Body to ECI
C_b_ECI = C_b_LVLH*C_LVLH_ECI;

%% Quaternions from B to ECI
eta_b_ECI = ((trace(C_b_ECI) + 1)^0.5)/2;
epsilon_b_ECI = [(C_b_ECI(2,3)-C_b_ECI(3,2))/(4*eta_b_ECI); 
            (C_b_ECI(3,1)-C_b_ECI(1,3))/(4*eta_b_ECI); 
            (C_b_ECI(1,2) - C_b_ECI(2,1))/(4*eta_b_ECI)];
quat_b_ECI = [epsilon_b_ECI; eta_b_ECI];

%% Euler Angles from B to ECI
phi_b_ECI = atan2(C_b_ECI(2,3), C_b_ECI(3,3));
theta_b_ECI = -asin(C_b_ECI(1,3));
psi_b_ECI = atan2(C_b_ECI(1,2), C_b_ECI(1,1));

E_b_ECI = [phi_b_ECI; theta_b_ECI; psi_b_ECI];

%% Initial State
state0 = [omega_b_ECI; E_b_ECI; epsilon_b_ECI; eta_b_ECI];
