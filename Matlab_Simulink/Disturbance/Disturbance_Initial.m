%% Nominal Operations Simulink Model
clc
clear
close all

%% Initial Conditions
wb = [0.001; -0.001; 0.002]; % initial angular velocity
J_SC = [812.0396, 0, 0; 0, 545.3729, 0; 0, 0, 627.7083];

muearth = 398600;
dyear_0 = decyear(2020, 3, 20, 12, 0, 0);

%% Calculating Initial Conditions
mu = 398600; % km^3/s^2
h = 53335.2; % km^2/s
e = 0; % none
Omega = 0*pi/180; % radians
inclination = 98.43*pi/180; % radians
omega = 0*pi/180; % radians
nu = 0*pi/180; % radians
a = h^2/mu/(1 - e^2);
orbital_period = 2*pi*sqrt(a^3/mu);
Re = 6378137; %m

T = [0;0;0];
% Set initial conditions
% orbital position and velocity h, e, RAAN, inc, w, TA, mu
[r_ECI_0, v_ECI_0] = curtis_COEstoRV(h, e, omega, inclination, omega, nu, mu); % Function gotten from H. Curtis "Orbital Mechanics for Engineering Students" in Appendix D
z_LVLH = -r_ECI_0/norm(r_ECI_0);
y_LVLH = -cross(r_ECI_0, v_ECI_0)/norm(cross(r_ECI_0, v_ECI_0));
x_LVLH = cross(y_LVLH, z_LVLH);

% Euler angles from body to LVLH
phi_0 = 0;
theta_0 = 0;
psi_0 = 0;
E_b_LVLH_0 = [phi_0; theta_0; psi_0];

% Calculate Initial Kinematics
C_LVLH_ECI_0 = [x_LVLH; y_LVLH; z_LVLH];
C_b_LVLH_0 = eye(3)*eye(3)*eye(3);
C_b_ECI_0 = C_b_LVLH_0*C_LVLH_ECI_0;
q_LVLH_ECI_0 = rotm2quat(C_LVLH_ECI_0);
q_b_LVLH_0 = [0; 0; 0; 1];

%q_b_ECI_0 = C2quat(C_b_ECI_0);
q_b_ECI_0 = quatmultiply(q_b_LVLH_0', q_LVLH_ECI_0);

% Euler angles from body to ECI
E_b_ECI_0 = rotm2eul(C_b_ECI_0);

% Initial body rates of spacecraft
w_LVLH_ECI_0 = C_b_ECI_0*(cross(r_ECI_0, v_ECI_0)/norm(r_ECI_0)^2)';
w_b_ECI_0 = [0.001; -0.001; 0.002];
w_b_LVLH_0 = w_b_ECI_0 - w_LVLH_ECI_0;

quat_b_ECI = q_b_ECI_0';

E_b_ECI = E_b_ECI_0;
E_dot = (1/cos(theta_0))*[cos(theta_0), sin(phi_0)*sin(theta_0), cos(phi_0)*sin(theta_0); ...
                            0, cos(phi_0)*cos(theta_0), -sin(phi_0)*cos(theta_0); ...
                            0, sin(phi_0), -cos(phi_0)];
orbit0 = [r_ECI_0'; v_ECI_0'];

%% surface prop
num_revs = 1;
sun_ECI = [1; 0; 0];
m_b = 0.5*[0; 0; -1];
cm = [0; 0; 0.234375000000000];

% First get rhos vectors with respect to the center of the spacecraft bus
Areas = 4*ones(6,1);
normals = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
cps = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];

% SP1
Areas = [Areas; 6; 6];
normals = [normals; [0 0 1]; [0 0 -1]];
cps = [cps; 0 2.5 .025; 0 2.5 -.025];

% SP2
Areas = [Areas; 6; 6];
normals = [normals; [0 0 1]; [0 0 -1]];
cps = [cps; 0 -2.5 .025; 0 -2.5 -.025];

% Sensor
Areas = [Areas; .25; .25; .25; .25; .25*.25];
normals = [normals; [1 0 0]; [-1 0 0]; [1 0 0]; [-1 0 0]; [0 0 1]];
cps = [cps; .125 0 1.5; -.125 0 1.5; 0 .125 1.5; -.125 0 1.5; 0 0 2];

% now subtract the center of mass to get the location of the rho vectors
% with respect to the center of mass
for i = 1:length(cps)
    cps(i,:) = cps(i,:) - cm';
end
% Now build the matrix
surfaceProperties = [Areas cps normals];

load aerowmm2020;

state0 = [wb; E_b_ECI'; quat_b_ECI];
orbit0 = [r_ECI_0'; v_ECI_0'];
