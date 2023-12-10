%% Enivronment and Simulator
g=9.8065; % Gravitational Acceleration
%tspan=[0,300];
wind=[0;0];
%% Waypoints
wps=[[0;0],[20000;0]]
v_nav=20;
v_nav_lim=5;
loiter_radius=200;
[~,ts]=reference_trajectory(0,wps,v_nav,loiter_radius)
%t_span=[0,ts(end)+2*pi*loiter_radius/v_nav]
%tspan=linspace(0,500,10000);
tspan=[0,200];
%% UAV Parameters
v_lim=[10,40]; % Velocity Limits
v_lim_ult=[8,45];
a_lim=[-1,4]; % Acceleration Limits
phi_max=45*pi/180; % Max Bank Angle
n_max=1/cos(phi_max); % Max Load Factor
v_nom=20; % Nominal Velocity
R_turn=(v_nom^2/g)/sqrt(n_max^2-1);  % Best Turn Radius
w_max=g*sqrt(n_max^2-1)/v_nom;   % Best Turn Rate
ta=0.1;  % Time Constant for acceleration dynamics
tp=0.1;  % Time Constant for roll dynamics
%% Flock Parameters
N=10; % Number of agents
R=500; % Comms Radius (greater than times turn rate)
d1=200;  % Inter Agent distance
d2=200;
k=1.4; % Lattice ratio
r1=d1*k;
r2=d2*k;
t0=100;
t_change=20;
x0_lim=[-1000;1000]+wps(1,1);
y0_lim=[-1000;1000]+wps(2,1);
%v0_lim=[20;20];
v0_lim=[v_lim(1);v_lim(2)];
v0_lim=[19;21];
d_0=280;
theta0_lim=[0;0]*pi/180;
%theta0_lim=[-30;30]*pi/180;
phi0_lim=[0;0];
%phi0_lim=[0;0]
a0_lim=[0;0];
%a0_lim=[0;0]
%phi0_lim=[0;0];
%a0_lim=[0;0];
d_min=10;
%% Flocking Controller Params
% Navigation Controller
c1=2e-4 ; % Distance Error Gain
chi=1.2;   % Damping Ratio
c2=2*chi*sqrt(c1)  % Velocity Error Gain
%c2=1;   % OG K_nav= 1e-3, c1=5 c2=1
% Potential Controller
a=0.15/5;
b=0.12;
h=0.45; % Bump function param
eps=1; % Sigma norm param




