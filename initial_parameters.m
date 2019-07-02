%% Time data

time.Ts = 0.4;                              % sample time
time.T = 10;                                % end time 
time.tspan = linspace(0,time.T,time.T*(1/time.Ts));% time span

%% Dimensions 
dim.nx = 12;                                % size of states
dim.nu = 4;                                 % size of inputs
dim.ny = 6;                                 % size of outputs

%% parameters drone

param.g = 9.81;                             % gravitation constant
param.m = 1;                                % mass of drone
param.l = 0.2;                              % length of arm
param.Kf = 1*10^(-10); % Thrust Coef
param.Km = 1*10^(-10); % Moment coef
param.r = 0.01; %radius of sphere
param.I = (2/5)*param.m*param.r^2; %Moment of Inertia sphere


%% MPC values
dim.N = 3;                                 % horizon length
time.t = length(time.tspan) - dim.N;         % time length
time.t_sim = time.tspan(1:time.t);

weight.Q = eye(dim.nx);
weight.Q(3,3) = 1000;
weight.R = eye(dim.nu);
weight.P = zeros(dim.nx); 