%% Time data

time.Ts = 0.5;                              % sample time
time.T = 20;                                % end time 
time.tspan = linspace(0,time.T,time.T*(1/time.Ts));% time span

%% Dimensions 
dim.nx = 12;                                % size of states
dim.nu = 4;                                 % size of inputs
dim.ny = 4;%6                                 % size of outputs

%% parameters drone

param.g = 9.81;                             % gravitation constant
param.m = 0.5;                                % mass of drone
param.l = 0.2;                              % length of arm
param.Kf = 1*10^(-1); % Thrust Coef
param.Km = 1*10^(1); % Moment coef
param.r = 0.2; %radius of sphere
param.I = (2/5)*param.m*param.r^2; %Moment of Inertia sphere


%% MPC values
dim.N = 10;                                 % horizon length
time.t = length(time.tspan) - dim.N;         % time length
time.t_sim = time.tspan(1:time.t);

weight.Q = 100*eye(dim.nx);
weight.R = eye(dim.nu);
weight.P = zeros(dim.nx); 