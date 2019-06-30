%% Data
param.T=0.2;                        %sampletime 
param.Tf=20;                    
param.tspan=0:param.T:param.Tf;      %tspan for prediction model generation
param.x0=zeros(12,1);


param.g = 9.81;% gravitation constant
param.m = 5;% mass of drone
param.l = 0.2;% length of arm
param.Kf = 3.13*10^(-5); % Thrust Coef
param.Km = 7.5*10&(-7); % Moment coef
param.r = 0.3; %radius of sphere
param.I = (2/5)*param.m*param.r^2; %Moment of Inertia sphere
 
dim.nx=12;
dim.nu=4;
dim.ny = 6;
dim.N=10;                             %receding horizon
dim.t=length(param.tspan)-dim.N;    

param.simultime=param.tspan(1:dim.t); %simulation horizon

weight.Q=1000*eye(dim.nx);
weight.R=eye(dim.nu);
weight.P=zeros(dim.nx);

