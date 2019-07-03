%% Final assignment Model Predictive Control 
% Mike de Pont - Daniel van Hanswijk
   

clear all
close all
clc

rand('seed',100)
%% Noise
prompt = 'Do you want to include disturbance signals (no=0, yes=1)? ';
State_Disturbance = input(prompt);

%% Parameters
initial_values

syms phi theta psi
angle.phi = phi;
angle.theta = theta;
angle.psi = psi;

%% Linear state space design
Xr = [0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';

syms X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12 'real'
syms U1 U2 U3 U4 'real'

f = @(t,x,u) [x(7:12); ...
    (-cos(x(4))*cos(x(6))*x(5))*u(1)/param.m;...
    (x(4)*cos(x(6)))*u(1)/param.m;...
    (-cos(x(4))*cos(x(5))*u(1) + param.m*param.g)/param.m;...
    u(2)*param.l/param.I;...
    -u(3)*param.l/param.I;...
    -u(4)/param.I];

Xs = [X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12]';
Us = [U1 U2 U3 U4]';

Fs = f(0,Xs,Us);

JsX = jacobian(Fs,Xs);
JsU = jacobian(Fs,Us);

JA = subs(JsX,[U1, U2, U3, U4],[-param.g*param.m,0,0,0]); 
LTI.A = double(subs(JA,Xs,Xr));
LTI.B = double(subs(JsU,Xs,Xr));
LTI.C = [eye(6), zeros(6,6)];


sys = ss(LTI.A,LTI.B,LTI.C,0);

sys_dr = c2d(sys,param.T);
LTI.A = sys_dr.A;
LTI.B = sys_dr.B;
LTI.C = sys_dr.C;
LTI.D = sys_dr.D;

LTI.B2 = [1, 1, 1, 1;...
          0, -1, 0, 1;...
          -1, 0, 1, 0;...
          -param.Km/param.Kf, param.Km/param.Kf, -param.Km/param.Kf, param.Km/param.Kf];
LTI.B = LTI.B*LTI.B2;


%% Reference tracking

[weight.Qf,~] = dare(LTI.A,LTI.B,weight.Q,weight.R); 
N_steps = param.Tf/param.T+1;

t = param.tspan;

% Define trajectory        
[xr, yr, zr, psir] = trajectorygen(Xr, N_steps, t);

dim.max_t = length(t) - dim.N;
t = t(1:dim.max_t);

%% Constraints

% Input constraitn matrix
Ic = [eye(4);-eye(4)];
Iu = kron(eye(dim.N),Ic);
I_const = repmat([param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4;...
                  param.m*param.g/4],dim.N,1);


%Output state constraint matrix
state_ub = [pi;pi/2;pi];
state_lb = [-pi;-pi/2;-pi];
state_constr = repmat([state_ub;-state_lb],dim.N,1);
Cz = [  0 0 0 1 0 0 0 0 0 0 0 0;...
        0 0 0 0 1 0 0 0 0 0 0 0;...
        0 0 0 0 0 1 0 0 0 0 0 0];
Cz = [Cz;-Cz];              
Cz = kron(eye(dim.N),Cz);

%%



%%
y = zeros(dim.ny,param.Tf/param.T-dim.N+1);      %Initialize y
x = zeros(dim.nx,param.Tf/param.T-dim.N+1);      %Initialize state matrix
x(:,1) = param.x0;

disp('Calculating optimal input sequence ...')
f = waitbar(0,'','Name','Model Predictive Control');
for i = 1:param.Tf/param.T-dim.N
   
    LTI.x0 = x(:,i);
    LTI.y0 = LTI.C*LTI.x0;
  
    
    for k = 1:dim.N
        LTI.Yr(k+(k-1)*5:k+(k-1)*5+5,1) = [xr(i+k-1);yr(i+k-1);zr(i+k-1);psir(i+k-1);psir(i+k-1);psir(i+k-1)];
    end
    
    warning off
    [H,h,const,T_state,S_state] = premodgen(LTI,param,dim, weight);
    nu = dim.nu;
    N = dim.N; 

    cvx_begin quiet
        variable u(nu*N,1)
        minimize (0.5*u'*H*u + h'*u + const)
        subject to
            %Iu*u < I_const;
            %Cz*(T_state*LTI.x0 + S_state*u) < state_constr;
    cvx_end
    warning on
    
    J(:,i) = u'*H*u + h'*u + const;
    u_rec(:,i) = u(1:nu);
    

    a = -1;
    b = 1;
    if State_Disturbance == 1 && (i == 30 || i == 60)
        Fd = 1*(-35+i)/15;
        x(:,i+1) = LTI.A*LTI.x0+LTI.B*u_rec(:,i)+Fd*[zeros(6,1);-1;1;1;zeros(3,1)]; 
        y(:,i+1) = LTI.C*x(:,i+1);
    else
        x(:,i+1) = LTI.A*LTI.x0+LTI.B*u_rec(:,i); 
        y(:,i+1) = LTI.C*x(:,i+1);
    end
    waitbar(i/(param.Tf/param.T-dim.N),f,sprintf('Calculating optimal sequence...'))
end

delta_J(:) = J(2:end) - J(1:end-1);
delete(f)
%% 

disp('Plotting figures...')
PID = PID_simulation(param);

driver_calculations

driver_plots

Visualization
