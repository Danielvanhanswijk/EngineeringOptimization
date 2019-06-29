%% Final assignment Engineering optimization
% Abram Dekker......
% Daniel van Hanswijk 4351479

%%
clear
close all
warning off

%%
initial_parameters

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

dsys = c2d(sys,time.T);
LTI.A = dsys.A;
LTI.B = dsys.B;
LTI.C = dsys.C;
LTI.D = dsys.D;

LTI.B2 = [1, 1, 1, 1;...
          0, -1, 0, 1;...
          -1, 0, 1, 0;...
          -param.Km/param.Kf, param.Km/param.Kf, -param.Km/param.Kf, param.Km/param.Kf];
      
LTI.B = LTI.B*LTI.B2;

%% Reference tracking

[weight.Qf,~] = dare(LTI.A,LTI.B,weight.Q,weight.R); 
N_steps = time.T/time.Ts+1;

% Define trajectory        
[xr, yr, zr, psir] = trajectorygen(Xr, N_steps, time.tspan);

dim.max_t = length(time.tspan) - dim.N;
t = time.tspan(1:dim.max_t);

%% Constraints & Minimization problem

constraints

y = zeros(dim.ny,time.T/time.Ts-dim.N+1);      %Initialize y
x = zeros(dim.nx,time.T/time.Ts-dim.N+1);      %Initialize state matrix
x(:,1) = zeros(12,1);

disp('Calculating optimal input sequence ...')
f = waitbar(0,'','Name','Model Predictive Control');
for i = 1:time.T/time.Ts-dim.N
   
    LTI.x0 = x(:,i);
    LTI.y0 = LTI.C*LTI.x0;
  
    
    for k = 1:dim.N
        LTI.Yr(k+(k-1)*5:k+(k-1)*5+5,1) = [xr(i+k-1);yr(i+k-1);zr(i+k-1);psir(i+k-1);psir(i+k-1);psir(i+k-1)];
    end
    
    warning off
    [H,h,const,T_state,S_state] = premodgen(LTI,param,dim, weight);
    nu = dim.nu;
    N = dim.N; 

    
    %u = fmincon(@(x)fun(x,H,h, const, dim), zeros(dim.nu*dim.N,1));
    u = optimvar('u', dim.nu*dim.N)
    prob = optimproblem('Objective', fun(u,H, h, const, dim), 'ObjectiveSense', 'min')
    problem = prob2struct(prob)
    [sol, fval, exitflag, output] = quadprog(problem);
    
    
    
    
    J(:,i) = 0.5*sol'*H*sol + h'*sol + const;
    u_rec(:,i) = sol(1:dim.nu);
    %z_rec(:,i) = z(1:dim.nu);
    
    x(:,i+1) = LTI.A*LTI.x0+LTI.B*u_rec(:,i); 
    y(:,i+1) = LTI.C*x(:,i+1);
    
    

    waitbar(i/(time.T/time.Ts-dim.N),f,sprintf('Calculating optimal sequence...'))
   
end

delta_J(:) = J(2:end) - J(1:end-1);
delete(f)
%% 

disp('Plotting figures...')

%driver_calculations

driver_plots

Visualization

