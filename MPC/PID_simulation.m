%% Final assignment Model Predictive Control 

% Mike de Pont - Daniel van Hanswijk
function PID = PID_simulation(param)

initial_values


%% Linear state space design
Xr = [0,0]';

syms X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12 'real'
syms U1 U2 U3 U4 'real'

f = @(x,u) [x(2); param.g - u(1)/param.m];

Xs = [X1 X2]';
Us = U1;

Fs = f(Xs,Us);

JsX = jacobian(Fs,Xs);
JsU = jacobian(Fs,Us);

JA = subs(JsX,U1,-param.g*param.m);
LTI.A = double(subs(JA,Xs,Xr));
LTI.B = double(subs(JsU,Xs,Xr));
LTI.C = [1,0];

param.T = 0.1;
sys = ss(LTI.A,LTI.B,LTI.C,0);

sys_dr = c2d(sys,param.T);
%% Siso system
Kp = 20;
Kd = 20;
Ki = 0;

PID.t = 0:param.T:20;
K = pid(Kp,Ki,Kd,0,param.T);

PID.CL = feedback(-sys_dr*K,1,-1);

PID.Y_hover = lsim(PID.CL,ones(1,length(PID.t)),PID.t,'r');



%% Step response
% [Y,T] = step(CL);
PID.S = stepinfo(PID.Y_hover,PID.t,1);

%% Height tracking
PID.Y_inc = lsim(PID.CL,PID.t,PID.t,'r');

%% Sinusoid tracking
PID.Y_sin = lsim(PID.CL,1*ones(1,length(PID.t)) + sin(0.1*PID.t.^2),PID.t,'r');

end