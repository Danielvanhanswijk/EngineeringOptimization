%% Plots visualizing the input/outputs of the drone

% Predefined trajectory
t = sort(t);
figure(1)
plot3(xr(1:param.Tf/param.T-dim.N+1),yr(1:param.Tf/param.T-dim.N+1),zr(1:param.Tf/param.T-dim.N+1));
title('Predefined trajectory')
xlabel('x direction (m)')
ylabel('y direction (m)')
zlabel('z direction (m)')


% The trajectory followed by the drone
t = sort(t);
figure(2)
plot3(xr(1:param.Tf/param.T-dim.N+1),yr(1:param.Tf/param.T-dim.N+1),zr(1:param.Tf/param.T-dim.N+1));
hold on
plot3(y(1,1:end-2),y(2,1:end-2),y(3,1:end-2),'r--')
plot3(y(1,1), y(2,1), y(3,1), '*')
hold off
legend('Predefined trajectory', 'Path traveled by drone', 'Initial position')
xlabel('x direction (m)')
ylabel('y direction (m)')
zlabel('z direction (m)')

%% Plots of the seperate data

% accuracy in every direction
figure(3)
subplot(3,1,1)
plot(t,xr(1:param.Tf/param.T-dim.N+1),t,y(1,1:param.Tf/param.T-dim.N+1))
legend('reference x direction','path flown by drone')
title('x-direction')
xlabel('time (s)')
ylabel('x position (m)')

subplot(3,1,2)
plot(t,yr(1:param.Tf/param.T-dim.N+1),t,y(2,1:param.Tf/param.T-dim.N+1))
legend('reference x direction','path flown by drone')
title('y-direction')
xlabel('time (s)')
ylabel('y position (m)')

subplot(3,1,3)
plot(t,zr(1:param.Tf/param.T-dim.N+1),t,y(3,1:param.Tf/param.T-dim.N+1))
legend('reference x direction','path flown by drone')
title('z-direction')
xlabel('time (s)')
ylabel('z position (m)')


% all input/output values
figure(4)
subplot(3,3,1)
plot(t,x(4,1:param.Tf/param.T-dim.N+1));
title('Angle phi drone')

subplot(3,3,2)
plot(t,x(5,1:param.Tf/param.T-dim.N+1));
title('Angle theta drone')

subplot(3,3,3)
plot(t,x(6,1:param.Tf/param.T-dim.N+1));
title('Angle psi drone')

subplot(3,3,4)
plot(t(1:end-1),u_rec(1,1:param.Tf/param.T-dim.N))
title('Input 1 (force rotor 1)')

subplot(3,3,5)
plot(t(1:end-1),u_rec(2,1:param.Tf/param.T-dim.N))
title('Input 2 (force rotor 2)')

subplot(3,3,6)
plot(t(1:end-1),u_rec(3,1:param.Tf/param.T-dim.N))
title('Input 3 (force rotor 3)')

subplot(3,3,7)
plot(t(1:end),x(1,:))
title('x-position drone')

subplot(3,3,8)
plot(t(1:end),x(2,:))
title('y-position drone')

subplot(3,3,9)
plot(t(1:end),x(3,:))
title('z-position drone')

%% Error plots

figure(5)
plot(param.simultime(1:end-1), RMSE.temp)
hold on
if State_Disturbance == 1
    plot(6,RMSE.temp(1,30),'r*')
    plot(12,RMSE.temp(1,60),'r*')
end
legend('Error in x direction (m)', 'Error in y direction (m)', 'Error in z direction (m)','Disturbance')
title('Error with respect to the trajectory')
xlabel('time (s)')
ylabel('error (m)')

hold off
%% Optimal costs
figure(6)
subplot(1,2,1)
plot(J)
title('cost function V_N')
xlabel('step')
ylabel('Cost V_N')
subplot(1,2,2)
plot(delta_J)
title('V^*(k) - V^*(k+1)')


%% PID Hovering plots

figure(7)
plot(PID.t,PID.Y_hover,'r')
hold on
plot(PID.t,ones(1,length(PID.t)),'b')
title('Step response')
xlabel('Time [s]')
ylabel('Altitude of drone [m]')
legend('Trajectory drone','Desired trajectory')

figure(8)
plot(PID.t,PID.Y_inc,'r')
hold on
plot(PID.t,PID.t,'b')
title('PID controller')
xlabel('Time [s]')
ylabel('Altitude of drone [m]')
legend('Trajectory drone','Desired trajectory')

figure(9)
plot(PID.t,PID.Y_sin,'r')
hold on
plot(PID.t,1*ones(1,length(PID.t)) + sin(0.1*PID.t.^2),'b')
title('PID controller')
xlabel('Time [s]')
ylabel('Altitude of drone [m]')
legend('Trajectory drone','Desired trajectory')

