%% Plots visualizing the input/outputs of the drone

% Predefined trajectory
t = sort(t);
figure(1)
plot3(xr(1:time.T/time.Ts-dim.N+1),yr(1:time.T/time.Ts-dim.N+1),zr(1:time.T/time.Ts-dim.N+1));
title('Predefined trajectory')
xlabel('x direction (m)')
ylabel('y direction (m)')
zlabel('z direction (m)')


% The trajectory followed by the drone
t = sort(t);
figure(2)
plot3(xr(1:time.T/time.Ts-dim.N+1),yr(1:time.T/time.Ts-dim.N+1),zr(1:time.T/time.Ts-dim.N+1));
hold on
plot3(y(1,1:end-2),y(2,1:end-2),y(3,1:end-2),'r--')
plot3(y(1,1), y(2,1), y(3,1), '*')
hold off
legend('Trajectory', 'Path of the drone', 'Initial position')
xlabel('x direction (m)')
ylabel('y direction (m)')
zlabel('z direction (m)')

%% Plots of the seperate data

% accuracy in every direction
figure(3)
subplot(3,1,1)
plot(t,xr(1:time.T/time.Ts-dim.N),t,y(1,1:time.T/time.Ts-dim.N))
legend('reference ','path flown by drone')
title('x-direction')
xlabel('time (s)')
ylabel('x position (m)')

subplot(3,1,2)
plot(t,yr(1:time.T/time.Ts-dim.N),t,y(2,1:time.T/time.Ts-dim.N))
title('y-direction')
xlabel('time (s)')
ylabel('y position (m)')

subplot(3,1,3)
plot(t,zr(1:time.T/time.Ts-dim.N),t,y(3,1:time.T/time.Ts-dim.N))
title('z-direction')
xlabel('time (s)')
ylabel('z position (m)')


% all input/output values
figure(4)
subplot(3,3,1)
plot(t,x(4,1:time.T/time.Ts-dim.N));
title('Angle phi drone')

subplot(3,3,2)
plot(t,x(5,1:time.T/time.Ts-dim.N));
title('Angle theta drone')

subplot(3,3,3)
plot(t,x(6,1:time.T/time.Ts-dim.N));
title('Angle psi drone')

subplot(3,3,4)
plot(t,u_rec(1,1:time.T/time.Ts-dim.N))
title('Input 1 (force rotor 1)')

subplot(3,3,5)
plot(t,u_rec(2,1:time.T/time.Ts-dim.N))
title('Input 2 (force rotor 2)')

subplot(3,3,6)
plot(t,u_rec(3,1:time.T/time.Ts-dim.N))
title('Input 3 (force rotor 3)')

subplot(3,3,7)
plot(t,x(1,1:end-1))
title('x-position drone')

subplot(3,3,8)
plot(t,x(2,1:end-1))
title('y-position drone')

subplot(3,3,9)
plot(t,x(3,1:end-1))
title('z-position drone')

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

%% 
figure(7)
plot(t,zr(1:time.T/time.Ts-dim.N),t,y(3,1:time.T/time.Ts-dim.N))
legend('reference x direction','path flown by drone')
title('z-direction')
xlabel('time (s)')
ylabel('z position (m)')


%%
figure(8)
plot(t,x(4,1:time.T/time.Ts-dim.N));
hold on
plot(t,(pi/4)*ones(size(t,2),1), 'r--')
plot(t,-(pi/4)*ones(size(t,2),1), 'r--')
title('Angle phi drone')
legend('angle \phi drone', 'Angle constraint')
axis([0 15 -1 1])
