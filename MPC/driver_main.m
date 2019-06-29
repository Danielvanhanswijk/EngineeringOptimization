%% READ ME
% 


%% Main driver file for different N
clear all
close all
clc


%% For finding N
% comment clear all out in main.m
% comment dim.N out in initial_values.m
clear all

dim.N = 10;
countt = 0;
N_range = (10:-1:3);


for N = N_range
    countt = countt + 1;
    dim.N = N;
    main
    
    if countt == 1
        amount = size(y,2)-2;
    end
    
    RootMeanSquare.x(countt) = RMSE.x;
    RootMeanSquare.y(countt) = RMSE.y;
    RootMeanSquare.z(countt) = RMSE.z;
    Position(:,:,countt) = [y(1,1:amount);y(2,1:amount);y(3,1:amount)];
    
   
   
   clearvars -except RootMeanSquare countt amount Position dim param xr yr zr N_range
end

for i = 1:size(N_range,2)
    figure(200)
    plot3(xr(1:amount),yr(1:amount),zr(1:amount));
    hold on
    plot3(Position(1,:,i),Position(2,:,i),Position(3,:,i),'--')
    
end
legend('Reference path','N=10', 'N=9', 'N=8', 'N=7','N=6', 'N = 5', 'N = 4')
xlabel('x direction (m)')
ylabel('y direction (m)')
zlabel('z direction (m)')

%% Cost functions
% comment clear all out in main_file
% comment dim.N out in initial_values
clear all
countt = 0;


for N = (10:-1:3)
    countt = countt + 1;
    dim.N = N;
    main_file
    
    if countt == 1
        amount = size(J,2);
    end
    
    Cost.J(countt,:) = J(1:amount);
    Cost.dJ(countt,:) = delta_J(1:amount-1);
    
    clearvars -except Cost countt amount
   figure(111)
   plot(Cost.J(countt,:))
   hold on
end


legend('N=15', 'N=14', 'N=13', 'N=12','N=11', 'N = 10', 'N = 9', 'N = 8', 'N = 7', 'N = 6', 'N = 4', 'N = 3')