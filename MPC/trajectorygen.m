function [xr, yr, zr, psir] = trajectorygen(Xr, N_steps, t)
prompt = 'Do you want to track a path or reference point (Reference path = 0, reference point =1)? ';
Own_path = input(prompt);


if Own_path == 0
    xr = 5*sin(t) ;
    yr = t.*cos(t) ;
    zr = t ;
    psir = zeros(1,N_steps);
    
elseif Own_path == 2
     happy = 0;
     while happy == 0
         close all
         str_x = input('Give an equation for x direction dependent on t: x = ','s')  ;
         str_y = input('Give an equation for y direction dependent on t: y = ','s') ;
         str_z = input('Give an equation for z direction dependent on t: z = ','s');
         f_x = inline(str_x,'t') ;
         f_y = inline(str_y, 't');
         f_z = inline(str_z, 't');
         xr = feval(f_x,t) - Xr(1)*ones(1,N_steps);
         yr = feval(f_y,t) - Xr(2)*ones(1,N_steps);
         zr = feval(f_z,t) - Xr(3)*ones(1,N_steps);
         psir = zeros(1,N_steps);
         
         plot3(xr,yr,zr);
         hold on
         plot3(Xr(1), Xr(2), Xr(3),'*')
         
         title('Predefined trajectory')
         xlabel('x direction (m)')
         ylabel('y direction (m)')
         zlabel('z direction (m)')
         legend('Trajectory', 'Initial position drone')
         
        
        prompt = 'Are you happy with this traject or do you want to make another(No, I want to make another=0, Yes, I am happy=1)? ';
        happy = input(prompt);
     end
     
elseif Own_path == 1
            xr = zeros(1,N_steps) ;
            yr = zeros(1,N_steps);
            zr = ones(1,N_steps);
            psir = zeros(1,N_steps);
     
     
end