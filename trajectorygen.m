function [xr, yr, zr, psir] = trajectorygen( N_steps, t)

    xr = 5*sin(t) ;
    yr = t.*cos(t) ;
    zr = t ;
    psir = zeros(1,N_steps);
    
%     xr = zeros(1,N_steps);
%     yr = zeros(1,N_steps);
%     zr =zeros(1,N_steps);
%     psir = zeros(1,N_steps);
     

end