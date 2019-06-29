function [xr, yr, zr, psir] = trajectorygen(Xr, N_steps, t)
prompt = 'Do you want to track a path or reference point (Reference path = 0, reference point =1)? ';
Own_path = input(prompt);


if Own_path == 0
    xr = 5*sin(t) ;
    yr = t.*cos(t) ;
    zr = t ;
    psir = zeros(1,N_steps);

     
elseif Own_path == 1
            xr = zeros(1,N_steps) ;
            yr = zeros(1,N_steps);
            zr = ones(1,N_steps);
            psir = zeros(1,N_steps);
     
     
end