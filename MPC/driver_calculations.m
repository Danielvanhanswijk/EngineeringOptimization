%% Calculate root mean square error
for k = 1:param.Tf/param.T-dim.N
    RMSE.temp(1:3,k) = [xr(1,k) - y(1,k);yr(1,k) - y(2,k); zr(1,k) - y(3,k)];
end
RMSE.x = sqrt(mean(RMSE.temp(1,:).^2));
RMSE.y = sqrt(mean(RMSE.temp(2,:).^2));
RMSE.z = sqrt(mean(RMSE.temp(3,:).^2));

%% Calculate total input

for i = 1:4
    input.prop(i) =  sum(abs(u_rec(i,:)));
end
input.average_u = mean(input.prop);