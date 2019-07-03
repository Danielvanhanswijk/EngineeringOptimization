
i = 1;
y = zeros(dim.ny,time.T/time.Ts-dim.N+1);      %Initialize y
x = zeros(dim.nx,time.T/time.Ts-dim.N+1);      %Initialize state matrix
x(:,1) = zeros(12,1);
LTI.x0 = x(:,1);
LTI.y0 = LTI.C*LTI.x0;
  
    
    for k = 1:dim.N
        LTI.Yr(k+(k-1)*3:k+(k-1)*3+3,1) = [xr(i+k-1);yr(i+k-1);zr(i+k-1);psir(i+k-1)];
    end
    
    warning off
    [H,h,const,T_state,S_state] = premodgen(LTI,param,dim, weight);
    nu = dim.nu;
    N = dim.N; 
    count1 = 0;
    count2 = 0;

iii = linspace(-6, 2, 100);
jjj = iii;
for ii = iii
    count1 = count1 +1;
    for jj = jjj
        count2 = count2+1;
        uu(:,count2,count1) = [ii*ones(4,1); jj*ones(4,1); zeros(4,1)];
        costfun(count2,count1) = fun(uu(:,count2,count1), H, h, const, dim);
    end
       count2 = 0; 
end

[row,col] = find(costfun == min(min(costfun)));
input2 = iii(row);
input1 = jjj(col);




count3 = 0;
xx(:,1) = zeros(12,1);
for uuu = [input1, input2, 0]
%%test c
    count3 = count3+1;
    
    xx(:,count3+1) = LTI.A*LTI.x0+LTI.B*uuu*ones(4,1); 
    LTI.x0 = xx(:,count3+1);
    yy(:,count3) = LTI.C*xx(:,count3+1);
end
    
max_input = -param.m*param.g;
min_input = param.m*param.g/4;

close all
figure(1)
contour(iii,jjj,costfun, 'ShowText', 'On')
xlabel('first input')
ylabel('second input')
hold on
plot(input1,input2, '*')
plot([min_input min_input min_input min_input], [min_input max_input max_input min_input], 'b')
plot([max_input max_input max_input max_input], [min_input max_input max_input min_input], 'b')
plot([min_input max_input max_input min_input], [max_input max_input max_input max_input], 'b')
plot([min_input max_input max_input min_input], [min_input min_input min_input min_input], 'b')
legend('cost function','optimal input','input constraints')



figure(2)
waterfall(iii,jjj,costfun)
xlabel('first input')
ylabel('second input')
hold on
plot3(input1,input2,costfun(row,col), 'y*')
patch([min_input min_input min_input min_input], [min_input max_input max_input min_input], [-1000  -1000  100000 100000], [-1 1 1 -1])
patch([max_input max_input max_input max_input], [min_input max_input max_input min_input], [-1000  -1000  100000 100000], [-1 1 1 -1])
patch([min_input max_input max_input min_input], [max_input max_input max_input max_input], [-1000  -1000  100000 100000], [-1 1 1 -1])
patch([min_input max_input max_input min_input], [min_input min_input min_input min_input], [-1000  -1000  100000 100000], [-1 1 1 -1])
legend('cost function','optimal input','input constraints')

