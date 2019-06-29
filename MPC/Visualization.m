%% visualisation

X = x';  

T = t';
x_r = xr';
z_r = zr';
y_r = yr';

l=0.5;
rc=0.25;
rx=rc*cos(linspace(0,2*pi,1e1));
rx=[rx rx(1)];
ry=rc*sin(linspace(0,2*pi,1e1));
ry=[ry ry(1)];
w=5;
Ax=[-w+min(x_r) w+max(x_r) -w+min(y_r) w+max(y_r) -w+min(z_r) w+max(z_r)];


figure(7)
for j=1:length(T)-5
    Rt=R(X(j,4:6));
    R1=Rt*([l+rc+rx;ry;zeros(size(rx))])+X(j,1:3)';
    R2=Rt*([rx;l+rc+ry;zeros(size(rx))])+X(j,1:3)';
    R3=Rt*([-l-rc+rx;ry;zeros(size(rx))])+X(j,1:3)';
    R4=Rt*([rx;-l-rc+ry;zeros(size(rx))])+X(j,1:3)';

    A1=Rt*([-l l;0 0;0 0])+X(j,1:3)';
    A2=Rt*([0 0; -l l;0 0])+X(j,1:3)';

figure(200)
plot3(x_r(1:length(T)-5),y_r(1:length(T)-5),z_r(1:length(T)-5),'r',A1(1,:),A1(2,:),A1(3,:),'k',A2(1,:),A2(2,:),A2(3,:),'k',R1(1,:),R1(2,:),R1(3,:),'r',R2(1,:),R2(2,:),R2(3,:),'b',R3(1,:),R3(2,:),R3(3,:),'b',R4(1,:),R4(2,:),R4(3,:),'b');

axis(Ax);
set(gca,'box','on')
drawnow
end
shg

function y=R(Xrot)
phi=Xrot(1);
theta=Xrot(2);
psi=Xrot(3);

Rpsi=[cos(psi)  -sin(psi)   0;
      sin(psi)  cos(psi)    0;
      0         0           1];
  
% rotation around y with theta
Rtheta=[cos(theta)    0       sin(theta);
        0             1       0
        -sin(theta)    0       cos(theta)];
  
% rotation around x with phi 
Rphi=[1       0           0;
      0       cos(phi)    -sin(phi);
      0       sin(phi)    cos(phi)];

y=Rpsi*Rtheta*Rphi;
end

