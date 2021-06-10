%% Linearization and Discretization
Ac = [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.inertia(1,1) 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.inertia(2,2) 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.inertia(3,3);
      0 -g 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.mass 0 0 0 0 0;
      g 0 0 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.mass 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/Vehicle.Airframe.mass 0 0 0;
      0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
Bc = [0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 1/Vehicle.Airframe.inertia(1,1) 0 0;
    0 0 1/Vehicle.Airframe.inertia(2,2) 0;
    0 0 0 1/Vehicle.Airframe.inertia(3,3);
    0 0 0 0;
    0 0 0 0;
    -1/Vehicle.Airframe.mass 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0
    0 0 0 0
    0 0 0 0
    0 0 0 0
    0 0 0 0
    0 0 0 0
    0 0 0 0];
Cc = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];
  
Dc = 0;
csys = ss(Ac, Bc, Cc, Dc);
dsys = c2d(csys, Ts);

A = dsys.A;
B = dsys.B;
C = dsys.C;


%% Kalman gain
Q(1,1) = 20;
Q(2,2)= 20;
Q(3,3)= 10;
Q(7,7) = 10;
Q(8,8) = 10;
Q(9,9) = 10;
Q(10,10) = 10;
Q(11,11) = 10;
Q(12,12) = 10;
Q(13,13) = 200;
Q(14,14) = 200;
Q(15,15) = 1;
Q(16,16) = 0.01;
Q(17,17) = 0.01;
Q(18,18) = 0.01;

Ro = eye(6);

%% Linear Quadratic Regulator
Alqr = A(1:12,1:12);
Blqr = B(1:12,1:4);
Bd = A(1:12,13:18);
Qlqr = eye(12)*0.1;
qlow = 	1;
qhi = 0.5;
Qlqr(1,1)= 10;   
Qlqr(2,2)= 10;
Qlqr(3,3)= 100;
Qlqr(4,4)= 10;
Qlqr(5,5)= 10;
Qlqr(6,6)= 100;
Qlqr(7,7)=50;
Qlqr(8,8)=50;
Qlqr(9,9)=10;
Qlqr(10,10)= 80;   %1.2;
Qlqr(11,11)= 80;  %1.2;
Qlqr(12,12)= 100;   %1;
Rlow = 1;
R(1,1) = 100;
R(2,2) = 10000000000;
R(3,3) = 10000000000; %2;
R(4,4) = 100;

M = [A(1:12,1:12)-eye(12) B(1:12,1:4);
     C(1:6,1:12)          zeros(6,4)];
M = pinv(M);

K = dlqr(Alqr, Blqr, Qlqr, R);
% abs(eig(Alqr-B*K))
%load eightCharPath
load CrossShapedPath
TFinal = 80;
Estimator.alt.kf.Q = 5e-3;
Estimator.alt.kf.R = 0.01;