clear all
clc
syms mb mB mw rb rw l thetax dthetax phix dphix real
syms Ib IB Iw g real
syms ddthetax ddphix real
syms thetaw_axis tau1 tau2 tau3 real
%%
% P is Potential Energy Term
% K is Kinetics Energy Term
% psi is angular position of virtiual wheel
% ball Energy
Pb_yz = 0 ;
Kb_yz = 0.5*((mb * rb^2 * dthetax^2) + ((Ib)*dthetax^2));

% Body Energy
PB_yz = mB*g*l*cos(phix);
vB_yz_sqr = (rb^2*dthetax^2) + (2*rb*l*dthetax*dphix*cos(phix)) + (l^2 * dphix^2);
KB_yz = 0.5*((mB * vB_yz_sqr) + (IB*dphix^2));

% virtual wheel Energy
vw_yz_sqr = (rb^2*dthetax^2) + (2*rb*(rb+rw)*dthetax*dphix*cos(phix)) + ( (rb+rw)^2*dphix^2 );
dpsi = rb*(dthetax - dphix)/(2*rw);
Pw_yz = mw*g*(rb+rw)*cos(phix);
Kw_yz = 0.5*( (mw * vw_yz_sqr) + (Iw * dpsi^2) );

% Lagrange
L = Kb_yz + KB_yz + Kw_yz - Pb_yz - PB_yz - Pw_yz
% respected to dthetax & dphix
LHS1 = simplify(diff(L,dthetax))
LHS2 = simplify(diff(L,dphix))
% respected to thetax & phix
LHS3 = simplify(diff(L,thetax))
LHS4 = simplify(diff(L,phix))
% diff(LHS1,t)
dLHS1 = Ib*ddthetax + ( mw*(rb^2*ddthetax + rb*(rb+rw)*( (cos(phix)*ddphix) - (sin(phix)*dphix^2) ))) + (0.5*mB*(2*rb^2*ddthetax + (2*rb*l*(cos(phix)*ddphix - sin(phix)*dphix^2))))+ (mb*rb^2*ddthetax) - ( (0.25*Iw*rb^2*(ddphix - ddthetax)) / rw^2)
% diff(LHS2,t)
dLHS2 = Ib*ddphix + ( 0.5*mB*(ddphix - 2*rb*l*dthetax*sin(phix)*dphix)) + (0.5*mw*(2*(rb+rw)^2*ddphix - 2*rb*(rb+rw)*dthetax*sin(phix)*dphix)) + ( (0.25*Iw*rb^2*(ddphix - ddthetax)) / rw^2)
%%
sum_LHS = [simplify(expand(dLHS1 - LHS3));
           simplify(expand(dLHS2 - LHS4))]
% Note dLHS1 dLHS2 น่าจะหามาผิด
% sum_LHS = [simplify(dLHS1 - LHS3);
%            simplify(dLHS2 - LHS4)]
% RHS1 =
%% MCG for x = [theta phi]
% mtotal = mb + mB + mw;
% rtotal = rb + rw;
% alpha  = (mw*(rb+rw)) + mB*l;
% M = [Ib + (rb^2*mtotal) + (rb^2*Iw/rw^2)  rb*alpha*cos(phix) - (rb^2*Iw/rw^2);
%      rb*alpha*cos(phix)-rb^2*Iw/rw^2      rtotal^2*mw + rb^2*Iw/rw^2 + IB]
% C = [0  -rb*alpha*dphix*sin(phix);
%      0               0]
% G = [0;
%      -alpha*g*sin(phix)]
%% MCG for x = [y phi]
mtotal = mb + mB + mw;
rtotal = rb + rw;
alpha  = (mw*(rb+rw)) + mB*l;
M = [(Ib + (rb^2*mtotal) + (rb^2*Iw/rw^2))/rb  rb*alpha*cos(phix) - (rb^2*Iw/rw^2);
     ((rb*alpha*cos(phix)) - (rb^2*Iw/rw^2))/rb      rtotal^2*mw + rb^2*Iw/rw^2 + IB]
C = [0  -rb*alpha*dphix*sin(phix);
     0               0]
G = [0;
     -alpha*g*sin(phix)]
%% find torque from motor 
syms Tx real
JT = [rb/rw ;
     -rb/rw]
Jw_b = [ cos(thetaw_axis)                           0              -sin(thetaw_axis) ; %% ล้อกับบอล
        -cos(thetaw_axis)/2     sqrt(3)*cos(thetaw_axis)/2         -sin(thetaw_axis);
        -cos(thetaw_axis)/2     -sqrt(3)*cos(thetaw_axis)/2        -sin(thetaw_axis)];
Txyz = Jw_b * [tau1;tau2;tau3]
% Tyz = JT*Txyz(1)
Tyz = JT*Tx
%% find ddqyz
dqyz =[dthetax ;dphix];
ddqyz = simplify(M\(Tyz - (C*dqyz) - G)) .* [rb ; 1]

%% find A
syms dy dx real
ddqyz_thetax = diff(ddqyz,thetax)
ddqyz_phix = diff(ddqyz,phix)
ddqyz_dthetax = diff(ddqyz,dthetax)
ddqyz_dphix = diff(ddqyz,dphix)
% A_mat = zeros(4,4)
A_mat(3:4,1) = subs(ddqyz_thetax,[thetax phix dphix],[0 0 0])
A_mat(3:4,2) = subs(ddqyz_phix,[thetax phix dphix],[0 0 0])
A_mat(3:4,3) = subs(ddqyz_dthetax,[thetax phix dphix],[0 0 0])
A_mat(3:4,4) = subs(ddqyz_dphix,[thetax phix dphix],[0 0 0])
A_mat(1:2,:) = [ 0 0 1 0; 0 0 0 1]
u = Tx
%% find B
ddqyz_u = diff(ddqyz,Tx)
B_mat(3:4,1) = subs(ddqyz_u,[thetax phix dphix],[0 0 0])
% ได้ของ plane yz แล้วไปหาของ xy plane ต่อกับเรียน LQR
%% linearize state space ของทั้งระบบ ในรูป ABCD
% x = [phix phiy phiz dx dy dphix dphiy dphiz]
syms IB_xy Iw_xy real
%อัน 8 states
A_sys = [0 0 0 0 0 1 0 0; %dphix
         0 0 0 0 0 0 1 0; %dphiy
         0 0 0 0 0 0 0 1;%dphiz
         rb*[0 A_mat(3,2) 0 0 0 0 0 0]; %ddx  เกิดจาก บอลหมุนรอบแกน y
         rb*[A_mat(3,2) 0 0 0 0 0 0 0]; %ddy  เกิดจาก บอลหมุนรอบแกน x
         [A_mat(4,2) 0 0 0 0 0 0 0];    %ddphix 
         [0 A_mat(4,2) 0 0 0 0 0 0];    %ddphiy
         0 0 0 0 0 0 0 0] %ddphiz 

% A_sys = [0 0 0 0 0 1 0 0; %dphix
%          0 0 0 0 0 0 1 0; %dphiy
%          rb*[A_mat(3,2) 0 0 0 0 0 0 0]; %ddx
%          rb*[ 0 A_mat(3,2) 0 0 0 0 0 0]; %ddy
%          0 0 0 0 0 0 0 0] %ddphiz 
% 
B_sys = [ 0 0 0;
          0 0 0;
          0 0 0;
          B_mat(3)/rb       0        0;
              0        B_mat(3)/rb   0;
          B_mat(4)          0        0;                                               
              0          B_mat(4)    0;   
              0             0    (-rb/( (IB_xy*rw^2) + (mw*rw^2*(rb+rw)^2 + (Iw_xy*rb^2))))]

C_sys = eye(8)
D_sys = zeros(8,3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 5 states
% A_sys = [rb*[0 A_mat(3,2) 0 0 0 ]; %ddx  เกิดจาก บอลหมุนรอบแกน y
%          rb*[A_mat(3,2) 0 0 0 0 ]; %ddy  เกิดจาก บอลหมุนรอบแกน x
%          [A_mat(4,2) 0 0 0 0 ];    %ddphix 
%          [0 A_mat(4,2) 0 0 0 ];    %ddphiy
%          0 0 0 0 0 ] %ddphiz 
% 
% B_sys = [ B_mat(3)/rb       0        0;
%               0        B_mat(3)/rb   0;
%           B_mat(4)          0        0;                                               
%               0          B_mat(4)    0;   
%               0             0    (-rb/( (IB_xy*rw^2) + (mw*rw^2*(rb+rw)^2 + (Iw_xy*rb^2))))]
% 
% C_sys = eye(5)
% D_sys = zeros(5,3)










%% LQR
% Initial Condition

% Inertia moment of the ball = 0.01165
% Body moment of inertia around x-axis , y-axis , z-axis = 0.3
% Omniwheel moment of inertia around x-axis = 7.937*10^-4
% Omniwheel moment of inertia around y-axis = 7.937*10^-4
% Omniwheel moment of inertia around z-axis = 1.176*10^-3
% mB = 4.906;
% mb = 4;
% mw1 = 1.415;
% rb = 0.125;
% rw = 0.05;
% l = 0.208;

%Substitution paremeters value
A_sys = subs(A_sys,[mb mB mw rb rw l Ib IB Iw g],[4 4.906 1.415 0.12 0.05 0.208 0.01165 0.3 7.937*10^-4 -9.81]) 
B_sys = subs(B_sys,[mb mB mw rb rw l Ib IB Iw g IB_xy Iw_xy],[4 4.9064 1.415 0.12 0.05 0.208 0.01165 0.3 7.937*10^-4 -9.81 0.3 1.176*10^-3]) 
%
% x0 = [0.08;
%       0.08;
%       0;
%       0;
%       0;
%       0;
%       0;
%       0;]

%System Dynamics
% A_sys B_sys C_sys D_sys

%Control Law
% % 8 states
Q_state = [5 5 5 30 30 30 30 30 ];
R_state = 10.*[1 1 1];
% Q_state = [1 1 1 1 1 1 1 1 1];
% R_state = [1 1 1];
Q = [  Q_state(1)      0             0            0         0         0             0              0;
         0         Q_state(2)        0            0         0         0             0              0;
         0             0         Q_state(3)       0         0         0             0              0;
         0             0             0        Q_state(4)    0         0             0              0;
         0             0             0            0      Q_state(5)   0             0              0;
         0             0             0            0         0      Q_state(6)       0              0;
         0             0             0            0         0         0         Q_state(7)         0;
         0             0             0            0         0         0             0         Q_state(8)];

R = [R_state(1)    0            0 ;
     0        R_state(2)        0 ;
     0             0      R_state(3)  ];

K = lqr(double(A_sys),double(B_sys),Q,R);

% 5 states
% Q_state = [50 50 5 1 5 5];
% R_state = 5*[1 1 1];
% % Q_state = [1 1 1 1 1 1 1 1 1];
% % R_state = [1 1 1];
% Q = [  Q_state(1)      0             0            0         0    ;  
%          0         Q_state(2)        0            0         0    ;  
%          0             0         Q_state(3)       0         0    ;  
%          0             0             0        Q_state(4)    0    ;  
%          0             0             0            0      Q_state(5)];
% 
% R = [R_state(1)    0            0 ;
%      0        R_state(2)        0 ;
%      0             0      R_state(3)];
% 
% K = lqr(double(A_sys),double(B_sys),Q,R);
















%%
%Closed loop system
% sys_close = ss((double(A_sys) - double(B_sys)*K), double(B_sys), C_sys, D_sys);
% sys_open = ss(double(A_sys), double(B_sys), C_sys, D_sys);
% CL_loop = feedback(sys,K);
% t = 0:0.02:5;
% step(sys_close,t)
% %%
% %Run response to initial condition
% % t = 0:0.005:10;
% % lens = (10/0.005)+1
% % [y,t,x] = initial(sys, x0, t)
% % state_error = [0 0 0 1 0 0 0 0] - x0
% setpoint = [0 ;
%             0 ;
%             0 ;
%             0 ;
%             0 ;
%             0 ;
%             0 ;
%             0.2]
% tf = 20
% [traj_u(1,:),qv1,qa1,t1] = GenTraj(x0(1),setpoint(1),0,tf);
% [traj_u(2,:),qv2,qa2,t2] = GenTraj(x0(2),setpoint(2),0,tf);
% [traj_u(3,:),qv3,qa3,t3] = GenTraj(x0(3),setpoint(3),0,tf);
% [traj_u(4,:),qv4,qa4,t4] = GenTraj(x0(4),setpoint(4),0,tf);
% [traj_u(5,:),qv5,qa5,t5] = GenTraj(x0(5),setpoint(5),0,tf);
% [traj_u(6,:),qv6,qa6,t6] = GenTraj(x0(6),setpoint(6),0,tf);
% [traj_u(7,:),qv7,qa7,t7] = GenTraj(x0(7),setpoint(7),0,tf);
% [traj_u(8,:),qv8,qa8,t8] = GenTraj(x0(8),setpoint(8),0,tf);
% % traj_u(1,:) = linspace(x0(1),setpoint(1),lens)
% % traj_u(2,:) = linspace(x0(2),setpoint(2),lens)
% % traj_u(3,:) = linspace(x0(3),setpoint(3),lens)
% % traj_u(4,:) = linspace(x0(4),setpoint(4),lens)
% % traj_u(5,:) = linspace(x0(5),setpoint(5),lens)
% % traj_u(6,:) = linspace(x0(6),setpoint(6),lens)
% % traj_u(7,:) = linspace(x0(7),setpoint(7),lens)
% % traj_u(8,:) = linspace(x0(8),setpoint(8),lens)
% % input_u2 = -K*(setpoint - x0) % ผิดดด u มี 3 ตัว
% % input_u = (-K*traj_u);
% % [y,tOut,x] = lsim(sys,input_u,t1);
% n = size(t1)
% % input_karn = setpoint.*ones(8,n(2))
% [y,tOut,x] = lsim(sys_close,setpoint.*ones(8,n(2)),t1);
% %%
% plot(tOut,y)
