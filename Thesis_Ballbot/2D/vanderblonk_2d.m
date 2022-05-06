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
%% MCG
mtotal = mb + mB + mw;
rtotal = rb + rw;
alpha  = (mw*(rb+rw)) + mB*l;
M = [Ib + (rb^2*mtotal) + (rb^2*Iw/rw^2)  rb*alpha*cos(phix) - (rb^2*Iw/rw^2);
     rb*alpha*cos(phix)-rb^2*Iw/rw^2      rtotal^2*mw + rb^2*Iw/rw^2 + IB]
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
ddqyz = simplify(M\(Tyz - (C*dqyz) - G))
%% find A
ddqyz_thetax = diff(ddqyz,thetax)
ddqyz_phix = diff(ddqyz,phix)
ddqyz_dthetax = diff(ddqyz,dthetax)
ddqyz_dphix = diff(ddqyz,dphix)
% A_mat = zeros(4,4)
A_mat(3:4,1) = subs(ddqyz_thetax,[thetax phix dthetax dphix],[0 0 0 0])
A_mat(3:4,2) = subs(ddqyz_phix,[thetax phix dthetax dphix],[0 0 0 0])
A_mat(3:4,3) = subs(ddqyz_dthetax,[thetax phix dthetax dphix],[0 0 0 0])
A_mat(3:4,4) = subs(ddqyz_dphix,[thetax phix dthetax dphix],[0 0 0 0])
A_mat(1:2,:) = [ 0 0 1 0; 0 0 0 1]
u = Tx
%% find B
ddqyz_u = diff(ddqyz,Tx)
B_mat(3:4,1) = subs(ddqyz_u,[thetax phix dthetax dphix],[0 0 0 0])
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0
     0 0 0 1]
% ได้ของ plane yz แล้วไปหาของ xy plane ต่อกับเรียน LQR