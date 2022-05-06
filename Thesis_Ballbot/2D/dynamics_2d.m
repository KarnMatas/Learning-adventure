syms qx1 qx2 rb mb mB phi dphix real
syms mw1  real
qx = [qx1 qx2];
Ib =sym('Ib',[3 3],'real')
IB =sym('IB',[3 3],'real')
Iw =sym('Iw',[3 3],'real')
% Rx = [1      0             0;
%       0     cos(q1) -sin(q1);
%       0     sin(q1)  cos(q1)] ;
Jvball = [0 0;
          1 0;
          0 0]
Jvbody = [0 0;
          1 -rb*sin(qx2);
          0  rb*cos(qx2)]
Jwball = [1/rb 0;
          0    0;
          0    0]
Jwbody = [1/rb  1;
          0     0;
          0     0]

Rx_qx2 = [1      0             0;
         0     cos(qx2) -sin(qx2);
         0     sin(qx2)  cos(qx2)] ;

Jwwheel1 = [0 1;
            0 0
            0 0]
Pbbw = [rb*cos(qx1);
             0;
        rb*sin(qx1)]
P00w = [0; qx1; 0] + (Rx_qx2 * Pbbw )

Jvwheel1 = [-rb*sin(qx1)                                   0;
            1 - rb*cos(qx1)*sin(qx2)   -rb*cos(qx2)*sin(qx1);
            rb*cos(qx1)*cos(qx2)       -rb*sin(qx1)*sin(qx2)]
           
M1 = mb*(Jvball')*Jvball +(Jwball')*Ib*Jwball
M2 = mB*(Jvbody')*Jvbody +(Jwbody')*IB*Jwbody
M3 = mw1*(Jvwheel1')*Jvwheel1 + (Jwwheel1')*Iw*Jwwheel1
sigmaM = simplify(M1+M3)