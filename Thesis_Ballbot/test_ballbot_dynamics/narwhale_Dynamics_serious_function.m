%% gen traj
Tmax = 5
[q1d,q1v,q1a,t1f] = GenTraj(0,0,0,Tmax);
[q2d,q2v,q2a,t2f] = GenTraj(0,-0.52359,0,Tmax);
[q3d,q3v,q3a,t3f] = GenTraj(0,-0.52359,0,Tmax);

% %% convert qiVSt  to timeseries for import to simulink
q1dsim = timeseries(q1d,linspace(0,Tmax,numel(q1d)));
q1vsim = timeseries(q1v,linspace(0,Tmax,numel(q1v)));
q1asim = timeseries(q1a,linspace(0,Tmax,numel(q1a)));
q2dsim = timeseries(q2d,linspace(0,Tmax,numel(q2d)));
q2vsim = timeseries(q2v,linspace(0,Tmax,numel(q2v)));
q2asim = timeseries(q2a,linspace(0,Tmax,numel(q2a)));
q3dsim = timeseries(q3d,linspace(0,Tmax,numel(q3d)));
q3vsim = timeseries(q3v,linspace(0,Tmax,numel(q3v)));
q3asim = timeseries(q3a,linspace(0,Tmax,numel(q3a)));
t = linspace(0,Tmax,numel(q1d));
%% test Function 3dof_dynamics
% h1= 275.99/1000.0; % lasted
% h2= 380/1000.0;
% l1= 20/1000.0;
% l2= 380/1000.0;
% l3= 268.23/1000.0;

m1 = 3.0358
m2 = 2.0778
m3 = 1.4388
% cm1 = [-43.29; -2.02; -88.16];
% cm2 = [162; -0.02; 8.59];
% cm3 = [-175.56; 0.06; -6.33];
% cm1 = [-43.29/1000; -2.02/1000; -88.16/1000];
% cm2 = [162/1000; -0.02/1000; 8.59/1000];
% cm3 = [-175.56/1000; 0.06/1000; -6.33/1000];
% I1 = [0.00971694169       0.00064648471      0.01364762489;
%       0.00064648471        0.01128154665     0.00136042243;
%       0.01364762489    0.00136042243       0.01196416148];
% I2 = [0.00247391072       0.00003538205         0.00217523776;
%       0.00003538205         0.03115417518       -0.000038745;
%       0.00217523776       -0.000038745         0.029417341];
% I3 = [0.0016       -0.00001724909   0.00032435212;
%       -0.00001724909   0.0174       -0.00000416776;
%       0.00032435212    -0.00000416776       0.0164];
I1 = [0.00971694169       -0.00064648471      -0.01364762489;
      -0.00064648471        0.01128154665     -0.00136042243;
      -0.01364762489    -0.00136042243       0.01196416148];
I2 = [0.00247391072       -0.00003538205         -0.00217523776;
      -0.00003538205         0.03115417518       0.000038745;
      -0.00217523776       0.000038745         0.029417341];
I3 = [0.0016       0.00001724909   -0.00032435212;
      0.00001724909   0.0174       0.00000416776;
      -0.00032435212    0.00000416776       0.0164];
m = [m1 m2 m3]
I = cat(3,I1,I2,I3)
q = [q1d;q2d;q3d];
qd = [q1v;q2v ;q3v];
qdd = [q1a;q2a;q3a];
% hold on
% plot(t,q(1,:))
% plot(t,q(2,:))
% plot(t,q(3,:))
rho = [1;1;1];
%%
% [sigmaM,C,G,u,Mt] = threedof_dynamics(m,I,q,qd,qdd,rho)
% parameters
    cm1 = [-43.29; -2.02; -88.16]./1000.0;
    cm2 = [162; -0.02; 8.59]./1000.0;
    cm3 = [-175.56; 0.06; -6.33]./1000.0;
    cm=cat(3,cm1,cm2,cm3);
    %find m
    Mt =zeros(3,3,5000);
    Ct =zeros(3,3,5000);
    Gt =zeros(3,5000);
    ut =zeros(3,5000);
    % forwardkinematics
    h1= 275.99/1000.0; % lasted
    h2= 380.0/1000.0;
    l1= 20.0/1000.0;
    l2= 380.0/1000.0;
    l3= 268.23/1000.0;
    DH =[0   0   h1   0;
        l1 pi/2  0  pi/2;
        h2  0    0  -pi/2];
    
    Hne=[1 0  0  l2 ;
         0 1  0  0; 
         0 0  1  0;
         0 0  0  1];
    H1c1 = [1 0 0 cm1(1);
       0 1 0  cm1(2);
       0 0 1  cm1(3);
       0 0 0     1];
    H2c2 = [1 0 0 cm2(1);
            0 1 0 cm2(2);
            0 0 1 cm2(3);
            0 0 0    1];
    H3c3 = [1 0 0 cm3(1);
            0 1 0 cm3(2);
            0 0 1 cm3(3);
            0 0 0    1];
    % q=[q1;q2;q3];
    % rho = [1;1;1];
    c =@(x) cos(x);
    s =@(x) sin(x);
    Rotz =@(d) [c(d) -s(d) 0 0;
            s(d)  c(d) 0 0;
            0      0   1 0;
            0      0   0 1];
    m1 = m(1);
    m2 = m(2);
    m3 = m(3);
    g = 9.80665;
    n = size(q);
    count = 0
    for i= 1:n(2) 
        q1 =q(1,i);
        q2 =q(2,i);
        q3 =q(3,i);
        q1d = qd(1,i);
        q2d = qd(2,i);
        q3d = qd(3,i);
        q1dd = qdd(1,i);
        q2dd = qdd(2,i);
        q3dd = qdd(3,i);
        H = forwardKinematics(q(1:3,i),rho,DH,Hne);
%         H = fkno_jointransform(q(1:3,i),rho,DH,Hne);
        Hcm(:,:,1) = H(:,:,1)*H1c1;
        Hcm(:,:,2) = H(:,:,2)*H2c2;
        Hcm(:,:,3) = H(:,:,3)*H3c3;
%         Hcm(:,:,1) = H(:,:,1)*H1c1*Rotz(q1);
%         Hcm(:,:,2) = H(:,:,1)*Rotz(q1)*H(:,:,2)*H2c2*Rotz(q2);
%         Hcm(:,:,3) = H(:,:,1)*Rotz(q1)*H(:,:,2)*Rotz(q2)*H(:,:,3)*H3c3*Rotz(q3);
        %Jacobian
        Jwc(:,:,1) = [0, 0, 0;
                  0, 0, 0;
                  1, 0, 0];
        Jwc(:,:,2) =[0,  sin(q1), 0;
                 0, -cos(q1), 0;
                 1,        0, 0];
        Jwc(:,:,3) =[0,  sin(q1),  sin(q1);
                 0, -cos(q1), -cos(q1);
                 1,        0,        0];
        Jvc(:,:,1) =[- cm1(2)*cos(q1) - cm1(1)*sin(q1), 0, 0;
                   cm1(1)*cos(q1) - cm1(2)*sin(q1), 0, 0;
                                             0, 0, 0];
        Jvc(:,:,2) =[cm2(3)*cos(q1) - l1*sin(q1) + cm2(2)*cos(q2)*sin(q1) + cm2(1)*sin(q1)*sin(q2), -cos(q1)*(cm2(1)*cos(q2) - cm2(2)*sin(q2)), 0;
                 l1*cos(q1) + cm2(3)*sin(q1) - cm2(2)*cos(q1)*cos(q2) - cm2(1)*cos(q1)*sin(q2), -sin(q1)*(cm2(1)*cos(q2) - cm2(2)*sin(q2)), 0;
                                                                                       0,          - cm2(2)*cos(q2) - cm2(1)*sin(q2), 0]
        Jvc(:,:,3) =[cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2), -cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3));
                 l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2), -sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3));
                                                                                                                      0,            cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3) - h2*sin(q2),            cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)];
 
        % find M matrix
        for j = 1:numel(rho)
            M(:,:,j) = (m(j)*(Jvc(:,:,j)')*Jvc(:,:,j)) + ((Jwc(:,:,j)')*Hcm(1:3,1:3,j)*I(:,:,j)*Hcm(1:3,1:3,j)'*Jwc(1:3,:,j));
        end
        sigmaM = M(:,:,1) + M(:,:,2) + M(:,:,3);
        Mt(:,:,i)=sigmaM;
        
        C = [0.5000*q3d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2))) + 0.5000*q2d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) - cos(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2)) - sin(q2)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) + sin(q2)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2)) - 2*m2*(cm2(1)*cos(q1)*cos(q2) - cm2(2)*cos(q1)*sin(q2))*(l1*cos(q1) + cm2(3)*sin(q1) - cm2(2)*cos(q1)*cos(q2) - cm2(1)*cos(q1)*sin(q2)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m2*(cm2(1)*cos(q2)*sin(q1) - cm2(2)*sin(q1)*sin(q2))*(cm2(3)*cos(q1) - l1*sin(q1) + cm2(2)*cos(q2)*sin(q1) + cm2(1)*sin(q1)*sin(q2))), 0.5000*q1d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) - cos(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2)) - sin(q2)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) + sin(q2)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2)) - 2*m2*(cm2(1)*cos(q1)*cos(q2) - cm2(2)*cos(q1)*sin(q2))*(l1*cos(q1) + cm2(3)*sin(q1) - cm2(2)*cos(q1)*cos(q2) - cm2(1)*cos(q1)*sin(q2)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m2*(cm2(1)*cos(q2)*sin(q1) - cm2(2)*sin(q1)*sin(q2))*(cm2(3)*cos(q1) - l1*sin(q1) + cm2(2)*cos(q2)*sin(q1) + cm2(1)*sin(q1)*sin(q2))) - 0.5000*q3d*(2*sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - 2*cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) - m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) - m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*cos(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2))) + 0.5000*q2d*(2*sin(q1)*(cos(q1)*cos(q2)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) - cos(q1)*cos(q2)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) - sin(q1)*(I(2,3,2)*cos(q2) + I(1,3,2)*sin(q2)) + cos(q1)*sin(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) + cos(q1)*sin(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2))) - 2*cos(q1)*(cos(q1)*(I(2,3,2)*cos(q2) + I(1,3,2)*sin(q2)) - cos(q2)*sin(q1)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) + cos(q2)*sin(q1)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) + sin(q1)*sin(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) + sin(q1)*sin(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2))) - 2*sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) + 2*cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) - 2*m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*cos(q1)*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2)) + 2*m2*sin(q1)*(cm2(1)*cos(q1)*cos(q2) - cm2(2)*cos(q1)*sin(q2))*(cm2(1)*cos(q2) - cm2(2)*sin(q2)) - 2*m2*cos(q1)*(cm2(1)*cos(q2)*sin(q1) - cm2(2)*sin(q1)*sin(q2))*(cm2(1)*cos(q2) - cm2(2)*sin(q2)) + 2*m2*sin(q1)*(cm2(2)*cos(q2) + cm2(1)*sin(q2))*(l1*cos(q1) + cm2(3)*sin(q1) - cm2(2)*cos(q1)*cos(q2) - cm2(1)*cos(q1)*sin(q2)) + 2*m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m2*cos(q1)*(cm2(2)*cos(q2) + cm2(1)*sin(q2))*(cm2(3)*cos(q1) - l1*sin(q1) + cm2(2)*cos(q2)*sin(q1) + cm2(1)*sin(q1)*sin(q2))), 0.5000*q1d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2))) - 0.5000*q2d*(2*sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - 2*cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) - m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) - m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*cos(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2))) - 0.5000*q3d*(2*sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - 2*cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) + 2*m3*sin(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*cos(q1)*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2)) - 2*m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1)) + 2*m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3)));
                                                          - 0.5000*q1d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) - cos(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2)) - sin(q2)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) + sin(q2)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2)) - 2*m2*(cm2(1)*cos(q1)*cos(q2) - cm2(2)*cos(q1)*sin(q2))*(l1*cos(q1) + cm2(3)*sin(q1) - cm2(2)*cos(q1)*cos(q2) - cm2(1)*cos(q1)*sin(q2)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m2*(cm2(1)*cos(q2)*sin(q1) - cm2(2)*sin(q1)*sin(q2))*(cm2(3)*cos(q1) - l1*sin(q1) + cm2(2)*cos(q2)*sin(q1) + cm2(1)*sin(q1)*sin(q2))) - 0.5000*q3d*(m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) - m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) - m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0.5000*q3d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) + 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m3*cos(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))) - 0.5000*q2d*(sin(q1)*(I(3,1,2)*cos(q1)*cos(q2)*(cos(q1)^2 + sin(q1)^2) - I(3,2,2)*cos(q1)*sin(q2)*(cos(q1)^2 + sin(q1)^2)) - cos(q1)*(I(3,1,2)*cos(q2)*sin(q1)*(cos(q1)^2 + sin(q1)^2) - I(3,2,2)*sin(q1)*sin(q2)*(cos(q1)^2 + sin(q1)^2)) + sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) - cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - 2*m2*(cm2(2)*cos(q2) + cm2(1)*sin(q2))*(cm2(1)*cos(q2) - cm2(2)*sin(q2)) - 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m2*cos(q1)^2*(cm2(2)*cos(q2) + cm2(1)*sin(q2))*(cm2(1)*cos(q2) - cm2(2)*sin(q2)) + 2*m2*sin(q1)^2*(cm2(2)*cos(q2) + cm2(1)*sin(q2))*(cm2(1)*cos(q2) - cm2(2)*sin(q2)) + 2*m3*cos(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m3*sin(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2))) - 0.5000*q1d*(sin(q1)*(cos(q1)*cos(q2)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) - cos(q1)*cos(q2)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) - sin(q1)*(I(2,3,2)*cos(q2) + I(1,3,2)*sin(q2)) + cos(q1)*sin(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) + cos(q1)*sin(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2))) - cos(q1)*(cos(q1)*(I(2,3,2)*cos(q2) + I(1,3,2)*sin(q2)) - cos(q2)*sin(q1)*(I(1,1,2)*cos(q2) - I(2,1,2)*sin(q2)) + cos(q2)*sin(q1)*(I(2,2,2)*cos(q2) + I(1,2,2)*sin(q2)) + sin(q1)*sin(q2)*(I(2,1,2)*cos(q2) + I(1,1,2)*sin(q2)) + sin(q1)*sin(q2)*(I(1,2,2)*cos(q2) - I(2,2,2)*sin(q2))) - sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) + cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) - I(3,1,3)*cos(q2 + q3)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*sin(q2 + q3)*(cos(q1)^2 + sin(q1)^2) + I(3,2,2)*cos(q2)*(cos(q1)^2 + sin(q1)^2) + I(3,1,2)*sin(q2)*(cos(q1)^2 + sin(q1)^2)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0.5000*q1d*(sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) + I(3,1,3)*cos(q2 + q3)*(cos(q1)^2 + sin(q1)^2) - I(3,2,3)*sin(q2 + q3)*(cos(q1)^2 + sin(q1)^2) - m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) + m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) - m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))) + 0.5000*q2d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) + 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m3*cos(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))) + 0.5000*q3d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) + 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m3*cos(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*sin(q1)^2*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0.5000*q2d*(m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) - m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) - m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))) - 0.5000*q1d*(cos(q2 + q3)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + cos(q2 + q3)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) + sin(q2 + q3)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q2 + q3)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - 2*m3*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(l1*cos(q1) + cm3(3)*sin(q1) + cm3(1)*cos(q2 + q3)*cos(q1) - cm3(2)*sin(q2 + q3)*cos(q1) - h2*cos(q1)*sin(q2)) + 2*m3*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(3)*cos(q1) - l1*sin(q1) - cm3(1)*cos(q2 + q3)*sin(q1) + cm3(2)*sin(q2 + q3)*sin(q1) + h2*sin(q1)*sin(q2))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0.5000*q1d*(sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) + I(3,1,3)*cos(q2 + q3)*(cos(q1)^2 + sin(q1)^2) - I(3,2,3)*sin(q2 + q3)*(cos(q1)^2 + sin(q1)^2) + m3*sin(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1) + h2*cos(q1)*cos(q2)) - m3*cos(q1)*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1) + h2*cos(q2)*sin(q1)) - m3*sin(q1)*(cm3(2)*cos(q2 + q3)*cos(q1) + cm3(1)*sin(q2 + q3)*cos(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + m3*cos(q1)*(cm3(2)*cos(q2 + q3)*sin(q1) + cm3(1)*sin(q2 + q3)*sin(q1))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2))) - 0.5000*q2d*(sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) - cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) + 2*m3*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3) + h2*cos(q2)) + 2*m3*cos(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) + 2*m3*sin(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2))) + 0.5000*q3d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) - 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*cos(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*sin(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0.5000*q2d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) - 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*cos(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*sin(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))) + 0.5000*q3d*(cos(q1)*(I(3,2,3)*cos(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,1,3)*sin(q2 + q3)*sin(q1)*(cos(q1)^2 + sin(q1)^2)) - sin(q1)*(I(3,1,3)*sin(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2) + I(3,2,3)*cos(q2 + q3)*cos(q1)*(cos(q1)^2 + sin(q1)^2)) - 2*m3*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*cos(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3)) + 2*m3*sin(q1)^2*(cm3(2)*cos(q2 + q3) + cm3(1)*sin(q2 + q3))*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))) + 0.5000*q1d*(sin(q1)*(cos(q2 + q3)*cos(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3)) - cos(q2 + q3)*cos(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) - sin(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*cos(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3))) - cos(q1)*(cos(q1)*(I(1,3,3)*cos(q2 + q3) - I(2,3,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(2,1,3)*cos(q2 + q3) + I(1,1,3)*sin(q2 + q3)) + sin(q2 + q3)*sin(q1)*(I(1,2,3)*cos(q2 + q3) - I(2,2,3)*sin(q2 + q3)) - cos(q2 + q3)*sin(q1)*(I(1,1,3)*cos(q2 + q3) - I(2,1,3)*sin(q2 + q3)) + cos(q2 + q3)*sin(q1)*(I(2,2,3)*cos(q2 + q3) + I(1,2,3)*sin(q2 + q3))) + I(3,1,3)*cos(q2 + q3)*(cos(q1)^2 + sin(q1)^2) - I(3,2,3)*sin(q2 + q3)*(cos(q1)^2 + sin(q1)^2))];
 
        
        

        Ct(1:3,1:3,i) =C;
        S =@(P) [ 0   -P(3)  P(2);
                P(3)  0    -P(1);
                -P(2)  P(1)   0];
        g0 = [0;0;-g];
%         Jc1 =[J(1:3,:,1);Jv_c1];
%         Jc2 =[J(1:3,:,2);Jv_c2];
%         Jc3 =[J(1:3,:,3);Jv_c3];
%         G1 = -[Jwc(:,:,1);Jvc(:,:,1)]'*[Hcm(1:3,1:3,1)*S(cm1)*Hcm(1:3,1:3,1)'; eye(3)]*(m(1)*g0); 
%         G2 = -[Jwc(:,:,2);Jvc(:,:,2)]'*[Hcm(1:3,1:3,2)*S(cm2)*Hcm(1:3,1:3,2)'; eye(3)]*(m(2)*g0);
%         G3 = -[Jwc(:,:,3);Jvc(:,:,3)]'*[Hcm(1:3,1:3,3)*S(cm3)*Hcm(1:3,1:3,3)'; eye(3)]*(m(3)*g0);
          %vvv ก่อนแก้แบบdiff เอง
%         G1 = -[Jwc(:,:,1);Jvc(:,:,1)]'*[Hcm(1:3,1:3,1)*S(cm1)*Hcm(1:3,1:3,1)'; eye(3)]*(m(1)*g0);  
%         G2 = -[Jwc(:,:,2);Jvc(:,:,2)]'*[Hcm(1:3,1:3,2)*S(cm2)*Hcm(1:3,1:3,2)'; eye(3)]*(m(2)*g0);
%         G3 = -[Jwc(:,:,3);Jvc(:,:,3)]'*[Hcm(1:3,1:3,3)*S(cm3)*Hcm(1:3,1:3,3)'; eye(3)]*(m(3)*g0);

%         G1 = -Jc1.'*[H(1:3,1:3,1)*S(cm1)*H(1:3,1:3,1).'; eye(3)]*(m(1)*g0); 
%         G2 = -Jc2.'*[H(1:3,1:3,2)*S(cm2)*H(1:3,1:3,2).'; eye(3)]*(m(2)*g0);
%         G3 = -Jc3.'*[H(1:3,1:3,3)*S(cm3)*H(1:3,1:3,3).'; eye(3)]*(m(3)*g0);
        G = [                                                                                                   0;
      - g*m3*(cm3(2)*sin(q2 + q3) - cm3(1)*cos(q2 + q3) + h2*sin(q2)) - g*m2*(cm2(2)*cos(q2) + cm2(1)*sin(q2));
                                                              g*m3*(cm3(1)*cos(q2 + q3) - cm3(2)*sin(q2 + q3))];
%         G = G1+G2+G3;
        Gt(1:3,i)= G;
        u = (Mt(1:3,1:3,i)*qdd(1:3,i)) + (Ct(1:3,1:3,i)*qd(1:3,i)) +Gt(1:3,i);
%         u = (Mt(1:3,1:3,i)*qdd(1:3,i)) ;
        ut(1:3,i)=u;
        count = count +1
    end
%     u= 1
    % เทส ฟังก์ชันหา M ให้ได้เลขออกมา
    % เทสฟังก์ชันหา C ให้ได้เลขออกมา
    % เทสฟังก์ชันหา G ให่ได้เลขออกมา
    % เทสให้ได้ U ออกมา
    % ลองเทสใส่ traj. q qd qdd ลองไป
    hold on
    subplot(3,1,1)
    plot(t,ut(1,:))
    hold on
    subplot(3,1,2)
    plot(t,ut(2,:))
    hold on
    subplot(3,1,3)
    plot(t,ut(3,:))
