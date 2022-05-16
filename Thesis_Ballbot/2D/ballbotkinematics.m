syms vx vy wbz rb rw thetaw_axis thetax thetay thetaz real
% vx = 0;
% vy = 0;
% wbz =0.5;
% thetax =0;
% thetay =0;
% thetaz =0;
% thetaw_axis = (45.075*pi/180.0);
% rw = 0.05;
% rb = 0.125;
Rx = [1      0             0;
      0 cos(thetax) -sin(thetax);
      0 sin(thetax)  cos(thetax)] ;
Ry = [cos(thetay)        0       sin(thetay);
        0                1            0;
      -sin(thetay)       0       cos(thetay)] ;
Rz = [cos(thetaz)   -sin(thetaz)   0;
      sin(thetaz)    cos(thetaz)   0 ;
          0              0         1];              
 %Rr = Rx * Ry * Rz should be Rz*Ry*Rx
%w_wheel = [w_wh1;w_wh2;w_wh3];
w_dir =[cos(thetaw_axis)             0                     -sin(thetaw_axis);
       -cos(thetaw_axis)/2   sqrt(3)*cos(thetaw_axis)/2    -sin(thetaw_axis);
       -cos(thetaw_axis)/2  -sqrt(3)*cos(thetaw_axis)/2  -sin(thetaw_axis)];
rb_param= [0  1 0;
           -1 0 0;
           0  0 -rb];
% w_wheel = (Rr * w_dir * [vx ; vy; wbz]) /rw
w_wheel = (w_dir * rb_param * [vx ; vy; wbz]) /rw
%%  test rig Kinematics (ทดสอบให้ถูกเสร็จแล้ว)
rb = 0.125;
rw = 0.05;
wbx = 0;
wby = 0.2;
wbz = 0; 
thetaw_axis = (48.1*pi/180.0);
% w_dir =[cos(thetaw_axis)             0                        sin(thetaw_axis);
%        -cos(thetaw_axis)/2  -sqrt(3)*cos(thetaw_axis)/2       sin(thetaw_axis);
%        -cos(thetaw_axis)/2  sqrt(3)*cos(thetaw_axis)/2       sin(thetaw_axis)]; % อันที่ดันถูก
w_dir =[cos(thetaw_axis)             0                        -sin(thetaw_axis);
       -cos(thetaw_axis)/2  sqrt(3)*cos(thetaw_axis)/2       -sin(thetaw_axis);
       -cos(thetaw_axis)/2  -sqrt(3)*cos(thetaw_axis)/2       -sin(thetaw_axis)]; % อันที่ควรจะถูก สรุปล้อสลับกันอันนี้ถูกแล้ว
% w_dir = [cos(thetaw_axis)*cos(0)          -sin(0)    cos(0)*sin(thetaw_axis);
%         cos(thetaw_axis)*sin(2*pi/3)    cos(2*pi/3)  sin(thetaw_axis)*sin(2*pi/3);
%               -sin(thetaw_axis)               0         cos(thetaw_axis)];
w_wheel = -(rb/rw)*w_dir*[wbx;wby;wbz]
% จากการทำ simulation ได้ตรวจสอบแล้วว่า สมการนี้ถูกต้องสำหรับ TesrRig
% มีเครื่องหมายลบจากการกระจาย พจน์เนื่องจาก ทิศ rb กับ rw
% ขนานกันในทิศตรงกันข้ามมาดอทกัน เลยได้ -1
%note -3.42 mm. ลองทำบอล รัศมี123 mm.
%%  test rig Kinematics (paper) ได้ค่าไม่คล้ายของรุ่นพี่
rb = 0.125;
rw = 0.05;
wbx = 1;
wby = 1;
wbz = 0; 
thetaw_axis = (48.1*pi/180.0); % ไม่ลบเหมือนของพี่เพราะวัดไม่เหมือนกัน
% w_dir =[        0                    cos(thetaw_axis)       sin(thetaw_axis);
%         sqrt(3)*cos(thetaw_axis)/2   -cos(thetaw_axis)/2    sin(thetaw_axis);
%         -sqrt(3)*cos(thetaw_axis)/2   -cos(thetaw_axis)/2   sin(thetaw_axis)];
w_dir =[        0                    cos(thetaw_axis)       sin(thetaw_axis);
        -sqrt(3)*cos(thetaw_axis)/2   -cos(thetaw_axis)/2    sin(thetaw_axis);
        sqrt(3)*cos(thetaw_axis)/2   -cos(thetaw_axis)/2   sin(thetaw_axis)];
w_wheel = -(rb/rw).*w_dir*[wbx;wby;wbz]
% มีเครื่องหมายลบจากการกระจาย พจน์เนื่องจาก ทิศ rb กับ rw
% ขนานกันในทิศตรงกันข้ามมาดอทกัน เลยได้ -1
%%  test rig Kinematics (paperของงรุ่นพี่)
% syms wbx wby wbz rb rw real
wbx = 2;
wby = 2;
wbz = 0; 
rb =0.125;
rw =0.05;
theta = -(48.1/180)*pi  %48.1
Rx = @(x) [1      0             0;
         0 cos(x) -sin(x);
         0 sin(x)  cos(x)] ;
Ry = @(y)[cos(y)        0       sin(y);
        0                1            0;
       -sin(y)       0       cos(y)] ;
Rz = @(z) [cos(z)   -sin(z)   0;
            sin(z)    cos(z)   0 ;
             0              0         1]; 
%(120/180)*pi)
% t1 = cross(Rz(pi/2)*Ry(theta)*[1;0;0],Rz(pi/2)*Ry(theta)*[0;1;0])
% t2 = cross(Rz(7*pi/6)*Ry(theta)*[1;0;0],Rz(7*pi/6)*Ry(theta)*[0;1;0])
% t3 = cross(Rz(11*pi/6)*Ry(theta)*[1;0;0],Rz(11*pi/6)*Ry(theta)*[0;1;0])
% w1 = dot(cross(Rz(pi/2)*Ry(theta)*[1;0;0],Rz(pi/2)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
% w2 = dot(cross(Rz(7*pi/6)*Ry(theta)*[1;0;0],Rz(7*pi/6)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
% w3 = dot(cross(Rz(11*pi/6)*Ry(theta)*[1;0;0],Rz(11*pi/6)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
w1 = dot(cross(Rz(0)*Ry(theta)*[1;0;0],Rz(0)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
w2 = dot(cross(Rz(2*pi/3)*Ry(theta)*[1;0;0],Rz(2*pi/3)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
w3 = dot(cross(Rz(4*pi/3)*Ry(theta)*[1;0;0],Rz(4*pi/3)*Ry(theta)*[0;1;0]), [wbx;wby;wbz].*(rb/rw))
%% หมุนล้อใน CAD Omni
Rx = @(x) [1      0             0  0;
           0     cos(x)    -sin(x) 0;
           0     sin(x)     cos(x) 0;
           0      0             0  1 ] ;
Ry = @(y)[cos(y)        0       sin(y)  0;
            0           1            0  0;
         -sin(y)        0       cos(y)  0;
            0           0            0  1] ;
Rz = @(z) [cos(z)       -sin(z)   0  0;
            sin(z)       cos(z)   0  0;
             0              0     1  0;
             0              0     0  1];
% [   1      0     0  -0.008;
%                                      0      1     0  0.044;
%                                      0      0     1  0.012;
%                                      0      0     0    1]


H1 = [   1      0     0  -0.022; 0      1     0  0.039;0      0     1  -0.012; 0      0     0    1] *Rz(28.9*pi/180)*Ry(90*pi/180)
p2 = Rz(-320*pi/180)*H1
