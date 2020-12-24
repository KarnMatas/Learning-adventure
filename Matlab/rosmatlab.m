rosshutdown
l3 = 0.27; %ท่อนแขนแรก
%l3_1 = 0.15;
l5 = 0.2; %ท่อนแขนถัดมา
l6 = 0.05; % joint ปลายแขนถึงปลายมือ
s2_link = 0.026; % ระยะ joint ถึง link ใหม่บริเวณl5,l6

c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
c5 = cos(q5);
c6 = cos(q6);
s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);
s5 = sin(q5);
s6 = sin(q6);
% ฟังก์ชันหลังจากนี้ใช้ตอน plot
fp4_q5 = @(q1,q2,q3,q4) [l5*c1*c2*s4 - s2_link*c1*c2*c4 - l3*c1*c2 + l5*c3*c4*s1 + s2_link*c3*s1*s4 - l5*c1*c4*s2*s3 - s2_link*c1*s2*s3*s4
 l5*c2*s1*s4 - l5*c1*c3*c4 - s2_link*c2*c4*s1 - s2_link*c1*c3*s4 - l3*c2*s1 - l5*c4*s1*s2*s3 - s2_link*s1*s2*s3*s4
                                                                                                    l5*(s2*s4 + c2*c4*s3) - l3*s2 - s2_link*c4*s2 + s2_link*c2*s3*s4
];
fp3_full = @(q1,q2)[  -l3*c1*c2
                      -l3*c2*s1
                      -l3*s2
                   ];
fp3_1 = @(q1,q2)[ -l3_1*c1*c2
                  -l3_1*c2*s1
                  l3_1*s2
                ];
fRe = @(q1,q2,q3,q4,q5,q6) [ c6*(c1*c2*s4 + c3*c4*s1 - c1*c4*s2*s3) + s6*(c5*(c1*c2*c4 - c3*s1*s4 + c1*s2*s3*s4) - s5*(s1*s3 + c1*c3*s2))     s5*(c1*c2*c4 - c3*s1*s4 + c1*s2*s3*s4) + c5*(s1*s3 + c1*c3*s2)       c6*(c5*(c1*c2*c4 - c3*s1*s4 + c1*s2*s3*s4) - s5*(s1*s3 + c1*c3*s2)) - s6*(c1*c2*s4 + c3*c4*s1 - c1*c4*s2*s3)
  s6*(c5*(c2*c4*s1 + c1*c3*s4 + s1*s2*s3*s4) + s5*(c1*s3 - c3*s1*s2)) - c6*(c1*c3*c4 - c2*s1*s4 + c4*s1*s2*s3)                                  s5*(c2*c4*s1 + c1*c3*s4 + s1*s2*s3*s4) - c5*(c1*s3 - c3*s1*s2)       s6*(c1*c3*c4 - c2*s1*s4 + c4*s1*s2*s3) + c6*(c5*(c2*c4*s1 + c1*c3*s4 + s1*s2*s3*s4) + s5*(c1*s3 - c3*s1*s2))
  s6*(c2*c3*s5 + c4*c5*s2 - c2*c5*s3*s4) + c6*(s2*s4 + c2*c4*s3)                                                                              s5*(c4*s2 - c2*s3*s4) - c2*c3*c5                                         c6*(c2*c3*s5 + c4*c5*s2 - c2*c5*s3*s4) - s6*(s2*s4 + c2*c4*s3)];
fp6 = @(q1,q2,q3,q4,q5,q6)  [l6*s6*(c5*(c1*c2*c4 - c3*s1*s4 + c1*s2*s3*s4) - s5*(s1*s3 + c1*c3*s2)) - l3*c1*c2 + l6*c6*(c1*c2*s4 + c3*c4*s1 - c1*c4*s2*s3) - s2_link*c1*c2*c4 + l5*c1*c2*s4 + l5*c3*c4*s1 + s2_link*c3*s1*s4 - l5*c1*c4*s2*s3 - s2_link*c1*s2*s3*s4
 l6*s6*(c5*(c2*c4*s1 + c1*c3*s4 + s1*s2*s3*s4) + s5*(c1*s3 - c3*s1*s2)) - l3*c2*s1 - l6*c6*(c1*c3*c4 - c2*s1*s4 + c4*s1*s2*s3) - l5*c1*c3*c4 - s2_link*c2*c4*s1 - s2_link*c1*c3*s4 + l5*c2*s1*s4 - l5*c4*s1*s2*s3 - s2_link*s1*s2*s3*s4
                                                                                                                                                                                                              l5*(s2*s4 + c2*c4*s3) - l3*s2 + l6*c6*(s2*s4 + c2*c4*s3) - s2_link*c4*s2 + l6*s6*(c2*c3*s5 + c4*c5*s2 - c2*c5*s3*s4) + s2_link*c2*s3*s4]; 


% Re = [0 1 0
%       0 0 1
%       1 0 0];


rosinit('192.168.20.50')
global A
global B
A = rossubscriber('/kumara/arm/pose/right')
pause(2)
while 1>0
    
    B = receive(A,10);
%     showdetails(B)

    p_1 = double(B.Data(1:3))/1000;
    p_2 = double(B.Data(8:10))/1000;
    p_3 = double(B.Data(15:17))/1000;
    p_4 = double(B.Data(22:24))/1000;
%     X = [p_1(1);p_2(1);p_3(1);p_4(1)];
%     Y = [p_1(2);p_2(2);p_3(2);p_4(2)];
%     Z = [p_1(3);p_2(3);p_3(3);p_4(3)];
    p1 = p_1-p_1;
    p2 = p_2-p_2;
    p3 = p_3-p_2;
    p4 = p_4-p_2;
%     X = [p1(1);p2(1);p3(1);p4(1)];
%     Y = [p1(2);p2(2);p3(2);p4(2)];
%     Z = [p1(3);p2(3);p3(3);p4(3)];
    
%     axis equal
%     view (90,24)
% rotationmatric at End effector
    preRe = B.Data(25:28)';
    Re = quat2rotm(double(preRe));
%  scaling
    n_13 = norm(p3-p1);
    p3_use = p3/n_13*l3;
    n_34 = norm(p4-p3);
    p4_temp = (p4-p3)/n_34*l5;
    p4_use = p4_temp+p3_use;
% 
    a = now;
    x = OpTest(p1,p2,p3_use,p4_use,Re,false);
%     ap3 = fp3_full(x(1),x(2));
%     ap4 = fp4_q5(x(1),x(2),x(3),x(4));
%     ap6 = fp6(x(1),x(2),x(3),x(4),x(5),x(6));
%     
%     x2 = [0 ap3(1) ap4(1)];
%     y2 = [0 ap3(2) ap4(2)];
%     z2 = [0 ap3(3) ap4(3)];
%     plot3(X,Y,Z,'-o',x2,y2,z2,'-o','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
     now - a

%     plot3(y2,-z2,x2);
%     hold on
%     fnplt(cscvn([X';Y';Z']),'r')
end
