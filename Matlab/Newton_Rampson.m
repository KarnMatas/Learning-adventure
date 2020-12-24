%  matas 61340500053 works with thanatat 61340500025
theta3 = 30*(pi)/180;
theta4 = 90*(pi)/180;
f1 = 6*cos(theta3)- 4*cos(theta4)-6;
f2 = 6*sin(theta3)- 4*sin(theta4)+1.732;
chk = sqrt(f1^2 + f2^2)
%% run nR
cutoff =0.001
iteration =0
while chk > cutoff
    [theta3,theta4,chk] = N_R(theta3,theta4)
    iteration = iteration+1
end