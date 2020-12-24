function [theta3new,theta4new,chk] = N_R(theta3,theta4)
    f1 = 6*cos(theta3)- 4*cos(theta4)-6;
    f2 = 6*sin(theta3)- 4*sin(theta4)+1.732;
    chk = sqrt(f1^2 + f2^2)
    f =[f1 ; f2];
    df = [-6*sin(theta3) 4*sin(theta4); 6*cos(theta3) -4*cos(theta4)];
    deltaf = -inv(df) * f
    theta3new = theta3 + deltaf(1)
    theta4new = theta4 + deltaf(2)
end

