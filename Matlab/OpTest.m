function x = OpTest(p1,p2,p3,p4,Re,calRe)
    l3 = 0.27;
    l3_1 = 0.15;
    l5 = 0.2;
    s2 = 0.026;
    p4_q5 = @(q1,q2,q3,q4) [
        l5*cos(q1)*cos(q2)*sin(q4) - s2*cos(q1)*cos(q2)*cos(q4) - l3*cos(q1)*cos(q2) + l5*cos(q3)*cos(q4)*sin(q1) + s2*cos(q3)*sin(q1)*sin(q4) - l5*cos(q1)*cos(q4)*sin(q2)*sin(q3) - s2*cos(q1)*sin(q2)*sin(q3)*sin(q4)
        l5*cos(q2)*sin(q1)*sin(q4) - l5*cos(q1)*cos(q3)*cos(q4) - s2*cos(q2)*cos(q4)*sin(q1) - s2*cos(q1)*cos(q3)*sin(q4) - l3*cos(q2)*sin(q1) - l5*cos(q4)*sin(q1)*sin(q2)*sin(q3) - s2*sin(q1)*sin(q2)*sin(q3)*sin(q4)
                                                                                                           l5*(sin(q2)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - l3*sin(q2) - s2*cos(q4)*sin(q2) + s2*cos(q2)*sin(q3)*sin(q4)
        ];
    p3_full = @(q1,q2)[  
        -l3*cos(q1)*cos(q2)
        -l3*cos(q2)*sin(q1)
        -l3*sin(q2)
        ];
    p3_1 = @(q1,q2)[  
        -l3_1*cos(q1)*cos(q2)
        -l3_1*cos(q2)*sin(q1)
         l3_1*sin(q2)
        ];
    
    % opt = optimoptions('fmincon','display','off');
    % winopen(fullfile(matlabroot,'toolbox\optim\optim'))
    lb = [-2*pi -2*pi -2*pi -2*pi -2*pi -2*pi];
    ub = [8 8 8 8 8 8];
    incon = @(u)[];
    ReE = @(q1,q2,q3,q4,q5,q6)[ cos(q6)*(cos(q1)*cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q1) - cos(q1)*cos(q4)*sin(q2)*sin(q3)) + sin(q6)*(cos(q5)*(cos(q1)*cos(q2)*cos(q4) - cos(q3)*sin(q1)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q5)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) sin(q5)*(cos(q1)*cos(q2)*cos(q4) - cos(q3)*sin(q1)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) + cos(q5)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) cos(q6)*(cos(q5)*(cos(q1)*cos(q2)*cos(q4) - cos(q3)*sin(q1)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q5)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - sin(q6)*(cos(q1)*cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q1) - cos(q1)*cos(q4)*sin(q2)*sin(q3))
sin(q6)*(cos(q5)*(cos(q2)*cos(q4)*sin(q1) + cos(q1)*cos(q3)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) + sin(q5)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))) - cos(q6)*(cos(q1)*cos(q3)*cos(q4) - cos(q2)*sin(q1)*sin(q4) + cos(q4)*sin(q1)*sin(q2)*sin(q3)) sin(q5)*(cos(q2)*cos(q4)*sin(q1) + cos(q1)*cos(q3)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - cos(q5)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) sin(q6)*(cos(q1)*cos(q3)*cos(q4) - cos(q2)*sin(q1)*sin(q4) + cos(q4)*sin(q1)*sin(q2)*sin(q3)) + cos(q6)*(cos(q5)*(cos(q2)*cos(q4)*sin(q1) + cos(q1)*cos(q3)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) + sin(q5)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)))
                                                                                                          sin(q6)*(cos(q2)*cos(q3)*sin(q5) + cos(q4)*cos(q5)*sin(q2) - cos(q2)*cos(q5)*sin(q3)*sin(q4)) + cos(q6)*(sin(q2)*sin(q4) + cos(q2)*cos(q4)*sin(q3))                                                                       sin(q5)*(cos(q4)*sin(q2) - cos(q2)*sin(q3)*sin(q4)) - cos(q2)*cos(q3)*cos(q5)                                                                                                           cos(q6)*(cos(q2)*cos(q3)*sin(q5) + cos(q4)*cos(q5)*sin(q2) - cos(q2)*cos(q5)*sin(q3)*sin(q4)) - sin(q6)*(sin(q2)*sin(q4) + cos(q2)*cos(q4)*sin(q3))];
    opt = optimoptions('fmincon','algorithm','sqp');
    if calRe
        cost = @(q1,q2,q3,q4,q5,q6)(norm(p1-[0 ;0 ;0]))^2 + (norm(p3-p3_full(q1,q2)))^2 + (norm(p4-p4_q5(q1,q2,q3,q4)))^2;
        eqcon = @(q1,q2,q3,q4,q5,q6) Re-ReE(q1,q2,q3,q4,q5,q6);
         x = fmincon(@(u)cost(u(1),u(2),u(3),u(4),u(5),u(6)),[0 0 0 0 0 0],[],[],[],[],lb,ub,@(u)deal(incon(u),eqcon(u(1),u(2),u(3),u(4),u(5),u(6))),opt);
    else 
       cost = @(q1,q2,q3,q4,q5,q6)(norm(p1-[0 ;0 ;0]))^2 + (norm(p3-p3_full(q1,q2)))^2 + (norm(p4-p4_q5(q1,q2,q3,q4)))^2 + (norm(Re-ReE(q1,q2,q3,q4,q5,q6))*10)^2;%
       eqcon =  @(u)[];
       x = fmincon(@(u)cost(u(1),u(2),u(3),u(4),u(5),u(6)),[1 0 0 0 0 0],[],[],[],[],lb,ub,@(u)deal(incon(u),eqcon(u)));
    end
end