function [qd,qv,qa,T] = GenTraj(q0,qf,v0,tf) % vf ,ai , af =0 
    t0 = 0;
%     vmax = pi/2;
% 
%     dq = qf - q0;
%     tf = abs(dq/((vmax-v0)/1.9));

    t = linspace(t0,tf,1000*(tf - t0));
    T = tf;

    ds = q0 - qf;
    tfv0 = tf*v0;

    co0 = q0;
    co1 = v0;
    co2 = 0;
    co3 =  - (2*(5*ds + 3*(tfv0)))/tf^3;
    co4 = (15*ds + 8*tfv0)/tf^4;
    co5 =  - (3*(2*ds + tfv0))/tf^5;

    qd = co0 + (co1.*t) + (co3.*t.^3) + (co4.*t.^4) + (co5.*t.^5);
    qv = co1 + (3*co3.*t.^2) + (4*co4.*t.^3) + (5*co5.*t.^4);
    qa = (6*co3.*t) + (12*co4.*t.^2) + (20*co5.*t.^3);
end

