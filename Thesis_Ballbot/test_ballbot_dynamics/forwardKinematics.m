%{
Name : forwardKinematics
Description : create forwardKinematics
Author : Thansak Pongpraket
         Matas   Manawakul   
Date : Mar 25 2021​
%}
function H = forwardKinematics(q,rho,DH,Hne)
n = numel(q)
c =@(x) cos(x);
s =@(x) sin(x);
Transx =@(a) [1 0 0 a;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];
Rotx =@(b) [1  0     0    0;
            0 c(b) -s(b)  0;
            0 s(b)  c(b)  0;
            0  0     0    1];
Transz =@(c) [1 0 0 0;
              0 1 0 0;
              0 0 1 c;
              0 0 0 1];
Rotz =@(d) [c(d) -s(d) 0 0;
            s(d)  c(d) 0 0;
            0      0   1 0;
            0      0   0 1];
if isa(q,'sym') 
    Hi = eye(4); 
    for i = 1 : n
        % Transform by fixed parameters
        Tx = Transx(DH(i,1));  
        Rx = Rotx(DH(i,2));
        Tz = Transz(DH(i,3));
        Rz = Rotz(DH(i,4));
        if rho(i)==1
            Hj = Rotz(q(i)); % Tranform by joint parameter
        else
            Hj = Transz(q(i));
        end

        Hi= Hi*Tx*Rx*Tz*Rz*Hj; % Transform a frame respect to base frame
        H(:,:,i) = simplify(Hi) ;
    end
        H(:,:,n+1) = simplify(Hi*Hne);
else % Numeric
    H = zeros(4,4,n+1);
    Hi = eye(4); 
    for i = 1 : n
        % Transform by fixed parameters
        Tx = Transx(DH(i,1));  
        Rx = Rotx(DH(i,2));
        Tz = Transz(DH(i,3));
        Rz = Rotz(DH(i,4));
        if rho(i)==1
            Hj = Rotz(q(i)); % Tranform by joint parameter
        else
            Hj = Transz(q(i));
        end

        Hi= Hi*Tx*Rx*Tz*Rz*Hj; % Transform a frame respect to base frame
        H(:,:,i) = Hi ;
    end
        H(:,:,n+1) = Hi*Hne;
end
end