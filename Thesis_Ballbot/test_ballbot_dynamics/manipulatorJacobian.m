%{
Name : manipulatorJacobian
Description : create manipulatorJacobian
Author : Thansak Pongpraket
         Matas   Manawakul   
Date : Mar 25 2021​
%}
function [J,Je] = manipulatorJacobian(q,rho,H,Hne)
n = numel(q);
S =@(P) [ 0   -P(3)  P(2);
          P(3)  0    -P(1);
         -P(2)  P(1)   0];
if isa(q,'sym')
%     H = forwardKinematics(q,rho,DH,Hne);
    for i = 1:n
        P00i = H(1:3,4,i);
        for j = 1:i
            P00j = H(1:3,4,j);
            z0j = H(1:3,3,j);
            if rho(j) == 1 % revolute joint
                Jwj = z0j;
                Jvj = cross(z0j,P00i-P00j);
            else
                Jwj = zeros(3,1);
                Jvj = z0j;
            end
                Ji(:,j) = [Jwj ; Jvj];
        end
        if i ~= n
            Ji(:,(i+1):n) = zeros(6,(n-i))
        end
            J(:,:,i) = simplify(Ji);
    end
    % Je
    Me = [      eye(3)                      zeros(3,3);
          S(H(1:3,1:3,n)*Hne(1:3,4)).'      eye(3)    ];
    Je = simplify(Me*J(:,:,i));
else % numeric
    Ji =zeros(6,n);
    J =zeros(6,n,n);
%         H = forwardKinematics(q,rho,DH,Hne);
      for i = 1:n
        P00i = H(1:3,4,i);
        for j = 1:i
            P00j = H(1:3,4,j);
            z0j = H(1:3,3,j);
            if rho(j) == 1 % revolute joint
                Jwj = z0j;
                Jvj = cross(z0j,P00i-P00j);
            else
                Jwj = zeros(3,1);
                Jvj = z0j;
            end
                Ji(:,j) = [Jwj ; Jvj];
        end
        if i ~= n
            Ji(:,(i+1):n) = zeros(6,(n-i));
        end
            J(:,:,i) = Ji;
      end
    % Je
    Me = [      eye(3)                      zeros(3,3);
          S(H(1:3,1:3,n)*Hne(1:3,4)).'      eye(3)    ];
    Je = Me*J(:,:,i);
end
end

