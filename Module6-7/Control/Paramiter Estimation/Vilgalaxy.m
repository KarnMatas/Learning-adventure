function [dx,y] = Vilgalaxy(t,x,u,par1,par2,varargin)
    y = [x(1)];                       %Angular position
    dx = [x(2);                
       -par1*x(2)+ par2*u(1)  
       ]; 
end









%     dx =[x(2);                      %Angular Velocity
%         ((-b*x(2))/J)+((Kt*x(3))/(100*J));  %Angular Acceleration
%         ((-Kb*x(2)*100)/L) - ((R*x(3))/L)+ (u(1)/L)       %Current
%         ]; 
    % par1= (b/J)
    % par2= (Kt/(100*J))
    % par3= ((Kb*100)/L)
    % par4= (R/L)
    % par5= 1/L