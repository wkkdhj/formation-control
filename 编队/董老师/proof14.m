clear
clc
% 
% G = topologies( get_edges(1) );
% 
% U = eye(6);
% U(:,1) = ones(6,1);
% 
% ut1 = U(:,1);
% Ut = U(:,2:end);
% 
% invU = inv(U);
% ub1 = invU(1,:);
% Ub = invU(2:end,:);

syms h(t, ii)

% h(t,ii) = [6*sin(2*t + ii*pi/4); 12*sin(2*t + ii*pi/4); 6*cos(2*t + ii*pi/4)+sin(16*t + ii*pi/4)];

h(t,ii) = [6*sin(2*t + ii*pi/4)+5*t; 12*sin(2*t + ii*pi/4)+5*t; 6*cos(2*t + ii*pi/4)+sin(16*t + ii*pi/4)+5*t];

A = [4 -2 2
     1  3 5
     2  7 4];
Bt = [0 1 0;
      0 0 1];

v = Bt*jacobian(h,t) - Bt*A*h

     