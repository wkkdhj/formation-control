function [xdot, h, v, G] = xdot_gen14(k, N)

%% Fixed data
A = [4 -2 2
     1  3 5
     2  7 4];
B = [0 0;eye(2)];

G = topologies( get_edges14(k) );

h    = form_spec14(N);
%hdot = form_spec_dot(N);
v    = form_comp14(N);

K1 = zeros(2,3);
% To assign eigenvalues of A+BK1+BK2
K2 = [3.9988  -4.9905  -3.0122
     -7.0005  -4.9783  -5.9995];
% beta = 0.2;
% (32), solve Lyapunov-like inequality to get P
% K3 = B'/P
K3 = [-13.9520  8.5232  -2.45
        7.0585 -2.4500   5.0634];

alp = 3;

xdot = @(t, x) ...
    (kron(eye(N), A + B*K1 + B*K2) - kron(alp*G.L, B*K3)) * x + ...
    (kron(alp*G.L, B*K3) - kron(eye(N), B*K2)) * h(t) + ...
    kron(eye(N), B) * v(t);

end