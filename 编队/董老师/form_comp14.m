function v = form_comp14(N)
% returns a N*dimx * 1 function handle about time.
v = @(t) [];

% for ii = 0:N-1
%     v = @(t) [v(t); 15*cos(2*t + ii*pi/3)-285*sin(2*t + ii*pi/3)];
% end

for ii = 0:N-1
    v = @(t) [v(t);
        - 6*cos(2*t + (pi*ii)/4) - 42*sin(2*t + (pi*ii)/4) - 5*sin(16*t + (pi*ii)/4);
 16*cos(16*t + (pi*ii)/4) - 24*cos(2*t + (pi*ii)/4) - 108*sin(2*t + (pi*ii)/4) - 4*sin(16*t + (pi*ii)/4)];
end

end
