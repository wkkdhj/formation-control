function h = form_spec14(N)
% returns a N*dimx * 1 function handle about time.
h = @(t) [];

for ii = 0:N-1
    h = @(t) [h(t); 6*sin(2*t + ii*pi/4); 12*sin(2*t + ii*pi/4); 6*cos(2*t + ii*pi/4)+sin(16*t + ii*pi/4)];
end

end

