close all
clear
clc

%% Scale
N = 8;
dimx = 3;

%% Derived Data
Bt = [0 1 0;
      0 0 1];
Bb = [eye(2), zeros(2,1)];
alp = 1;
tau0 = 10;

%% Chosen Scheme
% sgm = @(t) 1;
sgm = @(t) 0 ...
           + 1*(t >=      0 && t < 1*tau0 || t >= 4*tau0 && t < 5*tau0) ...
           + 2*(t >= 1*tau0 && t < 3*tau0 || t >= 9*tau0) ...
           + 3*(t >= 3*tau0 && t < 4*tau0 || t >= 8*tau0 && t < 9*tau0) ...
           + 4*(t >= 5*tau0 && t < 8*tau0);
% 

%% Randomly Generated Data
x0 = rand(dimx,N) - 0.5;
for nodeidx = 1:N
    x0(:,nodeidx) = nodeidx * x0(:,nodeidx);
end

%% Simulation
fps = 12;
tF = 30;
tUnitStep = 1/fps;
tSteps = tF/tUnitStep;
taxis = linspace(0,tF,tSteps+1);

x_valts = [reshape(x0, [], 1), NaN(N*dimx, tSteps)];

[~, h, ~, G] = xdot_gen14(1, N);
h_trj = NaN(3*N, tSteps+1);
h_trj(:, 1) = h(0);

topo_trj = cell(tSteps+1, 1);
topo_tnow = NaN(3, size(G.E, 1)/2*3);
for eidx = 1:size(G.E, 1)/2
    edgeIi = G.E(eidx, :);
    topo_sidx = edgeIi(1);
    topo_tidx = edgeIi(2);
    topoPosBase = (eidx-1)*3;
    topo_tnow(:,topoPosBase+1) = x0(:, topo_sidx);
    topo_tnow(:,topoPosBase+2) = x0(:, topo_tidx);
    topo_tnow(:,topoPosBase+3) = NaN(3,1);
end
topo_trj{1} = topo_tnow;

% Initialization before the loop to ensure consistent behavior inside loop
sgm_val = -1;

for tidx = 2:tSteps+1
    tjust = taxis(tidx-1);
    tnow  = taxis(tidx);
    
    if sgm(tnow) ~= sgm_val
       fprintf("\nSwitching to <Graph %d> at t = %.3f s\n", sgm(tnow), tnow);
       [xdot, h, ~, G] = xdot_gen14(sgm(tnow), N);
       sgm_val = sgm(tnow);
    end
    
    [~, x_odetrj] = ode45( xdot, [tjust, tnow], x_valts(:,tidx-1) );
    
    x_valts(:, tidx) = x_odetrj(end,:).';
    h_trj(:, tidx) = h(tnow);
    
    x_tnow = reshape(x_valts(:, tidx), dimx, N);
    topo_tnow = NaN(3, size(G.E, 1)/2*3);
    for eidx = 1:size(G.E, 1)/2
        edgeIi = G.E(eidx, :);
        topo_sidx = edgeIi(1);
        topo_tidx = edgeIi(2);
        topoPosBase = (eidx-1)*3;
        topo_tnow(:,topoPosBase+1) = x_tnow(:, topo_sidx);
        topo_tnow(:,topoPosBase+2) = x_tnow(:, topo_tidx);
        topo_tnow(:,topoPosBase+3) = NaN(3,1);
    end
    topo_trj{tidx} = topo_tnow;
end

%% Data Visualization

X = reshape(x_valts, dimx, N, tSteps+1);
H = reshape(h_trj, dimx, N, tSteps+1);

% v = VideoWriter('Reimplmentation_doi_10_1049_iet_cta_2013_1007.mp4', 'MPEG-4');
% v.FrameRate = fps;
% v.Quality = 100;
% open(v)


fig = figure('Name', 'Formation Trajectory', 'Color', 'white', 'Position', [50 50 1280 720]);%, 'visible', 'off');
set(gca,'TickLabelInterpreter', 'latex');
grid on
hold on
% second index: the i-th agent
%plth = plot3(X(1,:,1), X(2,:,1), X(3,:,1));
toph = plot3(topo_trj{1}(1,:), topo_trj{1}(2,:), topo_trj{1}(3,:), 'Color', [164 0 6 32]/255, 'LineStyle', ':', 'LineWidth', 2);
scth = scatter3(X(1,:,1), X(2,:,1), X(3,:,1), 250, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [164 0 6]/255, 'MarkerFaceAlpha', .9);%'MarkerFaceColor', 'none');
refh = scatter3(H(1,:,1), H(2,:,1), H(3,:,1), 100, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [164 0 6]/255, 'MarkerFaceAlpha', .25);
% refh.NodeChildren(3).LineWidth = 2;
% tsch = textscatter3(X(1,:,1), X(2,:,1), X(3,:,1), ["1" "2" "3" "4" "5" "6" "7" "8"], 'TextDensityPercentage', 100, 'MarkerColor', 'none', 'ColorData', [1 1 1]);
% axis([-15 15 -15 15 -15 15]);%*ceil(max(x_valts(:))/10)*10);
% uistack(scth, 'top');
% uistack(tsch, 'top');

axis('tight', 'manual', 'square')
axis([-1 1 -1 1 -1 1]*ceil(max(abs(x_valts(:)))/10)*10);
view(45,22.5)

xlabel('$$x_{i1} (t)$$', 'Interpreter', 'latex');
ylabel('$$x_{i2} (t)$$', 'Interpreter', 'latex');
zlabel('$$x_{i3} (t)$$', 'Interpreter', 'latex');
pause(tUnitStep/2)

sgm_val = -1;
last_change = -1;
for fii = 2:tSteps+1
    tjust = taxis(fii-1);
    tnow  = taxis(fii);
    
    if sgm(tnow) ~= sgm_val
        last_change = tnow;
        set(toph, 'Color', [164 0 6 128]/255);
        sgm_val = sgm(tnow);
    end
    
    if tnow - last_change < 2.5
        set(toph, 'Color', [164 0 6 255-(tnow-last_change)/2.5*(255-32)]/255, 'LineStyle', '-');
    else
        set(toph, 'Color', [164 0 6 32]/255, 'LineStyle', ':');
    end

    set(toph, 'XData', topo_trj{fii}(1,:), 'YData', topo_trj{fii}(2,:), 'ZData', topo_trj{fii}(3,:));
    set(scth, 'XData', X(1,:,fii), 'YData', X(2,:,fii), 'ZData', X(3,:,fii));
%     set(tsch, 'XData', X(1,:,fii), 'YData', X(2,:,fii), 'ZData', X(3,:,fii));
    set(refh, 'XData', H(1,:,fii), 'YData', H(2,:,fii), 'ZData', H(3,:,fii));
%     uistack(scth, 'top');
%     uistack(tsch, 'top');
    title(sprintf('$$ L_{\\sigma(%.2f)} = L_{%d} $$', taxis(fii), sgm(taxis(fii))), 'Interpreter', 'latex');
    
%     frame = getframe(gcf);
%     writeVideo(v,frame);
    
    fprintf("%d / %d\n",fii, tSteps+1);
    pause(tUnitStep/2)
end

% close(v);

