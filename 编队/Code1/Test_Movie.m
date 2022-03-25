%% 动画演示程序段

dN = 1;  % 动画加速度点火，越大动画加速度越快
% pi0   初始位置
% Piout 为每个队员的绝对位置
% Tout  为时间
Piout = Pi_out;
PlotColor = ['r','b','k','m'];   % 绘图颜色，长度必须与编队成员数一致

figure
axis equal
xlabel('X/m')
ylabel('Y/m')
axis([-1.5 1.5 -1.5 1.5])
hold on
h0 = plot( p0(1),p0(2),'+','MarkerSize',10,'Linewidth',2,'color','g' );
for jj = 1:Ni
    h(jj) = plot(pix0(1,jj),pix0(2,jj),'o','MarkerSize',10,'Linewidth',2,'color',PlotColor(jj));
end


for ii = 1:dN:length(Tout)
    set( h0,'xdata', P0out(1,ii),'ydata',P0out(2,ii))  % 更新位置
    if ii >= 1+dN
        line( [P0out(1,ii-1) P0out(1,ii)],[P0out(2,ii-1) P0out(2,ii)],'color','g' )
    end
    for jj = 1:Ni
        % 绘制轨迹
        if ii >= 1+dN
            line( [Piout(ii-dN,1,jj) Piout(ii,1,jj)],[Piout(ii-dN,2,jj)  Piout(ii,2,jj)],'color',PlotColor(jj) )
        end
        set( h(jj),'xdata', Piout(ii,1,jj),'ydata',Piout(ii,2,jj))  % 更新位置
        % 更新圆心坐标
    end
    pause(0.001);
    drawnow
end