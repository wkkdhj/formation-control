%% 考虑半径变化仿真结果
clear
clc

% 目标点参数
p0 = [0 0]';         % 目标圆心
r = 1;               % 半径 
pix0 = [-0.2   0.5  -1 
       -0.16   0.5  -1 ];     % 初始位置
pi_ = pix0 - p0;                   % 相对位置   
d = [2*pi/3 2*pi/3 2*pi/3 ];         % 期望角距离

% 圆形汇聚控制器参数
lamda = 1;            % 改变角速度
gama = 2.5;           % 改变半径收敛的速度 

% 间距布局控制器参数
c1 = 2;
c2 = 5;
alpha = 1;

% 避免碰撞控制器参数
R1 = 0.4;   % 最小距离
R2 = 0.8;   % 防止碰撞作用距离
beta = 2;

% 构造被控对象参数结构体
par.r = r;           % 圆半径
par.x0 = p0(1);      % 圆心坐标x
par.y0 = p0(2);      % 圆心坐标y
par.d =  d;          % 期望角距离
par.lamda = lamda;   % 圆形汇聚控制器参数
par.gama = gama;     % 圆形汇聚控制器参数
par.c1 = c1;         % 间距布局控制器参数
par.c2 = c2;         % 间距布局控制器参数
par.alpha = alpha;   % 间距布局控制器参数
par.R1 = R1;         % 避免碰撞控制器参数
par.R2 = R2;         % 避免碰撞控制器参数
par.beta = beta;     % 避免碰撞控制器参数

% 控制器采样时间
tend = 8;              % 仿真终止时间    
ts = 0.01;             % 采样时间
Tout = 0:ts:tend;      % 仿真时间

Ni = length(pix0);  % 编队单元数量 
X = pix0;            % 状态初始化
for ii = 1:length(Tout)      % 时间循环
    
    t = Tout(ii);
    if t > 3 && t < 5        % 3~5s内半径由1m变为1.5m
        par.x0 =  p0(1) + 0.001*(ii-300);        % 半径变化
        par.y0 =  p0(2) - 0.001*(ii-300);        % 半径变化
    end
    
    
    u_d = Fun_ctrl_D( X,par );               % 求解间距布局控制器控制量
    U_a = Fun_ctrl_A( X,par );               % 求解避免碰撞控制器控制量
    for jj = 1:Ni  % 编队成员循环
        
        u_c = Fun_ctrl_C( X(:,jj),par );           % 求解运行汇聚控制量
        ui = u_c .* u_d(jj) + beta*U_a(:,jj);      % 控制器融合
        X(:,jj) = X(:,jj) + ui*ts;                 % 更新位置
        Ui(:,jj) = ui;
    end
    
    % 计算相邻队员角位移
    alphaI = zeros(Ni,1);      % 初始化
    for jj = 1:Ni-1
        pi0 = X(:,jj);
        pi1 = X(:,jj+1);
        alphaI(jj) = Fun_Vector_Angle(pi0,pi1);  % 求解相邻夹角
    end
    pi0 = X(:,Ni);  %
    pi1 = X(:,1);
    alphaI(Ni) = Fun_Vector_Angle(pi0,pi1);     % 求解相邻夹角
    
    % 保存结果用于绘图
    alphaIout(:,ii) = alphaI;  % 保存角间距信息
    Ui_out(ii,:,:) = Ui;       % 保存每个个体的控制量
    Pi_out(ii,:,:) = X;        % 保存绝对位置信息
    P0out(:,ii) = [par.x0; par.y0];     % 保存圆心坐标信息
end

% 坐标变化图
figure
hold on
plot( Tout',Pi_out(:,:,1) )
plot( Tout',Pi_out(:,:,2) )
plot( Tout',Pi_out(:,:,3) )
xlabel('时间/s')
ylabel('坐标/m') 

% 轨迹图
figure
hold on
axis equal
for jj = 1:Ni
   plot( Pi_out(:,1,jj)+p0(1),Pi_out(:,2,jj)+p0(2) )
end
xlabel('X/m')
ylabel('Y/m')
axis([-1.5 1.5 -1.5 1.5])

% 角间距变化图
figure
xlabel('X/m')
ylabel('角位移/rad')
hold on
for jj = 1:Ni
    plot( Tout,alphaIout(jj,:) )
end
plot( [Tout(1) Tout(end)],[d(1) d(1)],'k--' )

% 相对距离
for ii = 1:length(Tout)
    dX(ii,1) =  norm( Pi_out(ii,:,1)-Pi_out(ii,:,end) );    % 相对距离
    for jj = 2:Ni 
        dX(ii,jj) =  norm( Pi_out(ii,:,jj)-Pi_out(ii,:,jj-1) );    % 相对距离
    end
end
figure
plot( Tout', dX)
xlabel('时间/s')
ylabel('相对距离/m')

% 动画演示
Test_Movie

