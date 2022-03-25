function  u_c = Fun_ctrl_C( X,par )
% u_c = Fun_ctrl_C( X,par ) 圆形汇聚控制器算法程序  

% 编队参数
r = par.r;     % 圆半径
x0 = par.x0;   % 圆心坐标x
y0 = par.y0;   % 圆心坐标y

% 无人船位置信息
xe = X(1);
ye = X(2);

% 无人船至圆心位置
p = sqrt( (xe-x0).^2 + (ye-y0).^2 );

% 控制参数
lamda = par.lamda;
gama = par.gama;

% 圆形汇聚控制向量
u_c = lamda * [ gama*(r^2-p^2)   -1;  1    gama*(r^2-p^2)  ] * X;   
