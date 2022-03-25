function  u_lay = Fun_ctrl_D( X,par )
% u_lay = Fun_ctrl_D( X,par ) 间距布置控制器算法程序  
% 输出u_lay为标量

% 编队参数
d = par.d;                 % 期望角度分布间距
c1 = par.c1;               % 间距布局控制器参数
c2 = par.c2;               % 间距布局控制器参数
alpha = par.alpha;         % 间距布局控制器参数
Ni = length(d);            % 编队成员数量

alphaI = zeros(Ni,1);      % 初始化
% 计算相邻队员角位移
for ii = 1:Ni-1
   pi0 = X(:,ii);
   pi1 = X(:,ii+1);
   alphaI(ii) = Fun_Vector_Angle(pi0,pi1);  % 求解相邻夹角
end
pi0 = X(:,Ni);  %
pi1 = X(:,1);
alphaI(Ni) = Fun_Vector_Angle(pi0,pi1);     % 求解相邻夹角
    
u_dis =  zeros(Ni,1);      % 初始化
% 间距布局控制控量
u_dis(1) = ( alphaI(1) * d(Ni) / ( d(1) + d(Ni) )  - alphaI(Ni) * d(1) / ( d(1) + d(Ni) ) ).^ alpha ;
for ii = 2:length(d)
   u_dis(ii) = ( alphaI(ii) * d(ii-1) / ( d(ii) + d(ii-1) )  - alphaI(ii-1) * d(ii) / ( d(ii) + d(ii-1) ) ).^ alpha ;   
end

u_lay = c1 + u_dis * c2/(2*pi);  % 控制量求解
