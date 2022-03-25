function  U_a = Fun_ctrl_A( X,par )
% u_a = Fun_ctrl_A( X,par ) 避免碰撞控制器算法程序
% X为编队各个成员的绝对位置

% 编队参数
R1 = par.R1;     % 最小安全距离
R2 = par.R2;     % 避碰距离

Ni = length(X);
% 避免碰撞控制器
for ii = 1:length(X)
    for jj = 1:length(X)
        P(ii,jj) = sqrt(sum( (X(:,ii) - X(:,jj)).^2 ) );
    end
end
Pij = (P.^2 - R2^2) / ( P.^2 - R1^2 );
Pij = min( Pij,0 ).^2;

dPij = zeros(Ni,2,Ni);     % 初始化
for ii = 1:length(X)
    for jj = 1:length(X)
        if P(ii,jj) >= R2
            dPij(ii,:,jj) = [0;0];
        else
            dPij_num = 4*(R2^2-R1^2)*( P(ii,jj)^2 - R2^2 ).*( X(:,ii) - X(:,jj)  );
            dPij_den = ( P(ii,jj)^2 - R1^2 )^3;
            dPij(ii,:,jj) = dPij_num/dPij_den;
        end
    end
end

U_a(:,1) = -(dPij(1,:,Ni) + dPij(1,:,2))';
for ii = 2:Ni-1
    U_a(:,ii) = -(dPij(ii,:,ii-1) + dPij(ii,:,ii+1))';
end
U_a(:,Ni) = -(dPij(Ni,:,Ni-1) + dPij(Ni,:,1))';