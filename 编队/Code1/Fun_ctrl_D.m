function  u_lay = Fun_ctrl_D( X,par )
% u_lay = Fun_ctrl_D( X,par ) ��಼�ÿ������㷨����  
% ���u_layΪ����

% ��Ӳ���
d = par.d;                 % �����Ƕȷֲ����
c1 = par.c1;               % ��಼�ֿ���������
c2 = par.c2;               % ��಼�ֿ���������
alpha = par.alpha;         % ��಼�ֿ���������
Ni = length(d);            % ��ӳ�Ա����

alphaI = zeros(Ni,1);      % ��ʼ��
% �������ڶ�Ա��λ��
for ii = 1:Ni-1
   pi0 = X(:,ii);
   pi1 = X(:,ii+1);
   alphaI(ii) = Fun_Vector_Angle(pi0,pi1);  % ������ڼн�
end
pi0 = X(:,Ni);  %
pi1 = X(:,1);
alphaI(Ni) = Fun_Vector_Angle(pi0,pi1);     % ������ڼн�
    
u_dis =  zeros(Ni,1);      % ��ʼ��
% ��಼�ֿ��ƿ���
u_dis(1) = ( alphaI(1) * d(Ni) / ( d(1) + d(Ni) )  - alphaI(Ni) * d(1) / ( d(1) + d(Ni) ) ).^ alpha ;
for ii = 2:length(d)
   u_dis(ii) = ( alphaI(ii) * d(ii-1) / ( d(ii) + d(ii-1) )  - alphaI(ii-1) * d(ii) / ( d(ii) + d(ii-1) ) ).^ alpha ;   
end

u_lay = c1 + u_dis * c2/(2*pi);  % ���������
