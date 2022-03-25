function  u_c = Fun_ctrl_C( X,par )
% u_c = Fun_ctrl_C( X,par ) Բ�λ�ۿ������㷨����  

% ��Ӳ���
r = par.r;     % Բ�뾶
x0 = par.x0;   % Բ������x
y0 = par.y0;   % Բ������y

% ���˴�λ����Ϣ
xe = X(1);
ye = X(2);

% ���˴���Բ��λ��
p = sqrt( (xe-x0).^2 + (ye-y0).^2 );

% ���Ʋ���
lamda = par.lamda;
gama = par.gama;

% Բ�λ�ۿ�������
u_c = lamda * [ gama*(r^2-p^2)   -1;  1    gama*(r^2-p^2)  ] * X;   
