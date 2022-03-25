%% ���ǰ뾶�仯������
clear
clc

% Ŀ������
p0 = [0 0]';         % Ŀ��Բ��
r = 1;               % �뾶 
pix0 = [-0.2   0.5  -1 
       -0.16   0.5  -1 ];     % ��ʼλ��
pi_ = pix0 - p0;                   % ���λ��   
d = [2*pi/3 2*pi/3 2*pi/3 ];         % �����Ǿ���

% Բ�λ�ۿ���������
lamda = 1;            % �ı���ٶ�
gama = 2.5;           % �ı�뾶�������ٶ� 

% ��಼�ֿ���������
c1 = 2;
c2 = 5;
alpha = 1;

% ������ײ����������
R1 = 0.4;   % ��С����
R2 = 0.8;   % ��ֹ��ײ���þ���
beta = 2;

% ���챻�ض�������ṹ��
par.r = r;           % Բ�뾶
par.x0 = p0(1);      % Բ������x
par.y0 = p0(2);      % Բ������y
par.d =  d;          % �����Ǿ���
par.lamda = lamda;   % Բ�λ�ۿ���������
par.gama = gama;     % Բ�λ�ۿ���������
par.c1 = c1;         % ��಼�ֿ���������
par.c2 = c2;         % ��಼�ֿ���������
par.alpha = alpha;   % ��಼�ֿ���������
par.R1 = R1;         % ������ײ����������
par.R2 = R2;         % ������ײ����������
par.beta = beta;     % ������ײ����������

% ����������ʱ��
tend = 8;              % ������ֹʱ��    
ts = 0.01;             % ����ʱ��
Tout = 0:ts:tend;      % ����ʱ��

Ni = length(pix0);  % ��ӵ�Ԫ���� 
X = pix0;            % ״̬��ʼ��
for ii = 1:length(Tout)      % ʱ��ѭ��
    
    t = Tout(ii);
    if t > 3 && t < 5        % 3~5s�ڰ뾶��1m��Ϊ1.5m
        par.x0 =  p0(1) + 0.001*(ii-300);        % �뾶�仯
        par.y0 =  p0(2) - 0.001*(ii-300);        % �뾶�仯
    end
    
    
    u_d = Fun_ctrl_D( X,par );               % ����಼�ֿ�����������
    U_a = Fun_ctrl_A( X,par );               % ��������ײ������������
    for jj = 1:Ni  % ��ӳ�Աѭ��
        
        u_c = Fun_ctrl_C( X(:,jj),par );           % ������л�ۿ�����
        ui = u_c .* u_d(jj) + beta*U_a(:,jj);      % �������ں�
        X(:,jj) = X(:,jj) + ui*ts;                 % ����λ��
        Ui(:,jj) = ui;
    end
    
    % �������ڶ�Ա��λ��
    alphaI = zeros(Ni,1);      % ��ʼ��
    for jj = 1:Ni-1
        pi0 = X(:,jj);
        pi1 = X(:,jj+1);
        alphaI(jj) = Fun_Vector_Angle(pi0,pi1);  % ������ڼн�
    end
    pi0 = X(:,Ni);  %
    pi1 = X(:,1);
    alphaI(Ni) = Fun_Vector_Angle(pi0,pi1);     % ������ڼн�
    
    % ���������ڻ�ͼ
    alphaIout(:,ii) = alphaI;  % ����Ǽ����Ϣ
    Ui_out(ii,:,:) = Ui;       % ����ÿ������Ŀ�����
    Pi_out(ii,:,:) = X;        % �������λ����Ϣ
    P0out(:,ii) = [par.x0; par.y0];     % ����Բ��������Ϣ
end

% ����仯ͼ
figure
hold on
plot( Tout',Pi_out(:,:,1) )
plot( Tout',Pi_out(:,:,2) )
plot( Tout',Pi_out(:,:,3) )
xlabel('ʱ��/s')
ylabel('����/m') 

% �켣ͼ
figure
hold on
axis equal
for jj = 1:Ni
   plot( Pi_out(:,1,jj)+p0(1),Pi_out(:,2,jj)+p0(2) )
end
xlabel('X/m')
ylabel('Y/m')
axis([-1.5 1.5 -1.5 1.5])

% �Ǽ��仯ͼ
figure
xlabel('X/m')
ylabel('��λ��/rad')
hold on
for jj = 1:Ni
    plot( Tout,alphaIout(jj,:) )
end
plot( [Tout(1) Tout(end)],[d(1) d(1)],'k--' )

% ��Ծ���
for ii = 1:length(Tout)
    dX(ii,1) =  norm( Pi_out(ii,:,1)-Pi_out(ii,:,end) );    % ��Ծ���
    for jj = 2:Ni 
        dX(ii,jj) =  norm( Pi_out(ii,:,jj)-Pi_out(ii,:,jj-1) );    % ��Ծ���
    end
end
figure
plot( Tout', dX)
xlabel('ʱ��/s')
ylabel('��Ծ���/m')

% ������ʾ
Test_Movie

