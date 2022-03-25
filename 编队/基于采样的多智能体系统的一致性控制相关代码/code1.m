clc;clear;close all;
tic
global A L B K p num xyz0 alpha_all
A=[ 0 -1; 
    2  1];
B=[ 1 0; 
    0 2];
K=[1.0073 -2.1716;
   0.7007 -1.4669];

L=[ 3    0  0   -1  -1   -1;  %%Laplacian����
    -1   1  0   0    0	 0;
    -1	-1	2	0	 0	 0;
    -1	0	0	 1	 0	 0;
    -1	0	0	-1	 2	 0;
    0	0	0	0	-1	 1];

period=0.05;  %����ʱ��

shijian=15;   %��ʱ��

num=(shijian/period);  %����ʱ��/����ʱ�䣩ʱ��������
p=[];              %����ձ���
alpha_c=[];        %����ձ���
for i=1:1:num      %���forѭ����ʾ��0-1�ֲ�����ֵ���䵽�����ĸ�ʱ�����
    tmin=(i-1)*period;
    tmax=i*period;
    p=[p tmin];
    
    pnum=rand(1,1);
    if pnum<0.8
        alpha=1;
    else
        alpha=0;
    end
    alpha_c=[alpha_c alpha];
end

pnum=rand(1,1);  %�����һ�εĸ�ֵ����ʾ��0-1�ֲ�����ֵ���䵽�����ĸ�ʱ�����
if pnum<0.8        
     alpha=1;
else
     alpha=0;
end
last_alpha=alpha;
alpha_all=[alpha_c last_alpha];    %���ж�����0-1�ֲ��ĸ�ֵ

xyz0=[-2 1, -4 0 , 3 0, 2 -2, -2 0, 1 1]; %12 ��Ƹ������������ֵ  ǰ������ʾx1��������ֵ  �Դ�����Ϊx2��������ֵ

tspan=[0,shijian];
sol=ddesd(@dde1,@ddelags,@hist,tspan);   %ʱ�ͺ���  
toc


%�����ǻ�ͼ��
zong=sol.y;  %����sol������ȡ����
time=sol.x;
x_column=[zong(1,:);zong(3,:);zong(5,:);zong(7,:);zong(9,:);zong(11,:)];%��ȡ�������x����
y_column=[zong(2,:);zong(4,:);zong(6,:);zong(8,:);zong(10,:);zong(12,:)];%��ȡ�������y����

figure(1)
plot(time',x_column','-');
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',13);
ylabel('$x_{1i}(i=1,2,...,6)$','Interpreter','latex','Fontname','Times New Roman','FontSize',15);
legend('x_{11}','x_{12}','x_{13}','x_{14}','x_{15}','x_{16}','Location','northeast')
figure(2)
plot(time',y_column','-');
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',13);
ylabel('$x_{2i}(i=1,2,...,6)$','Interpreter','latex','Fontname','Times New Roman','FontSize',15);  
legend('x_{21}','x_{22}','x_{23}','x_{24}','x_{25}','x_{26}','Location','northeast')

figure(3)
ttt=0:period:shijian;
plot(ttt,alpha_all,'.')  %��01�ֲ�ͼ
ylim([-0.5,1.5])
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',13);