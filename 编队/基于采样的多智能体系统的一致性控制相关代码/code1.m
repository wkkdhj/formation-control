clc;clear;close all;
tic
global A L B K p num xyz0 alpha_all
A=[ 0 -1; 
    2  1];
B=[ 1 0; 
    0 2];
K=[1.0073 -2.1716;
   0.7007 -1.4669];

L=[ 3    0  0   -1  -1   -1;  %%Laplacian矩阵
    -1   1  0   0    0	 0;
    -1	-1	2	0	 0	 0;
    -1	0	0	 1	 0	 0;
    -1	0	0	-1	 2	 0;
    0	0	0	0	-1	 1];

period=0.05;  %采样时间

shijian=15;   %总时间

num=(shijian/period);  %（总时间/采样时间）时间间隔段数
p=[];              %定义空变量
alpha_c=[];        %定义空变量
for i=1:1:num      %这个for循环表示将0-1分布的数值分配到上述的各时间段内
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

pnum=rand(1,1);  %（最后一段的赋值）表示将0-1分布的数值分配到上述的各时间段内
if pnum<0.8        
     alpha=1;
else
     alpha=0;
end
last_alpha=alpha;
alpha_all=[alpha_c last_alpha];    %所有段数的0-1分布的赋值

xyz0=[-2 1, -4 0 , 3 0, 2 -2, -2 0, 1 1]; %12 设计各智能体变量初值  前两个表示x1的两个初值  以此类推为x2的两个初值

tspan=[0,shijian];
sol=ddesd(@dde1,@ddelags,@hist,tspan);   %时滞函数  
toc


%后面是画图的
zong=sol.y;  %根据sol进行提取数据
time=sol.x;
x_column=[zong(1,:);zong(3,:);zong(5,:);zong(7,:);zong(9,:);zong(11,:)];%提取智能体各x数据
y_column=[zong(2,:);zong(4,:);zong(6,:);zong(8,:);zong(10,:);zong(12,:)];%提取智能体各y数据

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
plot(ttt,alpha_all,'.')  %画01分布图
ylim([-0.5,1.5])
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',13);