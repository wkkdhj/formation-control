clc;clear;close all;
tic
global A B K Gamma W P
A=[ 0  1  1;
    1  2  1; 
    -2 -6 -3];
B=[ 0 1; 
    -1 0;
    0 0;];
%% 求控制协议反馈增益矩阵
setlmis ([]);      %开始构建LMI
P = lmivar(1,[3,1]);  %声明LMI中待求矩阵信息
lmiterm([1 1 1 P],A,1,'s');%
lmiterm([1 1 1 0],-(2*B*B'));%
lmiterm([2 1 1 P],-1,1);%
lmisys = getlmis;  %LMI构建结束
[tmin1,xfeas] = feasp(lmisys);  %求解LMI
P = dec2mat(lmisys,xfeas,P); %输出求解结果
K=-B'*inv(P)
Gamma=inv(P)*B*B'*inv(P)
%% 
 W=[0  0  0  1  1  0  0  0  0  0  1 ;%邻接矩阵
    1  0  0  0  0  0  0  0  0  0  0;
    1  1  0  0  0  0  0  0  0  0  1;
    1  0  0  0  1  0  0  0  0  0  0;
    0  0  0  1  0  0  0  0  0  0  0;
    0  0  0  0  1  0  0  0  0  0  0;
    0  1  0  0  0  0  0  0  0  0  0;
    0  1  0  0  0  0  1  0  0  0  0;
    0  0  1  0  0  0  0  0  0  0  0;
    0  0  1  0  0  0  0  0  0  0  0;
    0  0  0  0  0  0  0  0  0  0  0];
xyz0=[0  -4  4,  4  -4  4 ,  0  4  4 ,  4  4  4 ,   -4 -4 0,  4  -4  0 ,  -4  4  0 ,  4  4  0 ,  0  -4  0,   0  4  0, 0  0  0]; %1-33
couping=ones(1,10)*1;%32-43
error=ones(1,30)*1;%44-73
xyz0=[xyz0 couping error];
tspan=[0,80];
[time,state_value]=ode45(@odefind_UGV,tspan,xyz0);
toc

zong=state_value;
sum_x1=[];
sum_x2=[];
sum_x3=[];

for i=1:11
    temp_x1=zong(:,3*i-2);
    temp_x2=zong(:,3*i-1);
    temp_x3=zong(:,3*i);
    sum_x1=[sum_x1 temp_x1];
    sum_x2=[sum_x2 temp_x2];
    sum_x3=[sum_x3 temp_x3];
end

%% 
len=length(time);
a1=1;
a2=2;
a3=len;
len=length(time);
h=0;
if h==0
[n1,numm1]=min(abs(time-50*ones(len,1)));
[n2,numm2]=min(abs(time-30*ones(len,1)));
%% 


%% 轨迹和轨迹快照
 for n=1:len
%  if n==1 || n==len || n==numm1 || n==numm2 
if  n==1
%% %状态快照t=0s
figure(1)
    plot3([sum_x1(n,1:10) sum_x1(n,1)],[sum_x2(n,1:10) sum_x2(n,1)],[sum_x3(n,1:10) sum_x3(n,1)],'-','LineWidth',1.7);
    hold on
    plot3(sum_x1(n,1:3),sum_x2(n,1:3),sum_x3(n,1:3),'cO',sum_x1(n,4:5),sum_x2(n,4:5),sum_x3(n,4:5),'b*',sum_x1(n,6:8),sum_x2(n,6:8),sum_x3(n,6:8),'mv',sum_x1(n,9:10),sum_x2(n,9:10),sum_x3(n,9:10),'gd','LineWidth',2,'MarkerSize',18);
    hold on
    plot3(sum_x1(n,11),sum_x2(n,11),sum_x3(n,11),'pr','LineWidth',2,'MarkerSize',18);
    xlabel('x_{i1}(t)');ylabel('x_{i2}(t)');zlabel('x_{i3}(t)');
    grid on;
   set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.3);
end
 
if  n==numm2
%% %状态快照t=30s
figure(2)
    plot3([sum_x1(n,1:10) sum_x1(n,1)],[sum_x2(n,1:10) sum_x2(n,1)],[sum_x3(n,1:10) sum_x3(n,1)],'-','LineWidth',1.7);
    hold on
    plot3(sum_x1(n,1:3),sum_x2(n,1:3),sum_x3(n,1:3),'cO',sum_x1(n,4:5),sum_x2(n,4:5),sum_x3(n,4:5),'b*',sum_x1(n,6:8),sum_x2(n,6:8),sum_x3(n,6:8),'mv',sum_x1(n,9:10),sum_x2(n,9:10),sum_x3(n,9:10),'gd','LineWidth',2,'MarkerSize',18);
    hold on
    plot3(sum_x1(n,11),sum_x2(n,11),sum_x3(n,11),'pr','LineWidth',2,'MarkerSize',18);
    xlabel('x_{i1}(t)');ylabel('x_{i2}(t)');zlabel('x_{i3}(t)');
    grid on;
   set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.3);
end


if  n==numm1 
%% %状态快照t=50s
figure(3)
    plot3([sum_x1(n,1:10) sum_x1(n,1)],[sum_x2(n,1:10) sum_x2(n,1)],[sum_x3(n,1:10) sum_x3(n,1)],'-','LineWidth',1.7);
    hold on
    plot3(sum_x1(n,1:3),sum_x2(n,1:3),sum_x3(n,1:3),'cO',sum_x1(n,4:5),sum_x2(n,4:5),sum_x3(n,4:5),'b*',sum_x1(n,6:8),sum_x2(n,6:8),sum_x3(n,6:8),'mv',sum_x1(n,9:10),sum_x2(n,9:10),sum_x3(n,9:10),'gd','LineWidth',2,'MarkerSize',18);
    hold on
    plot3(sum_x1(n,11),sum_x2(n,11),sum_x3(n,11),'pr','LineWidth',2,'MarkerSize',18);
    xlabel('x_{i1}(t)');ylabel('x_{i2}(t)');zlabel('x_{i3}(t)');
    grid on;
   set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.3);
end


if  n==len
%% %状态快照t=70s
figure(4)
    plot3([sum_x1(n,1:10) sum_x1(n,1)],[sum_x2(n,1:10) sum_x2(n,1)],[sum_x3(n,1:10) sum_x3(n,1)],'-','LineWidth',1.7);
    hold on
    plot3(sum_x1(n,1:3),sum_x2(n,1:3),sum_x3(n,1:3),'cO',sum_x1(n,4:5),sum_x2(n,4:5),sum_x3(n,4:5),'b*',sum_x1(n,6:8),sum_x2(n,6:8),sum_x3(n,6:8),'mv',sum_x1(n,9:10),sum_x2(n,9:10),sum_x3(n,9:10),'gd','LineWidth',2,'MarkerSize',18);
    hold on
    plot3(sum_x1(n,11),sum_x2(n,11),sum_x3(n,11),'pr','LineWidth',2,'MarkerSize',18);
    xlabel('x_{i1}(t)');ylabel('x_{i2}(t)');zlabel('x_{i3}(t)');
    grid;
   set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.3);
end
 end
 %% 自适应权重
 kkk=zong(1:len,32:43);
%plot(time(1:len),kkk,'LineWidth',1.5);
figure(5)
plot(time(1:len),(kkk(:,1)),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,2),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,3),'--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,4),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,5),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,6),'-.','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,7),'--','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,8),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,9),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,10),'-.','LineWidth',1.5);%'LineWidth',Line
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',15);ylabel('c_{i}','Fontname', 'Times New Roman','FontSize',15);
grid on
legend('c_{1}','c_{2}','c_{3}','c_{4}','c_{5}','c_{6}','c_{7}','c_{8}','c_{9}','c_{10}') 
%% 跟踪误差曲线
eee=zong(1:len,44:73);
figure(6)

plot(time(1:len),eee(:,1),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,2),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,3),'--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,4),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,5),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,6),'-.','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,7),'--','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,8),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,9),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,10),'-.','LineWidth',1.5);%'LineWidth',Line

plot(time(1:len),eee(:,11),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,12),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,13),'--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,14),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,15),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,16),'-.','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,17),'--','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,18),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,19),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,20),'-.','LineWidth',1.5);%'LineWidth',Line


plot(time(1:len),eee(:,21),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,22),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,23),'--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,24),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,25),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,26),'-.','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,27),'--','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,28),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,29),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),eee(:,30),'-.','LineWidth',1.5);%'LineWidth',Line

xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',15);ylabel('\xi_{i}','Fontname', 'Times New Roman','FontSize',15);
grid on
% legend('\xi_{1}','\xi_{2}','\xi_{3}','\xi_{4}','\xi_{5}','\xi_{6}','\xi_{7}','\xi_{8}','\xi_{9}','\xi_{10}') 

%% 求误差的能量曲线
% figure(7)





 
end

%% 智能体轨迹
%     figure(7)%轨迹
%     plot(sum_x(:,1),sum_y(:,1),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
%     hold on
%     plot(sum_x(:,2),sum_y(:,2),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
%     hold on
%     plot(sum_x(:,3),sum_y(:,3),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
%     hold on
%     plot(sum_x(:,4),sum_y(:,4),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
%     hold on
%     plot(sum_x(:,5),sum_y(:,5),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
%     hold on
%     plot(sum_x(:,6),sum_y(:,6),'r-','LineWidth',1.5);
%     grid on
% end
% end
% text(12,3.5,'t=25s','Fontname', 'Times New Roman','FontSize',16,'FontAngle','italic','FontWeight','bold');
% text(24,13,'{t=50s}','Fontname', 'Times New Roman','FontSize',16,'FontAngle','italic','FontWeight','bold');
% text(41,24,'t=80s','Fontname', 'Times New Roman','FontSize',16,'FontAngle','italic','FontWeight','bold');
% axis equal
%% 


%{
figure(7)
for n=1:len
if n==1 || n==len || n==numm1 || n==numm2 %20s 10s
    plot([sum_xv(n,1:6) sum_xv(n,1)],[sum_yv(n,1:6) sum_yv(n,1)],'-','LineWidth',1.7);
    hold on
    plot(sum_xv(n,1:2),sum_yv(n,1:2),'bD',sum_xv(n,3:4),sum_yv(n,3:4),'kV',sum_xv(n,5:6),sum_yv(n,5:6),'mO','LineWidth',1,'MarkerSize',6);
    hold on
    plot(sum_xv(n,7),sum_yv(n,7),'pr','LineWidth',1,'MarkerSize',10);
    hold on
    plot(sum_xv(:,1:6),sum_yv(:,1:6),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
    hold on
    plot(sum_xv(:,7),sum_yv(:,7),'r-','LineWidth',1.5);
%     plot(sum_x(:,1),sum_y(:,1),'-','Color',[0.6 0.9 0.6],'LineWidth',0.3);
%     hold on
%     plot(sum_x(:,2),sum_y(:,2),'-','Color','y','LineWidth',0.3);
%     hold on
%     plot(sum_x(:,3),sum_y(:,3),'-','Color','c','LineWidth',0.3);
%     hold on
%     plot(sum_x(:,4),sum_y(:,4),'-','Color','g','LineWidth',0.3);
%     hold on
%     plot(sum_x(:,5),sum_y(:,5),'-','Color',[1 0.8 0.5],'LineWidth',0.3);
%     hold on
%     plot(sum_x(:,6),sum_y(:,6),'-','Color',[0.6 1 1],'LineWidth',0.5);
%     hold on

    grid on
end
end
axis equal

figure(81)
kkk=zong(1:len,29:43);
%plot(time(1:len),kkk,'LineWidth',1.5);
plot(time(1:len),kkk(:,1),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,2),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,3),'m--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,5),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,7),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,8),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,9),'-.','LineWidth',1.5);%'LineWidth',Line
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',15);ylabel('Adaptive Coupling Weight','Fontname', 'Times New Roman','FontSize',15);
grid on
legend('c_{12},c_{21}','c_{13},c_{31}','c_{10}','c_{24},c_{42}','c_{35},c_{53}','c_{46},c_{64}','c_{56},c_{65}')

figure(91)
grid on
plot(time,sum_x,'-','LineWidth',1);grid on
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('x_{xi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${X}_{i}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);


figure(92)
plot(time,sum_y,'-','LineWidth',1);grid on
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('X_{yi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${Y}_{i}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);

figure(93)
plot(time,sum_xv,'-','LineWidth',1);grid on
axis([0 80 -5 4]);
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('v_{xi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${v}_{Xi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);

figure(94)
plot(time,sum_yv,'-','LineWidth',1);grid on
axis([0 80 -5 4]);
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('v_{yi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${v}_{Yi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
end


state=state_value;
sudu=0.1;
u_all=zeros(2*len,7);
for tnum=1:len
        t=time(tnum);%%表示此刻的时间
if t<30
u0=[0.015; 0.015];
else
u0=[0.01; 0.0015]; 
end

    
    s=state(tnum,:);
    zhongxin=s(25:28)';
    
    for m=1:6   
      f=[5*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);5*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
      gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
      s_temp1=s(4*m-3:4*m)';
      s(4*m-3:4*m)=s_temp1-f-zhongxin; 
    end
    
    % 
        C1=zeros(6,7);
        sum=0;
        for i=1:6
            for j=1:7
                if L(i,j)<0
                    sum=sum+1;
                    C1(i,j)=s(sum+28);
                end
            end
        end
        % 
        
        for m=1:6
        f=[5*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);5*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
        gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
        s_temp=s(4*m-3:4*m)';

        u1=[0;0;0;0];
        u=[0;0;0;0];
        for n=1:7
            if L(m,n)<0
                if n<=6
                    temp1=(-L(m,n))*(C1(m,n)+0)*(s_temp-s(4*n-3:4*n)');                    %the true value
                else
                    temp1=(-L(m,n))*(C1(m,n)+0)*(s_temp);
                end
                u=u+temp1;
            end
        end
        s_temp=s_temp+f+zhongxin;            
        u1=K*u+u0+gamma;
        u_all(2*tnum-1:2*tnum,m)=u1;
        end

    
            %%完成leader的状态方程建立
    for m=7:7
        s_temp=s(4*m-3:4*m)';
        u00=u0;
        u_all(2*tnum-1:2*tnum,m)=u00;
    end
end

u_allx=zeros(len,7);
u_ally=zeros(len,7);
for i=1:len
    u_allx(i,:)=u_all(2*i-1,:);
    u_ally(i,:)=u_all(2*i,:);
end

figure(11)
plot(time,u_allx,'-','LineWidth',1);
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
axis([0 80 -20 25]);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('$u_{xi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
grid on

figure(12)
plot(time,u_ally,'-','LineWidth',1);
axis([0 80 -20 25]);
grid on

legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('$u_{yi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);

%}
