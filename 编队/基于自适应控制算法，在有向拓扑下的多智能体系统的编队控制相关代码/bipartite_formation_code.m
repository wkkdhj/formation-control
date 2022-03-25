clc;clear;close all;
tic
global A B C F K L1 d F1 S r P 
A=[ 0 1 0 0; 
    0 0 0 0; 
    0 0 0 1; 
    0 0 0 0];
B=[ 0 0; 
    1 0; 
    0 0;
    0 1];
%  C=[1 0 0 0;
%     0 0 1 0];
C=[1 1 0 0;
   0 0 1 1];
%  F=-1*[1 0;
%        0 1];         %F2=[-3 -4 0 0;% 0 0 -2 -2];
F=-0*[1 0;
      0 1];  
r=1;


setlmis ([]);      %开始构建LMI
P = lmivar(1,[4,1]);  %声明LMI中待求矩阵,信息;
%P......1st LMI
lmiterm([1 1 1 P],A+B*F*C,1,'s');%
lmiterm([1 1 1 0],-2*(B*B'));%
%P......2st LMI
lmiterm([2 1 1 P],-1,1);%
%slove LMI
lmisys = getlmis;  %LMI构建结束
[tmin1,xfeas] = feasp(lmisys);  %求解LMI
P = dec2mat(lmisys,xfeas,P); %输出求解结果
P=inv(P);

setlmis ([]);      %开始构建LMI
S = lmivar(1,[4,1]);  %声明LMI中待求矩阵,信息;
%S......1st LMI
lmiterm([1 1 1 S],1,A+B*F*C,'s');%
lmiterm([1 1 1 0],-2*(C'*C));%
%S......2st LMI
lmiterm([2 1 1 S],-1,1);%

lmisys = getlmis;  %LMI构建结束
[tmin2,xfeas] = feasp(lmisys);  %求解LMI
S = dec2mat(lmisys,xfeas,S); %输出求解结果

H1=sort(eig(-(A+B*F*C)'*S-S*(A+B*F*C)+2*C'*C));
H2=sort(eig(-(A+B*F*C)'*P-P*(A+B*F*C)+2*P*B*B'*P));
A=A+B*F*C;
F1=-inv(S)*C';
K=-B'*P;
%eig(P*F1*C*C'*F1'*P)=18*4=72
%k=72/(1.9*0.2968);
%% The later operations are to code the main procedure
tic
L=[ 0   0   0   0   0   0   0;   %laplacian Matrix with structure balance
    3   -1  0   -1  0   0   -1; 
    -1  3   1   0   0	1   0;
    0	1	2	0	0	1	0;
    -1	0	0	2	1	0	0;
    0	0	0	1	1	0	0;
    0	1	-1	0	0	2	0];

L1=- [   1     0     0     0     0     0 -1;
    -1     2     0     0     1     0 0;
    -1     0     2     1     0     0 0;
     0     1     0     3    -1    -1 0;
     0     0     1     0     1     0 0;
     0     0     0     0    -1     1 0]; 
 
L2=[ 3  -1   0  -1   0   0 ; %laplacian Matrix with structure balance
    -1   3   1   0   0	 1 ;
     0	 1	 2	 0	 0	-1 ;
    -1	 0	 0	 2	 1	 0 ;
     0	 0	 0	 1	 1	 0 ;
     0	 1  -1	 0	 0	 2];
 
D=[  1   0  0   0   0   0 ;  %matrix to adjust the laplacian matrix
     0   1  0   0   0	0 ;
     0	 0  1	0	0	0 ;
     0	 0	0	-1	0	0 ;
     0	 0	0	0  -1	0 ;
     0	 0  0	0	0  -1];
 d=[1 1 1 -1 -1 -1];
D*L2*D;
%%四周的follower的坐标（6个）
xy0_agent=3*[0 0 2 0, 0 0 -4 0, 0 0 6 0, 0 0 -2 0, 0 0 4 0, 0 0 -6 0, 4 0 4 0];%1-28_______1-24 follower 25-28 leader
xy0_agent_observer=zeros(1,24);
xy0_agent_observer=[xy0_agent_observer 2 1 2 1 ]; %29-56________29-52 o_follower  53-56 o_leader
couping=ones(1,7)*1;%57-63  coulping rate
input_observer=rand(1,24);
input_observer=[input_observer 2 1 2 1];%64-91

xy0=[xy0_agent, xy0_agent_observer, couping, input_observer];
tspan=[0,80];
[time,state_value]=ode15s(@odefind_bipartite,tspan,xy0);
toc
%%
%以下对数据进行处理  并绘图
zong=state_value;
sum_x=[];
sum_xv=[];
sum_y=[];
sum_yv=[];
o_sum_x=[];
o_sum_xv=[];
o_sum_y=[];
o_sum_yv=[];
for i=1:7
    temp_x=zong(:,4*i-3);
    temp_xv=zong(:,4*i-2);
    temp_y=zong(:,4*i-1);
    temp_yv=zong(:,4*i);
    sum_x=[sum_x temp_x];
    sum_xv=[sum_xv temp_xv];
    sum_y=[sum_y temp_y];
    sum_yv=[sum_yv temp_yv];
    
    o_temp_x=zong(:,4*i-3+28);
    o_temp_xv=zong(:,4*i-2+28);
    o_temp_y=zong(:,4*i-1+28);
    o_temp_yv=zong(:,4*i+28);
    o_sum_x=[o_sum_x o_temp_x];
    o_sum_xv=[o_sum_xv o_temp_xv];
    o_sum_y=[o_sum_y o_temp_y];
    o_sum_yv=[o_sum_yv o_temp_yv];
end


len=length(time);
% 
% figure(1)                 %%%%%%%表示领导者运动的轨迹
% xlabel('X_{i1}');ylabel('X_{i2}');
% title('领导者运动的轨迹');
% hold on
% plot(o_sum_x(len,7)-sum_x(len,7),o_sum_y(len,7)-sum_y(len,7),'.');
% hold on
% axis equal


close all;
figure(2)   %%绘制所有智能体运动的仿真图
axis([-80,80,-55,55]);
plot(sum_x(1,1),sum_y(1,1),'bo','LineWidth',1);
hold on

text(-45,50,'t=80sec','Fontname', 'Times New Roman','FontSize',12,'FontAngle','italic');
text(22,-50,'{t=80sec}','Fontname', 'Times New Roman','FontSize',12,'FontAngle','italic');
text(-45,8,'t=50sec','Fontname', 'Times New Roman','FontSize',12,'FontAngle','italic');
text(22,-8,'t=50sec','Fontname', 'Times New Roman','FontSize',12,'FontAngle','italic');
plot(sum_x(1,2),sum_y(1,2),'kd','LineWidth',1);
hold on
plot(sum_x(1,3),sum_y(1,3),'mv','LineWidth',1);
hold on
plot(sum_x(1,4),sum_y(1,4),'gs','LineWidth',1);
hold on
plot(sum_x(1,5),sum_y(1,5),'c>','LineWidth',1);
hold on
plot(sum_x(1,6),sum_y(1,6),'^','Color',[0.5 0.2 0.2 ],'LineWidth',1);
hold on
plot(sum_x(1,7),sum_y(1,7),'rp','LineWidth',1,'MarkerSize',10);
hold on
%%%%the following denote the final snapshots.
plot(sum_x(len,1),sum_y(len,1),'bo','LineWidth',1);
hold on
plot(sum_x(len,2),sum_y(len,2),'kd','LineWidth',1);
hold on
plot(sum_x(len,3),sum_y(len,3),'mv','LineWidth',1);
hold on
plot(sum_x(len,4),sum_y(len,4),'gs','LineWidth',1);
hold on
plot(sum_x(len,5),sum_y(len,5),'c>','LineWidth',1);
hold on
plot(sum_x(len,6),sum_y(len,6),'^','Color',[0.5 0.2 0.2],'LineWidth',1);
hold on
plot(sum_x(len,7),sum_y(len,7),'rp','LineWidth',1,'MarkerSize',10);
hold on
axis equal

%%%%the following denote the Ts snapshots.
n=1;
b_box=zeros(n,1);
for i=1:n
    if i==1
        t_time=50;
    else
        t_time=65;
    end
[a,b]=min(abs(time-ones(len,1)*t_time));
len1=b;
b_box(i)=b;
hold on
plot(sum_x(len1,1),sum_y(len1,1),'bo','LineWidth',1);
hold on
plot(sum_x(len1,2),sum_y(len1,2),'kd','LineWidth',1);
hold on
plot(sum_x(len1,3),sum_y(len1,3),'mv','LineWidth',1);
hold on
plot(sum_x(len1,4),sum_y(len1,4),'gs','LineWidth',1);
hold on
plot(sum_x(len1,5),sum_y(len1,5),'c>','LineWidth',1);
hold on
plot(sum_x(len1,6),sum_y(len1,6),'^','Color',[0.5 0.2 0.2],'LineWidth',1);
hold on
plot(sum_x(len1,7),sum_y(len1,7),'rp','LineWidth',1,'MarkerSize',10);
hold on
axis([-90,90,-60,60]);
axis equal
end

for i=1:7
    if i<=6
        plot(sum_x(:,i),sum_y(:,i),'--','Color',[1 0.8 0.5],'LineWidth',1,'MarkerSize',3);
        hold on
%         plot(sum_x(1,i),sum_y(1,i),'d','LineWidth',1);
%         hold on
%         plot(sum_x(len,i),sum_y(len,i),'o','LineWidth',1);
%         hold on
        plot([sum_x(len,1) sum_x(len,2)],[sum_y(len,1) sum_y(len,2)],'b-',[sum_x(len,1) sum_x(len,3)],[sum_y(len,1) sum_y(len,3)],'b-',[sum_x(len,2) sum_x(len,3)],[sum_y(len,2) sum_y(len,3)],'b-','LineWidth',2);
        hold on
      for j=1:n
          len1=b_box(j);
        plot([sum_x(len1,1) sum_x(len1,2)],[sum_y(len1,1) sum_y(len1,2)],'g-',[sum_x(len1,1) sum_x(len1,3)],[sum_y(len1,1) sum_y(len1,3)],'g-',[sum_x(len1,2) sum_x(len1,3)],[sum_y(len1,2) sum_y(len1,3)],'g-','LineWidth',2);
        hold on
      end
    else
        plot(sum_x(:,i),sum_y(:,i),'r-','LineWidth',1);
        hold on
        %plot(-sum_x(:,i),-sum_y(:,i),'r--','LineWidth',1);
        %hold on
%         plot(sum_x(1,i),sum_y(1,i),'d','LineWidth',1);
%         hold on
%         plot(sum_x(len,i),sum_y(len,i),'s','LineWidth',1);
        hold on
        plot([sum_x(len,4) sum_x(len,5)],[sum_y(len,4) sum_y(len,5)],'b-',[sum_x(len,5) sum_x(len,6)],[sum_y(len,5) sum_y(len,6)],'b-',[sum_x(len,6) sum_x(len,4)],[sum_y(len,6) sum_y(len,4)],'b-','LineWidth',2);
        axis equal
        hold on
        for j=1:n
            len1=b_box(j);
            plot([sum_x(len1,4) sum_x(len1,5)],[sum_y(len1,4) sum_y(len1,5)],'g-',[sum_x(len1,5) sum_x(len1,6)],[sum_y(len1,5) sum_y(len1,6)],'g-',[sum_x(len1,6) sum_x(len1,4)],[sum_y(len1,6) sum_y(len1,4)],'g-','LineWidth',2);
            hold on
        end
    end
end
xlabel('x_{xi}','Fontname', 'Times New Roman','FontSize',20,'FontAngle','italic');ylabel('x_{yi}','Fontname', 'Times New Roman','FontSize',20,'FontAngle','italic');
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','northeast');
grid on
box on
figure(3)  %绘制观测器的终点图
title('观测器的终点图');
hold on
i=len;
plot(o_sum_x(i,1),o_sum_y(i,1),'ro','LineWidth',1);
hold on
plot(o_sum_x(i,2),o_sum_y(i,2),'ro','LineWidth',1);
hold on
plot(o_sum_x(i,3),o_sum_y(i,3),'go','LineWidth',1);
hold on
plot(o_sum_x(i,4),o_sum_y(i,4),'ro','LineWidth',1);
hold on
plot(o_sum_x(i,5),o_sum_y(i,5),'go','LineWidth',1);
hold on
plot(o_sum_x(i,6),o_sum_y(i,6),'go','LineWidth',1);
hold on
plot(o_sum_x(i,7),o_sum_y(i,7),'rd','LineWidth',1);
hold on
plot(-o_sum_x(i,7),-o_sum_y(i,7),'gd','LineWidth',1);
axis equal


% figure(4)            %%观测器与实际状态的误差--------即判断观测器是否观测成功
% title('观测器与实际状态的误差--X坐标');
% hold on
% for n=1:len
%     n1=-1;
%     n2=-1;
%     for m=1:6
%         t=time(n);
%         if d(m)>0
%             n1=n1+1;
%             f=r*[10*sin(t+2*(n1-1)*pi/3);10*cos(t+2*(n1-1)*pi/3);10*cos(t+2*(n1-1)*pi/3);-10*sin(t+2*(n1-1)*pi/3)];
%         else
%             n2=n2+1;
%             f=r*[10*sin(t+2*(n2-1)*pi/3);10*cos(t+2*(n2-1)*pi/3);10*cos(t+2*(n2-1)*pi/3);-10*sin(t+2*(n2-1)*pi/3)];
%         end
%         if m<=6
%             plot(t,o_sum_x(n,m)+f(1)-sum_x(n,m),'.','LineWidth',1);
%             hold on
%         end
%     end
%     for m=7:7
%         plot(t,o_sum_x(n,m)-sum_x(n,m),'o','LineWidth',1);
%         hold on
%     end
% end

figure(51)        %%%%%%%%%%%表示耦合系数的变化
hold on
plot(time,zong(:,57:62),'-','LineWidth',1);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',20);ylabel('Adaptive Coulping Weights','Fontname', 'Times New Roman','FontSize',17);
grid on
legend('follower{1}','follower{2}','follower{3}','follower{4}','follower{5}','follower{6}','location','southeast')

figure(52) 
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',20);ylabel('Observation Error of All Agents','Fontname', 'Times New Roman','FontSize',17);
hold on
plot(time,abs(sum(o_sum_x+o_sum_xv+o_sum_y+o_sum_yv-(sum_x+sum_xv+sum_y+sum_yv),2)),'-','LineWidth',1.5);
hold on
grid on


% figure(6)        %%%%%%%%%%%表示输入的控制率的变化
% title('输入的控制率的变化');
% hold on
% for i=1:6
%     plot(time,zong(:,64:91),'-','LineWidth',1);
%     hold on
% end

% figure(7)  %绘制观测器的终点图
% title('观测器的终点图——x轴');
% hold on
% for i=1:len
%     plot(time,o_sum_x(:,7:7),'LineWidth',1);
%     hold on
%     plot(time,sum_x(:,7:7),'LineWidth',1);
%     hold on
% end



% figure(8)  %绘制智能体的终点图
% title('编队终点图');
% hold on
% for i=len:len;
%     plot(sum_x(i,1),sum_y(i,1),'ro','LineWidth',1);
%     hold on
%     plot(sum_x(i,2),sum_y(i,2),'ro','LineWidth',1);
%     hold on
%     plot(sum_x(i,3),sum_y(i,3),'go','LineWidth',1);
%     hold on
%     plot(sum_x(i,4),sum_y(i,4),'ro','LineWidth',1);
%     hold on
%     plot(sum_x(i,5),sum_y(i,5),'go','LineWidth',1);
%     hold on
%     plot(sum_x(i,6),sum_y(i,6),'go','LineWidth',1);
%     hold on
%     plot(sum_x(i,7),sum_y(i,7),'d','LineWidth',1);
%     hold on
%     plot(-sum_x(i,7),-sum_y(i,7),'gd','LineWidth',1);
%     axis equal
%     r=r*10;
%     theta=0:pi/100:2*pi;
%     x=r*cos(theta)+sum_x(i,7); y=r*sin(theta)+sum_y(i,7);
%     plot(x,y,'-');
%     hold on; 
%     x=r*cos(theta)-sum_x(i,7); y=r*sin(theta)-sum_y(i,7);
%     plot(x,y,'-');
%     hold on; 
% end
% grid on

% figure(9)  %绘制观测器智能体的终点图
% title('观测器编队终点图');
% hold on
% for i=len:len;
%     plot(o_sum_x(i,1),o_sum_y(i,1),'ro','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,2),o_sum_y(i,2),'ro','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,3),o_sum_y(i,3),'go','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,4),o_sum_y(i,4),'ro','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,5),o_sum_y(i,5),'go','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,6),o_sum_y(i,6),'go','LineWidth',1);
%     hold on
%     plot(o_sum_x(i,7),o_sum_y(i,7),'d','LineWidth',1);
%     hold on
%     plot(-o_sum_x(i,7),-o_sum_y(i,7),'gd','LineWidth',1);
%     axis equal
% end









% figure(10)  %绘制智能体的运动轨迹
% title('智能体的运动轨迹');
% hold on
% for i=len:len
%     j=i ;
% % plot(sum_x(j:i,1)-sum_x(j:i,7),sum_y(j:i,1)-sum_y(j:i,7),'r-','LineWidth',1);
% % hold on
% % plot(sum_x(j:i,2)-sum_x(j:i,7),sum_y(j:i,2)-sum_y(j:i,7),'g-','LineWidth',2);
% % hold on
% % plot(sum_x(j:i,3)+sum_x(j:i,7),sum_y(j:i,3)+sum_y(j:i,7),'b-','LineWidth',1.5);
% % hold on
% % plot(sum_x(j:i,4)-sum_x(j:i,7),sum_y(j:i,4)-sum_y(j:i,7),'y-','LineWidth',0.7);
% % hold on
% plot(sum_x(:,5),sum_y(:,5),'g-','LineWidth',1);
% hold on
% plot(sum_x(:,6),sum_y(:,6),'g-','LineWidth',1);
% hold on
% plot(sum_x(:,7),sum_y(:,7),'d','LineWidth',1);
% hold on
% plot(-sum_x(:,7),-sum_y(:,7),'gd','LineWidth',1);
% axis equal
% end