clc;clear;close all;
tic
global A L T D M B K C W 
A=[ 0 1 0 0; 
    0 0 0 0; 
    0 0 0 1; 
    0 0 0 0];
B=[ 0 0; 
    1 0; 
    0 0;
    0 1];
M=[1;1];
C=[1 1 0 0;
   0 0 1 1];
D=B;
L1=1; L2=0.05;
n=0; r=-99; u=-100;
X=2*L1*n+2*L2*r;
Y=(L2*u-L1);

setlmis ([]);      %开始构建LMI
S = lmivar(1,[4,1]);  %声明LMI中待求矩阵,信息
lmiterm([1 1 1 S],1,A,'s');%
lmiterm([1 1 1 0],-1*(C'*C));%
lmiterm([1 1 1 0],X);%%%%%%%%%%%%%%%%%%%
lmiterm([1 1 2 S],1,1);%
lmiterm([1 1 2 0],Y);%%%%%%%%%%%%%%%%%%%%%%%%
lmiterm([1 2 1 S],1,1);%
lmiterm([1 2 1 0],Y);%%%%%%%%%%%%%%%%
lmiterm([1 2 2 0],-2*L2);%%%%%%%%%%%%%%%%%%%%%%
lmiterm([-2 1 1 S],1,1);
lmisys = getlmis;  %LMI构建结束
[tmin1,xfeas] = feasp(lmisys);  %求解LMI
S = dec2mat(lmisys,xfeas,S); %输出求解结果

setlmis ([]);      %开始构建LMI
P = lmivar(1,[4,1]);  %声明LMI中待求矩阵,信息
lmiterm([1 1 1 P],A,1,'s');%
lmiterm([1 1 1 0],-2*(B*B'));%
lmiterm([1 1 2 0],3*inv(S)*C');%
lmiterm([1 2 1 0],3*C*inv(S));%
lmiterm([1 2 2 0],-1);%
lmiterm([-2 1 1 P],1,1);
lmisys = getlmis;  %LMI构建结束
[tmin2,xfeas] = feasp(lmisys);  %求解LMI
P = dec2mat(lmisys,xfeas,P); %输出求解结果


T=inv(S)*C';%
pni=inv(P);
W=(pni)*B*B'*pni;
K=-B'*(pni);

ppppp=sort(eig(P));
sssss=(eig(S));
L=[ 3   -1  0   -1  0   0   -1; 
    -1  3   -1   0  0	-1  0;
    0	-1	2	0	0	-1	0;
    -1	0	0	2	-1	0	0;
    0	0	0	-1	1	0	0;
    0	-1	-1	0	0	2	0];
Z=eig(A);
%%四周的follower的坐标（6个）
xyz0=2*[-4 0 4 0, -4 0 0 0, 4 0 0 0, 4 0 -4 0, 2 0 -2 0, -2 0 2 0, 1 0 1 0]; %1-28
couping=ones(1,13)*1;%29-41
observer=0.0*ones(1,28); %42-69

xyz0=[xyz0  couping  observer];
tspan=[0,80];
[time,state_value]=ode45(@odefind_UGV,tspan,xyz0);
toc

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
    
    o_temp_x=zong(:,4*i-3+41);
    o_temp_xv=zong(:,4*i-2+41);
    o_temp_y=zong(:,4*i-1+41);
    o_temp_yv=zong(:,4*i+41);
    o_sum_x=[o_sum_x o_temp_x];
    o_sum_xv=[o_sum_xv o_temp_xv];
    o_sum_y=[o_sum_y o_temp_y];
    o_sum_yv=[o_sum_yv o_temp_yv];
end
len=length(time);
a1=1;
a2=2;
a3=len;
len=length(time);
% figure(2)
% xlabel('X_{i1}');ylabel('X_{i2}')
% hold on
% plot(sum_x(:,7),sum_y(:,7),'b-','LineWidth',1.2);
% hold on


% figure(3)   %%绘制所有智能体运动的仿真图
% title('所有智能体在规定时间内的运动');
% hold on
% for i=1:7
%     if i<=6
%         plot(sum_x(:,i),sum_y(:,i),'-','LineWidth',1);
%         hold on
%         plot(sum_x(1,i),sum_y(1,i),'o','LineWidth',1);
%         hold on
%         plot(sum_x(len,i),sum_y(len,i),'d','LineWidth',1);
%         hold on
%     else
%         plot(sum_x(:,i),sum_y(:,i),'r-','LineWidth',2);
%         hold on
%         plot(sum_x(1,i),sum_y(1,i),'ro','LineWidth',2);
%         hold on
%         plot(sum_x(len,i),sum_y(len,i),'rd','LineWidth',2);
%         hold on
%         axis equal
%     end
% end
% grid on

% figure(4)            %%观测器与实际状态的误差--------即判断观测器是否观测成功
% title('观测器与实际状态的误差--X坐标');


% figure(5)            %%观测器与实际状态的误差--------即判断观测器是否观测成功
% title('观测器与实际状态的误差--Xv坐标');
% hold on
% for n=1:len
%     for m=1:6
%         t=time(n);
%        f=[0;0;0;0];
%         if m<=6
%             plot(t,o_sum_xv(n,m)+f(2)-sum_xv(n,m),'.','LineWidth',1);
%             hold on
%         end
%     end
%     for m=7:7
%         plot(t,o_sum_xv(n,m)-sum_xv(n,m),'o','LineWidth',1);
%         hold on
%     end
% end

h=0;
if h==0
[n1,numm1]=min(abs(time-40*ones(len,1)));
[n2,numm2]=min(abs(time-60*ones(len,1)));

figure(6)
for n=1:len
if n==1 || n==len || n==numm1 || n==numm2 %20s 10s
    plot([sum_x(n,1:6) sum_x(n,1)],[sum_y(n,1:6) sum_y(n,1)],'-','LineWidth',1.7);
    hold on
    plot(sum_x(n,1:2),sum_y(n,1:2),'bo',sum_x(n,3:4),sum_y(n,3:4),'ks',sum_x(n,5:6),sum_y(n,5:6),'mv','LineWidth',1,'MarkerSize',6);
    hold on
    plot(sum_x(n,7),sum_y(n,7),'pr','LineWidth',1,'MarkerSize',10);
    hold on
    plot(sum_x(:,1:6),sum_y(:,1:6),'--','Color',[1 0.7 0.5],'LineWidth',0.3,'MarkerSize',3);
    hold on
    plot(sum_x(:,7),sum_y(:,7),'r-','LineWidth',1.5);
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
text(55,38,'t=40s','Fontname', 'Times New Roman','FontSize',13,'FontAngle','italic','FontWeight','bold');
text(85,68,'{t=60s}','Fontname', 'Times New Roman','FontSize',13,'FontAngle','italic','FontWeight','bold');
text(115,98,'t=80s','Fontname', 'Times New Roman','FontSize',13,'FontAngle','italic','FontWeight','bold');
axis equal

figure(7)
for n=1:len
if n==1 || n==len || n==149 || n==167 %20s 10s
    plot([sum_xv(n,1:6) sum_xv(n,1)],[sum_yv(n,1:6) sum_yv(n,1)],'-','LineWidth',1.7);
    hold on
    plot(sum_xv(n,1:2),sum_yv(n,1:2),'bo',sum_xv(n,3:4),sum_yv(n,3:4),'ks',sum_xv(n,5:6),sum_yv(n,5:6),'mv','LineWidth',1,'MarkerSize',6);
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
%plot3(sum_x(:,1:2),sum_y(:,1:2),sum_z(:,1:2),'g','LineWidth',1,'MarkerSize',12);
%plot3(sum_x(:,3:4),sum_y(:,3:4),sum_z(:,3:4),'r','LineWidth',1,'MarkerSize',12);


figure(81)
kkk=zong(1:len,29:41);
%plot(time(1:len),kkk,'LineWidth',1.5);
plot(time(1:len),kkk(:,1),'--','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,2),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,3),'--','LineWidth',2);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,5),'-','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,6),'-.','LineWidth',1);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,8),'-.','LineWidth',1.5);%'LineWidth',Line
hold on
plot(time(1:len),kkk(:,10),'--','LineWidth',1.5);%'LineWidth',Line
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',15);ylabel('Adaptive Coulping Weights','Fontname', 'Times New Roman','FontSize',15);
grid on
legend('c_{12},c_{21}','c_{14},c_{41}','c_{10}','c_{32},c_{23}','c_{26},c_{62}','c_{36},c_{63}','c_{45},c_{54}')
figure(82)
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',15);ylabel('$||e_0(t)+e(t)||$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',18);
hold on
plot(time,abs(sum(o_sum_x+o_sum_xv+o_sum_y+o_sum_yv-(sum_x+sum_xv+sum_y+sum_yv),2)),'-','LineWidth',1);
hold on
grid on


figure(91)
grid on
plot(time,sum_x,'-','LineWidth',1);grid on
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('x_{xi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${x}_{xi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
figure(92)
plot(time,sum_y,'-','LineWidth',1);grid on
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('x_{yi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${x}_{yi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
figure(93)
plot(time,sum_xv,'-','LineWidth',1);grid on
axis([0 80 -5 4]);
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('v_{xi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${v}_{xi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
figure(94)
plot(time,sum_yv,'-','LineWidth',1);grid on
axis([0 80 -5 4]);
legend('follower1','follower2','follower3','follower4','follower5','follower6','leader','Location','southeast','Orientation','vertical')
%xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('v_{yi}','Fontname', 'Times New Roman','FontSize',15);
xlabel('Time(sec)','Fontname', 'Times New Roman','FontSize',12);ylabel('${v}_{yi}$','Interpreter','latex','Fontname', 'Times New Roman','FontSize',20);
end




state=state_value;
sudu=0.1;
u_all=zeros(2*len,7);
for tnum=1:len
    t=time(tnum);%%表示此刻的时间
    if t<30
        u0=[0.01; 0.01];
    else
        u0=[0; 0];
    end
    
    s=state(tnum,:);
    zhongxin=s(66:69)';
    
    for m=1:6   
      f=[10*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);10*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
      gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
      s_temp1=s(4*m-3:4*m)';
      s(4*m-3:4*m)=s_temp1-f-zhongxin; 
      o_temp1=s(4*m-3+41:4*m+41)';
      s(4*m-3+41:4*m+41)=o_temp1-f-zhongxin;
    end
    
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
    for m=1:6
        f=[10*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);10*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
        gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
        s_temp=s(4*m-3:4*m)';
        o_temp=s(4*m-3+41:4*m+41)';
        u1=[0;0;0;0];
        u=[0;0;0;0];
        for n=1:7
            if L(m,n)<0
                if n<=6
                    temp1=(-L(m,n))*(C1(m,n)+0)*(o_temp-s(4*n-3+41:4*n+41)');                    %the true value
                else
                    temp1=(-L(m,n))*(C1(m,n)+0)*(o_temp);
                end
                u=u+temp1;
            end
        end
        s_temp=s_temp+f+zhongxin;
        o_temp=o_temp+f+zhongxin;  
        
        X1=s_temp(2); X2=s_temp(4);g=[-X1*(X1^2+X2^2);-X2*(X1^2+X2^2)];
        OX1=o_temp(2); OX2=o_temp(4);og=[-OX1*(OX1^2+OX2^2);-OX2*(OX1^2+OX2^2)];
        u1=K*u-og+u0+gamma;
        u_all(2*tnum-1:2*tnum,m)=u1;
    end
    %%完成leader的状态方程建立
    for m=7:7
        s_temp=s(4*m-3:4*m)';
        o_temp=s(4*m-3+41:4*m+41)';
%         s_temp=s_temp+f+zhongxin;
%         o_temp=o_temp+f+zhongxin;  
        X1=s_temp(2); X2=s_temp(4);g=[-X1*(X1^2+X2^2);-X2*(X1^2+X2^2)];
        OX1=o_temp(2); OX2=o_temp(4);og0=[-OX1*(OX1^2+OX2^2);-OX2*(OX1^2+OX2^2)];
        u00=(u0-og0);
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



% saveas(figure(11),'fig11.eps','psc2');
% saveas(figure(12),'fig12.eps','psc2');
% saveas(figure(81),'fig81.eps','psc2');
% saveas(figure(82),'fig82.eps','psc2');
% saveas(figure(91),'fig91.eps','psc2');
% saveas(figure(92),'fig92.eps','psc2');
% saveas(figure(93),'fig93.eps','psc2');
% saveas(figure(94),'fig94.eps','psc2');