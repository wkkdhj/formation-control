function sdot=odefind_UGV(t,s)
%%
global A L T B K C W 
% xyz0=1*[-5 0 3 0, 5 0 5 0, 2 0 2 0, 3 0 -3 0, -1 0 -4 0, -3 0 -1 0, 10 0 10 0]; %1-28
% couping=ones(1,13)*1;%29-41
% observer=zeros(1,28); %42-69  66-69
zhongxin=s(66:69);
%%
%%将（点的位置-编队距离值） 
sudu=0.1;
ki=0.5;
if t<30
u0=[0.01; 0.01];
else
u0=[0; 0]; 
end

%u0=[0;0];
for m=1:6   
      f=[10*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);10*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
      gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
      s_temp1=s(4*m-3:4*m);
      s(4*m-3:4*m)=s_temp1-f-zhongxin; 
      o_temp1=s(4*m-3+41:4*m+41);
      s(4*m-3+41:4*m+41)=o_temp1-f-zhongxin;
end
%%
%建立couping weight矩阵
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
%%
%完成follower编队状态方程的建立
sdot=zeros(28,1);  Cdot=zeros(6,7);  odot=zeros(28,1);
for m=1:6
        f=[10*sin(sudu*t+(m-1)*pi/3);1*cos(sudu*t+(m-1)*pi/3);10*cos(sudu*t+(m-1)*pi/3);-1*sin(sudu*t+(m-1)*pi/3)];
        gamma=[-sudu*sin(sudu*t+(m-1)*pi/3);-sudu*cos(sudu*t+(m-1)*pi/3)];
%     f=[0;0;0;0];
%     gamma=[0;0];
    s_temp=s(4*m-3:4*m);
    o_temp=s(4*m-3+41:4*m+41);
    u=[0;0;0;0];
    for n=1:7
        if L(m,n)<0
            if n<=6
                Cdot(m,n)=ki*(-L(m,n))*((o_temp-s(4*n-3+41:4*n+41))'*W*(o_temp-s(4*n-3+41:4*n+41)));   %-(C1(m,n))the coulping weight
                temp1=(-L(m,n))*(C1(m,n)+0)*(o_temp-s(4*n-3+41:4*n+41));                    %the true value
            else
                Cdot(m,n)=ki*(-L(m,n))*((o_temp)'*W*o_temp);
                temp1=(-L(m,n))*(C1(m,n)+0)*(o_temp);
            end
            u=u+temp1;
        end
    end
    if m==1
        ddd=[1.4*cos(8*t);1.3*sin(4*t)];%;0.1*sin(9*t);0.2*cos(8*t)];%
    elseif m==2
        ddd=[2.5*cos(7*t);1.7*sin(9*t)];%;0.2*sin(9*t);0.4*cos(7*t)];
    elseif m==3
        ddd=[1.8*cos(9*t);2.6*sin(6*t)];%;0.6*sin(8*t);0.7*cos(7*t)];
    elseif m==4
        ddd=[1.7*cos(8*t);1.8*sin(9*t)];%;0.8*sin(6*t);0.3*cos(8*t)];
    elseif m==5
        ddd=[1.4*cos(6*t);1.6*sin(8*t)];%;0.2*sin(4*t);0.2*cos(7*t)];
    elseif m==6
        ddd=[1.1*cos(9*t);1.6*sin(8*t)];%;0.5*sin(5*t);0.5*cos(9*t)];
    end
    %ddd=[0;0];
    s_temp=s_temp+f+zhongxin;
    o_temp=o_temp+f+zhongxin;
    ei=s_temp-o_temp;
    X1=s_temp(2); X2=s_temp(4);g=[-X1*(X1^2+X2^2);-X2*(X1^2+X2^2)];
    %g=0.1*[-X1^3;0];
    OX1=o_temp(2); OX2=o_temp(4);og=[-OX1*(OX1^2+OX2^2);-OX2*(OX1^2+OX2^2)];
    %og=0.1*[-OX1^3;0];
    %OX1_leader=zhongxin(2); OX2_leader=zhongxin(4); og_leader=[-OX1_leader*(OX1_leader^2+OX2_leader^2);-OX2_leader*(OX1_leader^2+OX2_leader^2)];
    
    u=K*u-og+u0+gamma;
    sdot(4*m-3:4*m)=A*s_temp+B*u+B*g;   %%实际的状态
    odot(4*m-3:4*m)=A*o_temp+B*u+B*og+1*T*C*ei;%+1*T*ddd;    %%观测的状态
end
%%
%%完成leader的状态方程建立
for m=7:7
    s_temp=s(4*m-3:4*m);
    %ddd=[1.6*cos(6*t);2.4*sin(8*t)];%;0.6*sin(7*t);0.5*cos(8*t)];
    %ddd=[0;0];
    o_temp=s(4*m-3+41:4*m+41);
    o=s_temp-o_temp;  
    X1=s_temp(2); X2=s_temp(4);g=[-X1*(X1^2+X2^2);-X2*(X1^2+X2^2)];
    %g=0.1*[-X1^3;0];
    OX1=o_temp(2); OX2=o_temp(4);og=[-OX1*(OX1^2+OX2^2);-OX2*(OX1^2+OX2^2)];
    %og=0.1*[-OX1^3;0];
    u0=(u0-og);
    sdot(4*m-3:4*m)=A*s(4*m-3:4*m)+B*u0+B*g;
    odot(4*m-3:4*m)=A*s(4*m-3+41:4*m+41)+B*u0+B*og+1*T*C*o;+1*T*ddd;%
end
%%
%将矩阵型的couping weight 转化成矢量型
cdot=zeros(13,1);        
p=0;
for i=1:6
    for j=1:7
        if C1(i,j)~=0
            p=p+1;
            cdot(p)=Cdot(i,j);
        end
    end
end
%%
sdot=[sdot;cdot;odot];