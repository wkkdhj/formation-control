function sdot=odefind_bipartite(t,s)
%%
global A B C F K L1 d F1 S r P 
%%
zhongxin=s(25:28);  %表示输出反馈后的领导者的位置（速度）信息
zhongxin_o=s(53:56);  %表示输出反馈后的领导者的位置（速度）信息
zhongxin_input_observer=s(88:91);%s(53:56);   %表示输入的中心点的信息   （该点的状态等于zhongxin_o=s(53:56)）
beta=0.2;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  创建迭代的储存单位
%建立实际状态的储存单位
sdot=zeros(28,1);
%建立观测状态的储存单位
odot=zeros(28,1);
%建立couping weight储存单位
cdot=zeros(7,1);   
%建立input observer储存单位
idot=zeros(28,1);  
n1=-1;
n2=-1;
sudu1=0.2;
sudu2=0.3;
rad1=sudu1*10;
rad2=sudu2*10;
for m=1:6
    if d(m)>0
        n1=n1+1;
        f=r*[10*sin(sudu1*t+2*(n1-1)*pi/3);rad1*cos(sudu1*t+2*(n1-1)*pi/3);10*cos(sudu1*t+2*(n1-1)*pi/3);-rad1*sin(sudu1*t+2*(n1-1)*pi/3)];
        gamma=[-rad1*sudu1*sin(sudu1*t+2*(n1-1)*pi/3);-rad1*sudu1*cos(sudu1*t+2*(n1-1)*pi/3)];
    else
        n2=n2+1;
        f=r*[10*sin(sudu2*t+2*(n2-1)*pi/3);rad2*cos(sudu2*t+2*(n2-1)*pi/3);10*cos(sudu2*t+2*(n2-1)*pi/3);-rad2*sin(sudu2*t+2*(n2-1)*pi/3)];
        gamma=[-rad2*sudu2*sin(sudu2*t+2*(n2-1)*pi/3);-rad2*sudu2*cos(sudu2*t+2*(n2-1)*pi/3)];
    end
    s_temp=s(4*m-3:4*m);
    o_temp=s(4*m-3+28:4*m+28);
    input_observer_temp=s(4*m-3+63:4*m+63);
    W_input=zeros(4,1);   %每次运行时更新
    o_sum=zeros(4,1);   
    DDc=[d(m),d(m),1,1]; %DDc=[1,1,d(m),d(m)];   %%定义d的分布
    DD=diag(DDc);
    DDc1=[d(m),1]; %DDc=[1,1,d(m),d(m)];
    DD1=diag(DDc1);
    
    L=[1,1,d(m),d(m)]; %DDc=[1,1,d(m),d(m)];   %%定义d的分布
    LLL=diag(DDc);
   
    for n=1:7
       if m~=n
           if n<7
               input_observer_temp1=abs(L1(m,n))*(input_observer_temp-sign(L1(m,n))*s(4*n-3+63:4*n+63));  %智能体与其中一个智能体比较的差值
               o_temp1=abs(L1(m,n))*(o_temp-sign(L1(m,n))*s(4*n-3+28:4*n+28));
           else
               input_observer_temp1=abs(L1(m,n))*(input_observer_temp-d(m)*DD*zhongxin_o);  %智能体与其中一个智能体比较的差值
               o_temp1=abs(L1(m,n))*(o_temp-d(m)*DD*zhongxin_o);
           end
           W_input=W_input+input_observer_temp1;             %单个智能体与邻近智能体之间的总差值
           o_sum=o_sum+o_temp1;
       end
    end 
         g=W_input'*S*W_input;                    %表示一个中间的替代向量
         ei=o_temp-(s_temp-f); 

         %%%%%%%%%%%%%%%%%%
                 if norm(d(m)*B'*P*DD*o_sum)>=0.0001
                     H1=(d(m)*B'*P*DD*o_sum)/norm(d(m)*B'*P*DD*o_sum);
                 else
                     H1=(d(m)*B'*P*DD*o_sum)/0.0001;
                 end
                 %%%%%%%%%%%%%%%%%%%
                 if norm(d(m)*B'*S*DD*W_input)>=0.0001
                     H2=(d(m)*B'*S*DD*W_input)/norm(d(m)*B'*S*DD*W_input);
                 else
                     H2=d(m)*B'*S*DD*W_input/0.0001;
                 end      
         upbound_input=d(m)*beta*DD1*H1;%
         upbound=d(m)*beta*DD1*H2;%d(m)*

         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
         ui=K*(s(4*m-3+28:4*m+28)-s(4*m-3+63:4*m+63))-upbound_input;  % 1.....控制输入
       
         sdot(4*m-3:4*m)=A*s(4*m-3:4*m)+B*ui+B*gamma;           % 2.....输入矩阵
         odot(4*m-3:4*m)=A*s(4*m-3+28:4*m+28)+B*ui+F1*C*ei;
         cdot(m)=W_input'*(C'*C)*W_input;            %跟随者智能体的耦合系数
         idot(4*m-3:4*m)=A*s(4*m-3+63:4*m+63)+(s(56+m)+g)*F1*C*W_input-B*upbound;  %% it denotes the input observered value
         %%%%%%%%%%%%%%%%%%%%%%%%
end
%%
%%完成leader的状态方程建立
u0=[0;0];
if t<=30
    u0=[0.05;0];
elseif t<=60
    u0=[-0.05;-0.06];
else
    u0=[0;0];
end

% if t<=25
%     u0=[0.05;0];
% elseif t<=50
%     u0=[-0.05;0.05];
% elseif t<=75
%     u0=[0.06;-0.05];
% else
%     u0=[0;0];
% end

% 
% if t<20
%     u0=[0.1;0];
% elseif t<40
%     u0=[-0.1;-0.08];
% elseif t<60
%     u0=[0;0];
% elseif t<80
%     u0=[-0.15;0.08];
% else
%     u0=[0;0];
% end

for m=7:7
    s_temp=s(4*m-3:4*m);
    o_temp=s(4*m-3+28:4*m+28);
    e0=o_temp-s_temp;
    sdot(4*m-3:4*m)=A*s(4*m-3:4*m)+B*u0;
    odot(4*m-3:4*m)=A*s(4*m-3+28:4*m+28)+B*u0+F1*C*e0;%
    idot(4*m-3:4*m)=A*s(4*m-3+63:4*m+63)+B*u0+F1*C*e0;
    cdot(m)=0;
end
%%
sdot=[sdot;odot;cdot;idot];
