function sdot=odefind_UGV(t,s)
global A B K Gamma W P
zhongxin=s(31:33);
P1=inv(P);
%% ��Ӷ���
for m=1:10   
      f=[15*sin(t+(m-1)*pi/5);-15*cos(t+(m-1)*pi/5);30*cos(t+(m-1)*pi/5)];
      s_temp1=s(3*m-2:3*m);
      s(3*m-2:3*m)=s_temp1-f-zhongxin; 
end
%% ����ʱ�������
error=ones(30,1);%32-43
for m=1:10        
      s_temp= s(3*m-2:3*m);
      xi=[0;0;0];      
      for n=1:11
          if W(m,n)>0
              if n<=10
                 xi111=(W(m,n))*(s_temp-s(3*n-2:3*n)); 
              else
                  xi111=(W(m,n))*(s_temp); 
              end
               xi= xi+xi111;
          end          
      end     
   error(3*m-2:3*m)=xi;   
end
%% �������Ȩ��
coupling=ones(10,1);
for m=1:10   
    coupling(m)=(error(3*m-2:3*m))'*Gamma*(error(3*m-2:3*m));   
end
  
%% ����rho
rho=zeros(10,1);
for m=1:10
    rho(m)=(error(3*m-2:3*m))'*P1*(error(3*m-2:3*m));
end
%% 
%%%���follower���״̬���̵Ľ���;
sdot=zeros(33,1); 
for m=1:10
     s_temp=s(3*m-2:3*m);
     f=[15*sin(t+(m-1)*pi/5);-15*cos(t+(m-1)*pi/5);30*cos(t+(m-1)*pi/5)]; 
     u=[0;0;0];  
     u=(coupling(m)+rho(m))*K*(error(3*m-2:3*m));
     
     s_temp=s_temp+f+zhongxin;
     sdot(3*m-2:3*m)=A*s_temp+B*u;
end
 
%% 
%%���leader��״̬���̽���
for m=11:11
    sdot(3*m-2:3*m)=A*s(3*m-2:3*m);
end
%%
sdot=[sdot;coupling;error;];

