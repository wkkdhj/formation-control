function sdot=dde1(t,s,Z)
global A L B K p num alpha_all shijian
sdot=zeros(12,1);  %�����������ռ�

num=fix(num);      %����ʱ��ε�����
for i=1:1:num-1
    if t>=p(i) && t<p(i+1)
        alpha=alpha_all(i);
    end
end
if t>=p(num)   %�����һ��  ����һ��ʱ��ν���alpha����ȡ
   alpha=alpha_all(length(alpha_all));
end
%����Ϊ����ȡ��ǰt���ڵ�alpha

for m=1:6      %�������ļ���
    u=[0;0];
    for n=1:6
        if L(m,n)<0
            if n<=6
                temp1=(-L(m,n))*(Z(2*m-1:2*m,1)-Z(2*n-1:2*n,1));  %Z��a,1���еĶ����ʾddelags������Ƶ�ʱ��
                                                                  %��ʾ������x�Ĳ�����
            end
            u=u+temp1;
        end
    end
    sdot(2*m-1:2*m)=A*s(2*m-1:2*m)+alpha*B*K*u;   %%ʵ�ʵ�״̬
end
%sdot(2*m-1:2*m) ��ʾxi������������΢�ֺ����ĸ�ֵ