function sdot=dde1(t,s,Z)
global A L B K p num alpha_all shijian
sdot=zeros(12,1);  %给予变量储存空间

num=fix(num);      %计算时间段的整数
for i=1:1:num-1
    if t>=p(i) && t<p(i+1)
        alpha=alpha_all(i);
    end
end
if t>=p(num)   %对最后一个  不足一个时间段进行alpha的提取
   alpha=alpha_all(length(alpha_all));
end
%上述为了提取当前t所在的alpha

for m=1:6      %控制器的计算
    u=[0;0];
    for n=1:6
        if L(m,n)<0
            if n<=6
                temp1=(-L(m,n))*(Z(2*m-1:2*m,1)-Z(2*n-1:2*n,1));  %Z（a,1）中的定义表示ddelags中所设计的时滞
                                                                  %表示智能体x的采样量
            end
            u=u+temp1;
        end
    end
    sdot(2*m-1:2*m)=A*s(2*m-1:2*m)+alpha*B*K*u;   %%实际的状态
end
%sdot(2*m-1:2*m) 表示xi的两个变量的微分函数的赋值