function  tt= ddelags(t,y)  %��ʾʱ��ĺ���

global p num
delay=0;
for i=1:1:num-1             %��ÿ��ʱ������ʱ���ͺ�  
                            %�ٸ�����  ��a1-a2ʱ��� ��������ȡ��ʱ��Ϊa1
                            %��ô��a1+t��ʱ�̣���������ʱ��Σ�����Ҫ�����Ļ�  a1+tʱ��ʱ���ͺ���� t 
                            %Ҳ���� x��a1+t-t�� (t�ڳ����б�ʾdelay) �Դ�����
    if t>=p(i) && t<p(i+1)
        delay=t-p(i);
    end
end
tt=t-delay;
end
