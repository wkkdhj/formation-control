function  tt= ddelags(t,y)  %表示时间的函数

global p num
delay=0;
for i=1:1:num-1             %对每个时间段设计时间滞后  
                            %举个例子  在a1-a2时间段 按照文章取的时间为a1
                            %那么当a1+t的时刻（不超出改时间段），若要采样的话  a1+t时刻时间滞后就是 t 
                            %也就是 x（a1+t-t） (t在程序中表示delay) 以此类推
    if t>=p(i) && t<p(i+1)
        delay=t-p(i);
    end
end
tt=t-delay;
end
