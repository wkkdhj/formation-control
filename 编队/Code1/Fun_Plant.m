function dX = Fun_Plant(X,U)
% dX = Fun_Plant(X,U) ���ض�����ѧģ��

phi = X(3);
u = U(1);
v = U(2);
w = U(3);
dX = [ u*cos(phi)-v*sin(phi) 
       u*sin(phi)+v*cos(phi)
       w ];
   