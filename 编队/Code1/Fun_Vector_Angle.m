function theta = Fun_Vector_Angle(a,b)
% theta = Fun_Vector_Angle(a,b) �����������a��b�н�
% thetaȡֵ��Χ 0 ~ 2*pi

if length(a) ~= 3
    a = [a(:); 0];
    b = [b(:); 0];
end
theta  = atan2( norm(cross(a,b)),dot(a,b) );