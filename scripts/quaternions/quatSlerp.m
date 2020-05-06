function [ q3 ] = quatSlerp( q1, q2, t )
%QUATSLERP quaternion slerp
%   computes the spherical linear quaternion interpolation
%   at a value of t between 0 and 1 between quaternions q1 and q2


% make inputs unit quaternions
q1 = normalize(q1);
q2 = normalize(q2);


one = 1.0 - eps;
d = q1'*q2;
absD = abs(d);

if(absD >= one)
    scale0 = 1 - t;
    scale1 = t;
else
    % theta is the angle between the 2 quaternions
    theta = acos(absD);
    sinTheta = sin(theta);
    
    scale0 = sin( ( 1.0 - t ) * theta) / sinTheta;
    scale1 = sin( ( t * theta) ) / sinTheta;
end
if(d < 0)
    scale1 = -scale1;
end

q3 = scale0 * q1 + scale1 * q2;
q3 = q3 ./ norm(q3);
end
