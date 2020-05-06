function q = quatFromAxAngle(axis,angle)
%%QUATFROMAXANGLE
%   Returns quaternion conjugate
%
%   q = QUATFROMAXANGLE(axis,angle) returns a normalized quaternion from a
%       3-dimensional axis vector and a scalar angle in radians.
%
%   2018 Enrico Eberhard


axis = normalize(axis);

q(1) = cos(angle/2);

q(2:4) = sin(angle/2).*axis;

q = normalize(q);