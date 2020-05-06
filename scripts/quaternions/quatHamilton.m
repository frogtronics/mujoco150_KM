function q = quatHamilton(q1,q2)
%%QUATHAMILTON
%   Multiplies two quaternions via the Hamilton product
%
%   q = QUATHAMILTON(q1,q2) returns the normalized Hamilton product of two
%       input quaternions.
%
%   https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
%
%   2018 Enrico Eberhard

q(1) = q1(1)*q2(1)-dot(q1(2:4),q2(2:4));

q(2) = q1(1)*q2(2) + q2(1)*q1(2) + q1(3)*q2(4) - q1(4)*q2(3);
q(3) = q1(1)*q2(3) + q2(1)*q1(3) + q1(4)*q2(2) - q1(2)*q2(4);
q(4) = q1(1)*q2(4) + q2(1)*q1(4) + q1(2)*q2(3) - q1(3)*q2(2);

q = normalize(q);

end