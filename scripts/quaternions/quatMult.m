function q = quatMult(q1,q2)
%%QUATMULT
%   Multiplies two quaternions
%
%   q = QUATMULT(q1,q2) returns the normalized quaternion product
%       of input quaternions q1 and q2
%
%   The product of two quaternions yields a new quaternion composed of the
%   combined rotations. The product is non-commutative (order matters!)
%
%   2018 Enrico Eberhard

q(1) = q1(1)*q2(1)-dot(q1(2:4),q2(2:4));

q(2:4) = q1(1)*q2(2:4) + q2(1)*q1(2:4) + cross(q1(2:4),q2(2:4));

q = normalize(q);

end