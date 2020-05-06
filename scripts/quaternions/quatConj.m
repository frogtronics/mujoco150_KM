function q_ = quatConj(q)
%%QUATCONJ
%   Returns quaternion conjugate
%
%   q_ = QUATCONJ(q) returns normalized [q(1) -q(2:4)]
%
%   2018 Enrico Eberhard

q_ = q;

q_(2:4) = q_(2:4) * -1;

q_ = normalize(q_);

end