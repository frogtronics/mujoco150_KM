function b = quatMultVec(q,a)
%%QUATMULTVEC
%   Multiplies a quaternion by a vector
%
%   b = QUATMULTVEC(q,a) transforms an input vector a by quaternion
%       rotation q to yield a new output vector b.
%
%   2018 Enrico Eberhard

%swap q and a if in wrong order
if (length(q) == 3) && (length(a) == 4)
    c = a;
    a = q;
    q = c;
end

v = zeros(1,4); v(2:4) = a(1:3);

v = quatMult(quatMult(q,v),q.*[1 -1 -1 -1]);

b = v(2:4);
    
end