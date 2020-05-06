function q = quatFromTM(TM)
%%QUATFROMROT
%   Quaternion from transformation matrix
%
%   q = quatFromRot(TM) takes a 4x4 transformation matrix with rotational
%       and translational components and returns a quaternion for just the 
%       rotational component.
%
%   Same as quatFromR, except allows easier passing of 4x4 matrix.
%
%   code adapted from:
%   http://www.peterkovesi.com/matlabfns/Rotations/matrix2quaternion.m
%
%   2018 Enrico Eberhard

%get rotation elements
R = TM(1:3, 1:3);

%get quaternion
q = quatFromRot(R);
    
end