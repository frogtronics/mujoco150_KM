function q = quatFromRot(R)
%%QUATFROMROT
%   Quaternion from rotation matrix
%
%   q = quatFromRot(R) takes a 3x3 rotation matrix and returns a quaternion
%       for that rotation.
%
%   Same as quatFromTM, except directly passing 3x3 rotation matrix
%
%   code adapted from:
%   http://www.peterkovesi.com/matlabfns/Rotations/matrix2quaternion.m
%
%   2018 Enrico Eberhard

% Find rotation axis as the eigenvector having unit eigenvalue
% Solve (R-I)v = 0;
[v,d] = eig(R-eye(3));

% The following code assumes the eigenvalues returned are not necessarily
% sorted by size. This may be overcautious on my part.
d = diag(abs(d));   % Extract eigenvalues
[~, ind] = sort(d); % Find index of smallest one
if d(ind(1)) > 0.001   % Hopefully it is close to 0
    warning('Rotation matrix is dubious');
end

axis = v(:,ind(1)); % Extract appropriate eigenvector

if abs(norm(axis) - 1) > .0001     % Debug
    warning('non unit rotation axis');
end

% Now determine the rotation angle
twocostheta = trace(R)-1;
twosinthetav = [R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)]';
twosintheta = axis'*twosinthetav;

theta = atan2(twosintheta, twocostheta);

q = [cos(theta/2); axis*sin(theta/2)]';
    
end