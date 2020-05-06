function [P, DT, K, v] = vectorspace(V)
%vectorspace
%   
%   For an DxN input of vectors v, return approximately
%   N^2 points (of dimensionality D) that represent
%   the bounding volume of all vector combinations.
%
%   D must be 2 or 3.
%
%   [P, DT, K] = vectorspace(V) also returns a convex hull
%   triangulation DT with vertex indices K.
%
%   Usage:
%       trisurf(K,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3))
%
%
%   [P, DT, K, v] = vectorspace(V) also returns volume of
%   generated polytope v
%
%   2018 Enrico Eberhard

if nargin==0
    error('Input argument V required')
end

if any(size(V)<2)
    error('Input argument V must have one dimension >= 2')
end

if size(V,1) > size(V,2)
   V = V'; 
end

D = size(V,1);

P = zeros(D,1);

steps = 1;
for v = 1:length(V)
    
    if all(V(:,v)==0)
        continue
    end
    
    newp = [P, P + V(:,v)];
    steps = steps + 1;
    if(steps > D)
        try
            K = convhull(newp');
            P = newp(:,K);
        catch
            P = newp;
        end
    else
        P = newp;
    end

end

if nargout > 1
    wrnstate = warning;
    warning('off','MATLAB:delaunayTriangulation:DupPtsWarnId');
    DT = delaunayTriangulation(P');
    if nargout >= 3
        [K,v] = convexHull(DT);
    end
    warning(wrnstate);
end


end