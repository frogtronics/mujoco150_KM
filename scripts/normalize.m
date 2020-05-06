function n = normalize(A)
%%NORMALIZE
%
%   Normalizes a vector or a row of vectors

if all(A==0)
    n = zeros(size(A));
    return
end

n = bsxfun(@rdivide,A,sqrt(sum(A.^2,2)));
n(~isfinite(A)) = 1;
n(isnan(A)) = 0;

end