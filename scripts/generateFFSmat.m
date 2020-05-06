function generateFFSmat(file, mirror)
%
%   Takes a CSV file of 3xN matrix end-effector forces
%   based on  transformed muscle forces, and saves a .mat 
%   structure of boundary points and triangulation.
%
%   Mirrors data around YZ plane (e.g. use left leg data as both legs)
%   unless second input argument is false.

if nargin < 2
    mirror = true;
end

EE = importdata(file);

if mirror
    EE2 = EE;
    EE2(1:3:end,:) = -EE2(1:3:end,:);
    EE = [EE EE2];
end

n = size(EE,1)/3;

space = struct;
for fr = 1:n
    
    fprintf('Frame %02i of %i\n', fr, n);
    
    [~, DT, K, v] = vectorspace(EE(fr*3-2:fr*3,:));
    
    %#ok<*AGROW>
    space(fr).K = K;
    space(fr).DT = DT;

    %find the magnitude of each vertex
    F = sqrt(sum(DT.Points.^2,2));

    space(fr).Norm = normalize(DT.Points);

    % generate colourdata based on distance
    % scale between 1 and 64 for forces 0 to 1N
    C = F*63 + 1;
    
    space(fr).F = F;
    space(fr).C = C;
    space(fr).v = v;
end

if mirror
    save(strrep(file, '.txt', '_FFS_mir.mat'), 'space')
else
    save(strrep(file, '.txt', '_FFS.mat'), 'space')
end
