% This file contains some test methods for computationally generating 
% a feasible force space (or alternatively, generating a polytope
% containing the Minkowski sum of vectors)


%% Generate some 2d vectors

v = rand(2,40) - 0.5;

figVec2d = figure(1);
lines = repelem(v,1,3);
lines(:,1:3:end) = 0;
lines(:,3:3:end) = 0;
plot(lines(1,:), lines(2,:));
axis equal;

%% pseudo method (this doesn't really work)
% add vectors if they contribute towards a given direction

figPseudoFFS2d = figure(2);
plot(lines(1,:), lines(2,:));
hold on;
scr = scatter([], [], 'k.');
scp = scatter([], [], 'rx');
hold off;
axis equal;

r = 0:0.1:pi*2;

p = zeros(2,length(r));


phi = atan2(v(2,:),v(1,:));

for ri = 1:length(r)
    
    scr.XData = [scr.XData cos(r(ri))]; scr.YData = [scr.YData sin(r(ri))];
    
    for vi = 1:length(v)
        
        diff = abs(phi(vi) - r(ri));
        while diff > pi
            diff = diff - 2*pi;
        end
        
        %rad2deg(diff)
        
        if diff < pi/2
            p(:,ri) = p(:,ri) + v(:,vi);
        end
    end
    
    scp.XData = [scp.XData p(1,ri)]; scp.YData = [scp.YData p(2,ri)];
end

K = convhull(p');
hold on;
plot(p(1,K), p(2,K));
hold off


%% method brute force
% add all combinations of vectors, then convex hull them

% (will run out of memory with n > 40)

% seriously, don't try to run this

figBruteFFS2d = figure(3);
plot(lines(1,:), lines(2,:));
hold on;
scp = scatter([], [], 'rx');
hold off;
axis equal;


p = zeros(2,1);
    
for vi = 1:length(v)
    
    newp = p + v(:,vi);
    p = [p newp]; %#ok<AGROW>

    scp.XData = p(1,:); scp.YData = p(2,:);
    drawnow; pause(0.5);
end


K = convhull(p');
hold on;
plot(p(1,K), p(2,K));
hold off


%% slightly better method
% add all combinations of vectors and convex hull each step
% remove interior points

% (repeated convex hulling at increasingly large)

figFFS2d = figure(4);
plot(lines(1,:), lines(2,:));
hold on;
scp = scatter([], [], 'rx');
bl = plot(0, 0,'k-');
hold off;
axis equal;


p = zeros(2,1);
    
for vi = 1:length(v)
    
    newp = [p, p + v(:,vi)];
    
    if(vi > 2)
        K = convhull(newp');
        p = newp(:,K);
        bl.XData = newp(1,K); bl.YData = newp(2,K);
    else
        p = newp;
    end

    scp.XData = p(1,:); scp.YData = p(2,:);
    drawnow; pause(0.5);
end



%% Generate some 3d vectors

v = rand(3,50) - 0.5;

figVec3d = figure(5);
lines = repelem(v,1,3);
lines(:,1:3:end) = 0;
lines(:,3:3:end) = 0;
plot3(lines(1,:), lines(2,:), lines(3,:));
axis equal;


%% add all combinations of vectors and convex hull each step
figFFS3d = figure(6);
plot3(lines(1,:), lines(2,:), lines(3,:));
hold on;
scp = scatter3([], [], [], 'r.');
%bl = plot3(0, 0, 0,'k-');
hold off;
axis equal;


p = zeros(3,1);

sz = zeros(1,length(v));
tm = zeros(1,length(v));

tic;
    
for vi = 1:length(v)
    
    newp = [p, p + v(:,vi)];
    
    if(vi > 3)
        K = convhull(newp');
        ptmp = newp(:,K);
        p = [unique(ptmp(1,:), 'stable');
             unique(ptmp(2,:), 'stable');
             unique(ptmp(3,:), 'stable')];
         
        %bl.XData = newp(1,K); bl.YData = newp(2,K);
    else
        p = newp;
    end
    
    %scp.XData = p(1,:); scp.YData = p(2,:); scp.ZData = p(3,:);
    %drawnow; pause(3);
    
    sz(vi) = size(p,2);
    tm(vi) = toc;
end

scp.XData = p(1,:); scp.YData = p(2,:); scp.ZData = p(3,:);



