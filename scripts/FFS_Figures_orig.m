%%

load('KM04_HOP_07_EE_FFS.mat');

frame = 1;

ffsFig = figure(1);
ax = gca;
ax.Colormap = colormap('jet');

srf = trisurf(1:3, [0 0 0], [0 0 0], [0 0 0]);
srf.CDataMapping = 'scaled';
srf.FaceColor = 'interp'; srf.EdgeAlpha = 0.1;

axis equal;

xlabel('(lateral -) X (+ medial)')
ylabel('(back -) Y (+ fore)')
zlabel('(down -) Z (+ up)')


%ax.GridAlpha = 0;

c = colorbar; 
c.Label.String = 'Force (N)';


srf.Vertices = space(frame).DT.Points;
srf.Faces = space(frame).K;
srf.CData = space(frame).F;

lmax = max(srf.Vertices) + 0.1;
lmin = min(srf.Vertices) - 0.1;

ax.XLim = [lmin(1) lmax(1)];
ax.YLim = [lmin(2) lmax(2)];
ax.ZLim = [lmin(3) lmax(3)];

ax.CLim = [0 0.5];

title('Feasible forces: left foot, crouched')


%%

formatFigure(ffsFig, 'a6', 'Thesis');

view(90,0); %side
title('Feasible forces: left foot, crouched (side view)')
%exportThesisFigure(ffsFig,4,'FFS_Side','png');

view(0,90); %top
title('Feasible forces: left foot, crouched (top view)')
exportThesisFigure(ffsFig,4,'FFS_Top','png');

view(0,0); %back
title('Feasible forces: left foot, crouched (back view)')
%exportThesisFigure(ffsFig,4,'FFS_Back','png');


%% MIRRORED

load('KM04_HOP_07_EE_FFS_mir.mat');

ffsmirFig = figure(2);
ax = gca;
ax.Colormap = colormap('jet');

srf = trisurf(1:3, [0 0 0], [0 0 0], [0 0 0]);
srf.CDataMapping = 'scaled'; 
srf.FaceColor = 'interp'; srf.EdgeAlpha = 0.1;

axis equal;

xlabel('(left -) X (+ right)')
ylabel('(back -) Y (+ fore)')
zlabel('(down -) Z (+ up)')


%ax.GridAlpha = 0;

c = colorbar;
c.Label.String = 'Force (N)';


srf.Vertices = space(1).DT.Points;
srf.Faces = space(1).K;
srf.CData = space(1).F;

lmax = max(srf.Vertices) + 0.1;
lmin = min(srf.Vertices) - 0.1;

ax.XLim = [lmin(1) lmax(1)];
ax.YLim = [lmin(2) lmax(2)];
ax.ZLim = [lmin(3) lmax(3)];

ax.CLim = [0 1];



%%

formatFigure(ffsmirFig, 'a6', 'Thesis');

view(90,0); %side
title('Feasible forces: both feet, crouched (side view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_Side','png');

view(0,90); %top
title('Feasible forces: both feet, crouched (top view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_Top','png');

view(0,0); %back
title('Feasible forces: both feet, crouched (back view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_Back','png');





%% Mid-Jump MIRRORED

load('KM04_HOP_07_EE_FFS_mir.mat');

frame = 30;

ffsmirFig = figure(2);
ax = gca;
ax.Colormap = colormap('jet');

srf = trisurf(1:3, [0 0 0], [0 0 0], [0 0 0]);
srf.CDataMapping = 'scaled'; 
srf.FaceColor = 'interp'; srf.EdgeAlpha = 0.1;

axis equal;

xlabel('(left -) X (+ right)')
ylabel('(back -) Y (+ fore)')
zlabel('(down -) Z (+ up)')


%ax.GridAlpha = 0;

c = colorbar;
c.Label.String = 'Force (N)';


srf.Vertices = space(frame).DT.Points;
srf.Faces = space(frame).K;
srf.CData = space(frame).F;

lmax = max(srf.Vertices) + 0.1;
lmin = min(srf.Vertices) - 0.1;

ax.XLim = [lmin(1) lmax(1)];
ax.YLim = [lmin(2) lmax(2)];
ax.ZLim = [lmin(3) lmax(3)];

ax.CLim = [0 1.5];



%%

formatFigure(ffsmirFig, 'a6', 'Thesis');

view(90,0); %side
title('Feasible forces: both feet, mid-jump (side view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_MID_Side','png');

view(0,90); %top
title('Feasible forces: both feet, mid-jump (top view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_MID_Top','png');

view(0,0); %back
title('Feasible forces: both feet, mid-jump (back view)')
exportThesisFigure(ffsmirFig,4,'FFSMIR_MID_Back','png');









%%
P = convexHull(space(frame).DT);
TR = triangulation(P,space(frame).DT.Points);

sc = 0.1;

srf.FaceAlpha = 0.0;

RBP=RigidBodyParams(TR); 
X = [RBP.centroid; RBP.centroid + RBP.PAI(:,1)'*RBP.eigs(3)*100];
Y = [RBP.centroid; RBP.centroid + RBP.PAI(:,2)'*RBP.eigs(2)*100];
Z = [RBP.centroid; RBP.centroid + RBP.PAI(:,3)'*RBP.eigs(1)*100];
hold on;
arrow3d(X(:,1), X(:,2), X(:,3), 0.75, 0.05*sc, 0.1*sc, 'r');
arrow3d(Y(:,1), Y(:,2), Y(:,3), 0.75, 0.05*sc, 0.1*sc, 'g');
arrow3d(Z(:,1), Z(:,2), Z(:,3), 0.75, 0.05*sc, 0.1*sc, 'b');
hold off;




%% animate a space

figure(ffsFig);

ax.XLim = [-3 3];
ax.ZLim = [-3 3];
ax.YLim = [-3 3];

while 1
for fr = [1:length(space) length(space)-1:-1:2]
        
    if ~isempty(space(fr).K)
        srf.Vertices = space(fr).DT.Points;
        srf.Faces = space(fr).K;
        srf.CData = space(fr).F;
    end

    %print(sprintf('~/Desktop/SEBMEDIA/ffsmir/side_grf/frame%02i',fr), '-dpng');
    pause(0.1)
    drawnow;
end
end