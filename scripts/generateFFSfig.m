function generateFFSfig(file, frame, outputfile)

if nargin < 2
    frame = 1;
end


if (nargin < 3) || (outputfile == "") 
    exportQ = false;
else
    exportQ = true;
end

load(file);

sizeFFS = size(space);
nframes = sizeFFS(2);

if frame > nframes
    frame = nframes;
    disp('WARNING- frame chosen is outside of range. Last frame used instead');
else
    frame = frame;
end

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

title('')


%%

formatFigure(ffsFig, 'a4', 'RSOS');%see comments in formatFigure RSOS 
%RSOS is royal society open science
%view(-37.5,30); %3D view
view(90,0); %side
%view(0,90); %top
%view(0,0); %back
if exportQ
    print(outputfile, '-dpng', '-r500');
end
end
