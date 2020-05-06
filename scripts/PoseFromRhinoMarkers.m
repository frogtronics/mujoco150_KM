function data = PoseFromRhinoMarkers

text = fileread('ScanPoseMarkers.txt');


%%What to look for if SITES
%find "Object name: " and take all non-whitespace characters following
%skip the minimum number of characters until 
% "Point at (" XVAL "," YVAL "," ZVAL ")."

nameExp = 'point.*?Object name: (?<name>.*?)\s';
xExp = 'Point at \x{28}(?<x>.*?),';
yExp = '(?<y>.*?),';
zExp = '(?<z>.*?)\x{29}';

%concatenate the expressions
exp = [nameExp,'.*?',xExp,yExp,zExp];

%find the point names and coordinates using named tokens
points = regexp(text,exp,'names');

%put names and x y z coords into nx4 cell array
pointCellArray = squeeze(struct2cell(points));

sites.names = pointCellArray(1,:);
sites.xyz = str2double(pointCellArray(2:4,:));


%make sites coords into 1x36 array
data = zeros(1,36);

for marker = 1:numel(sites.names)
    
    %get the number of this marker
    ID = regexp(sites.names{marker},'\D*(\d+)','tokens');
    ID = str2double(ID{1}{1});
    
    data(ID*3 - 2 : ID*3) = sites.xyz(:,marker);
    
end


%% Print the data to output file

text = '';

for d = data
    text = [text sprintf('%d\t',d)];
end

outfile = fopen('ScanPose.dat','w');

if ~outfile
    assignin('base','text',text);
    error('Problem opening output file')
else
    fprintf(outfile,text);
end

fclose(outfile);

