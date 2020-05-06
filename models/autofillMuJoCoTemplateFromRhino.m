function [sites,sph,cyl] = autofillMuJoCoTemplateFromRhino(templateFile,rhinoFile,outFile)
%%AUTOFILLMUJOCOTEMPLATEFROMRHINO fills an XML template with values from
%   a Rhino reference file for MuJoCo modelling.
%
%   [sites,sph,cyl] = autofillMuJoCoTemplateFromRhino(templateFile,
%                                                       rhinoFile,outFile)
%
%   This function takes three filename inputs: the source template to fill,
%   the Object Information text file from Rhino, and an output file for
%   saving the filled template.
%
%   The output [sites,sph,cyl] is a collection of the object information
%   that was parsed from the Rhino reference file. The sites structure
%   contains the names and XYZ coordinates of all points. The sph structure
%   contains the names, XYZ coordinates and radii of all spheres, and the
%   cyl structre contains the names, XYZ from-to coordinates and radii of
%   all cylinders.
%
%   If the file is on the MATLAB path or in the current MATLAB directory,
%   just the filename will suffice (i.e. 'template.xml'). For files in
%   other locations, use a full or relative file path. Remember that
%   filepath notation depends on the OS.
%   
%   Example full filepath on Mac or Linux:
%       '/Users/enricoeberhard/Documents/template.xml'
%   Example full filepath on Windows:
%       'C:\Users\enricoeberhard\Documents\template.xml'
%
%
%   For more information on MuJoCo models, visit:
%   http://mujoco.org/book/modeling.html



if nargin==0
	templateFile = 'Kassina/KassinaTMP.xml';
    rhinoFile = 'Kassina/KassinaInfo.txt';
    outFile = 'Kassina/Kassina.xml';
end

%extra params
zeroedMesh = true;


filled = '';
template = fileread(templateFile); %#ok<*NASGU>


%%Replace all color references in the template with rgba="r g b a"
%...in future use default classes instead
rgba.code = {'c_LD',    'c_IL',     'c_CI',...
             'c_CS',    'c_II',     'c_IE',...
             'c_CR',     'c_SM',    'c_GR',...
             'c_GL',     'c_ST',    'c_AM',...
             ...
             'c_AL',     'c_SA',    'c_IFB',...
             'c_IFM',    'c_PY',    'c_GE',...
             'c_OE',     'c_PE',    'c_QF',...
             'c_PL'};
         
rgba.value = {'0 0 1 1',    '1 1 0 1',          '0 0.4 0 1',...
              '0 1 0 1',    '0.9 0.6 0.2 1',  '0.8 0 0 1',...
              '0.4 0 0.4 1', '1 0.9 0 1',       '0.2 0.2 0.2 1',...
              '1 0.2 0.6 1', '0 0.8 0 1',       '0.9 1 0.8 1',...
              ...
              '1 0.8 0.9 1', '1 0.2 1 1',       '0.4 0.7 1 1',...
              '0 0 0.8 1',   '0.6 0 0 1',       '0.8 0.8 1 1',...
              '1 0.8 0.6 1', '0.8 0.6 1 1',     '0.6 0.6 1 1',...
              '1 0.5 0 1'};

for musc = 1:length(rgba.code)
    rgbaStr = ['rgba="',rgba.value{musc},'"'];
    template = strrep(template, rgba.code{musc}, rgbaStr);
end


%extract coordinates from the rhino data file
[sites,sph,cyl] = parseDataFromRhino(rhinoFile);


%find the bodies and the hierarchy
[bodies,bodyParents] = findBodyHierarchy(template);
firstJoint = cell(1,length(bodies));

TM = struct;                        
%go through each body
for b = 1:length(bodies)
    body = bodies{b}{1};
    parent = bodyParents{b}{1};
    
    if b < length(bodies)
    %extract the xml section for the body
        exp = strcat('<body name="', body, '".*?<body name=');
        localBody = regexp(template, exp, 'match');
    else %last body
        exp = strcat('<body name="', body, '".*?</body>');
        localBody = regexp(template, exp, 'match');
    end
    modBod = localBody{1};
    
    %extract the header for the body
    exp = strcat('<body name="', body, '".*?>');
    header = regexp(modBod,exp,'match');
    
    %check if this body is mirrored
    mirrored = false;
    if (contains(header{1},'mirror'))
        mirrored = true;
        
        %get the mirror params
        [match1, srcBody] = regexp(header{1},'mirror="(\w*)"','match','tokens');
        [match2, mirBody] = regexp(header{1},'across="(\w*)"','match','tokens');

        %mirror frame points of source body across mirror frame
        %and add to existing site structure
        sites = addMirroredFrameSites(srcBody{1}{1},body,...
                                      mirBody{1}{1},TM,sites);
                                  
        %delete the mirror params from header
        modBod = strrep(modBod,match1{1},'');
        modBod = strrep(modBod,match2{1},'');
    end
    
    
    %generate the transform matrix for the body
    TM.(body) = getTMforBody(body,sites);
    
    %if this doesn't work, just use make TM.(body) = eye(4);
    
    
    %check if this body is morphed
    morph = false;
    if (contains(header{1},'morph="'))
        morph = true;
        
        %extract morph factor
        [match, factorStr] = regexp(header{1},'morph="(.*?)"','match','tokens');
        
        factor = str2double(factorStr{1});
        
        %scale TM in Z direction by morph factor
        TM.(body) = TM.(body)*diag([1 1 factor 1]);
        
        
        %search for the meshes defined in this body
        meshes = regexp(modBod,'<geom type="mesh" mesh="(.*?)"','tokens');
        
        %then go to the mesh assets and set scale attribute
        for mesh = meshes
            
            searchstr = sprintf('<mesh file="%s.stl".*?>',mesh{1}{1});
            meshAssets = regexp(template,searchstr,'match');
            
            %%TODO - check if scale attribute already exists
            %if so, overwrite; preserve positive/negative
            
            for asset = meshAssets
                [matchS,scale] = regexp(asset,['scale="(?<x>.*?) '...
                        '(?<y>.*?) (?<z>.*?)"'],'match','names');
            
                if ~isempty(matchS{1}) %scale already exists
                    org = matchS{1}{1};
                    rep = sprintf('scale="%f %f %f"', str2double(scale{1}.x),...
                          str2double(scale{1}.y),str2double(scale{1}.z)*factor);
                            
                    %replace match in asset
                    asset2 = strrep(asset{1},org,rep);
                    
                    %replace asset in template
                            
                    template = strrep(template,asset{1},asset2);        
                else
                    org = asset{1};
                    rep = sprintf('%s scale="0.001 0.001 %f"/>',...
                                        asset{1}(1:end-2), factor/1000);

                    template = strrep(template,org,rep);
                end
            end
            
        end
        
        %delete the morph params from header
        modBod = strrep(modBod,match{1},'');
    end
        
    
    %check if this body is a target
    if contains(header{1},'targetbody="')
        
        %extract target body
        [match1, tBod] = regexp(header{1},'targetbody="(.*?)"','match','tokens');
        
        %get all body positions from the template
        tok = regexp(template, ['body name="(?<b>.*?)" '...
            'pos="(?<x>.*?) (?<y>.*?) (?<z>.*?)"'],'names');
        
        tLoc = [0 0 0];
        if contains(header{1},'targetsite="')
            %extract target site
            [match2, tSite] = regexp(header{1},'targetsite="(.*?)"','match','tokens');
            
            if ~isempty(tSite)&&nameExistsHere(tSite{1},sites)
                %get site coords (global frame)
                tPos = xyzFromRowName(tSite{1},sites);
            
                %find the position of the target in the target body
                tLoc = [tPos 1]*TM.(tBod{1}{1});
            end
            
            modBod = strrep(modBod,match2{1},'');
        end
        
        %find the index of the target body
        ind = findBodyID(tBod{1},bodies);
        
        %add the target body pos
        tOff = [tLoc(1) + str2double(tok(ind).x),...
                tLoc(2) + str2double(tok(ind).y),...
                tLoc(3) + str2double(tok(ind).z)];

        %while parent is not worldbody, continue adding each parent
        % body pos to offset
        
        %find parent of target body
        parent = findBodyParent(tBod{1},bodies,bodyParents);
        while(~strcmp(parent{1},'worldbody'))
            %get the body ID of the parent
            ind = findBodyID(parent{1}, bodies);
            tOff = [tOff(1) + str2double(tok(ind).x),...
                    tOff(2) + str2double(tok(ind).y),...
                    tOff(3) + str2double(tok(ind).z)];
                
            %find parent of parent
            parent = findBodyParent(parent{1},bodies,bodyParents);
        end
        
        %now that the global offset is found, make a string
        posStr = sprintf('pos="%2.6f %2.6f %2.6f"',...
                            tOff(1),tOff(2),tOff(3));
        
        %replace targetsite="" and targetbody="" in header with targetPos
        modBod = strrep(modBod,match1{1},posStr);
        
        %re-extract the header for the body (now that it has been updated)
        exp = strcat('<body name="', body, '".*?>');
        header = regexp(modBod,exp,'match');
    end
    
    
    %move the body position relative to the parent, using the first
    %joint of the child as the intermediate point,
    %unless it's the worldbody or has no TM
    if ~strcmp(parent,'worldbody') && ~all(all(TM.(body)==eye(4)))
        
        %find the name of the joint in this body
        jName = regexp(localBody{1},'<joint.*?name="(\w*)"','tokens');
        
        if isempty(jName) %no joints, this body is rigidly connected
            jName = {strcat(body,'Origin')}; %use the origin instead
        end
        
        firstJoint{b} = jName{1};
        
        jPos = xyzFromRowName(jName{1},sites);
        
        if ~mirrored
            %find the position of the joint in the local body
            jLoc = [jPos 1]*TM.(body);

            %find the position of the joint in the parent body
            jPar = [jPos 1]*TM.(parent);

            %find difference 
            bodyPos = jPar(1:3)-jLoc(1:3);
        else
            %find the position of the joint in the src body
            jLoc = [jPos 1]*TM.(srcBody{1}{1});

            for bb = 1:length(bodies)
                if strcmp(bodies{bb}{1},srcBody{1}{1})
                    srcParent = bodyParents{bb}{1};
                    break
                end
            end
            %find the position of the joint in parent of the src body
            jPar = [jPos 1]*TM.(srcParent);

            %find difference and mirror X 
            bodyPos = jPar(1:3)-jLoc(1:3);
            bodyPos(1) = -bodyPos(1);
        end
        
        org = sprintf('<body name="%s"',body);
        rep = sprintf('%s pos="%2.6f %2.6f %2.6f"',...
                       org,bodyPos(1),bodyPos(2),bodyPos(3));
                        
        modBod = strrep(modBod,org,rep);
        
    elseif isempty(regexp(header{1},'pos=".*?"', 'once'))
        %if it's a root body and doesn't have a defined position,
        %set pos to 0 0 0
        org = sprintf('<body name="%s"',body);
        rep = sprintf('%s pos="0 0 0"',org);
        modBod = strrep(modBod,org,rep);
    end
    
    if ~zeroedMesh
    
        %move and rotate the stl mesh to 0,0,0
        quat = quatFromTM(TM.(body));
        pos = TM.(body)(4,1:3);

        org = sprintf('<geom type="mesh"');
        rep = sprintf(['%s pos="%2.6f %2.6f %2.6f" ',...
                       'quat="%2.6f %2.6f %2.6f %2.6f"'],...
                       org,pos(1),pos(2),pos(3),...
                       -quat(1),quat(2),quat(3),quat(4));
        modBod = strrep(modBod,org,rep);
    
    end


    %fill in the joint and site coords
    for site = sites.names
        
        pos = [xyzFromRowName(site{1},sites) 1];
        
        org = sprintf('name="%s"',site{1});
        
        if ~mirrored
            pos = pos*TM.(body);
            rep = sprintf('%s pos="%2.6f %2.6f %2.6f"',...
                       org,pos(1),pos(2),pos(3));
        else
            pos = pos*TM.(srcBody{1}{1})*diag([-1 1 1 1]);
            rep = sprintf('name="%s_mir" pos="%2.6f %2.6f %2.6f"',...
                       site{1},pos(1),pos(2),pos(3));
        end
        
        modBod = strrep(modBod,org,rep);
    
    end

    
    %add wrapping geometry information
    for sphere = sph.names
        
        pos = [xyzFromRowName(sphere{1},sph) 1]*TM.(body);
        r = radFromRowName(sphere{1},sph);
        
        org = sprintf('name="%s"',sphere{1});
        rep = sprintf(['%s type="sphere" ',...
                       'pos="%2.6f %2.6f %2.6f" size="%2.6f"'],...
                        org,pos(1),pos(2),pos(3),r);
        modBod = strrep(modBod,org,rep);
    
    end
    
    for cylinder = cyl.names
        
        [from,to] = fromtoFromRowName(cylinder{1},cyl);
        from = [from 1]*TM.(body);
        to = [to 1]*TM.(body);
        
        r = radFromRowName(cylinder{1},cyl);
        
        org = sprintf('name="%s"',cylinder{1});
        rep = sprintf(['%s type="cylinder" ',...
                       'fromto="%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f" ',...
                       'size="%2.6f"'],...
                        org,from(1),from(2),from(3),to(1),to(2),to(3),r);
        modBod = strrep(modBod,org,rep);
    
    end

    %finally, replace the section of the template with the modified version
    template = strrep(template,localBody{1},modBod);
    
end

                        

%%Finally, output the result to file

if exist('outFile','var')
    FID = fopen(outFile,'w');
else
    FID = fopen('Autofilled.xml','w');
end

if FID==-1
    error('Could not open output file');
end

fprintf(FID,template);

fclose(FID);

assignin('base','TM',TM);

end






function [sites,sph,cyl] = parseDataFromRhino(rhinoFile)


%%This function takes a text file with points, wrapping cylinders and 
% spheres, and creates a nice set of variables

%%Points start with label "point"
%%Cylinders start with label "extrusion"
%%Spheres start with label "surface"

% point  
%   ID: 3af5750b-b4ef-46e9-ae0c-00949a962d53 (212)
%   Object name: left_IL_org_1
%   Layer name: Reference
%   Render Material: 
%     source = from layer
%     index = -1
%   Geometry:
%     Valid point.
%     Point at (-2.677,0.407,-5.082).


text = fileread(rhinoFile);


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



%%What to look for if SPHERES...
%find "surface" + minimum chars until "Object name: "
%skip the minimum number of characters until 
% "center = (" XVAL "," YVAL "," ZVAL ")"
% "radius = r"

nameExp = 'surface.*?Object name: (?<name>.*?)\s';
xExp = 'center = \x{28}(?<x>.*?),';
yExp = '(?<y>.*?),';
zExp = '(?<z>.*?)\x{29}';
rExp = '.*?radius = (?<r>[0-9.]+)';

%concatenate the expressions
exp = [nameExp,'.*?',xExp,yExp,zExp,rExp];

%find the point names and coordinates using named tokens
spheres = regexp(text,exp,'names');

%put names and x y z coords into nx4 cell array
pointCellArray = squeeze(struct2cell(spheres));

sph.names = pointCellArray(1,:);
sph.xyz = str2double(pointCellArray(2:4,:));
sph.r = str2double(pointCellArray(5,:));



%%What to look for if CYLINDERS...
%find "extrusion" + minimum chars until "Object name: "
%skip the minimum number of characters until 
%Path: (XFROM,YFROM,ZFROM) to (XTO,YTO,ZTO)
%skip the minimum number of characters until 
%radius = r

nameExp = 'extrusion.*?Object name: (?<name>.*?)\s';
fromExp = 'Path: \x{28}(?<xf>.*?),(?<yf>.*?),(?<zf>.*?)\x{29}';
toExp = ' to \x{28}(?<xt>.*?),(?<yt>.*?),(?<zt>.*?)\x{29}';
rExp = '.*?radius = (?<r>[0-9.]+)';


%concatenate the expressions
exp = [nameExp,'.*?',fromExp,toExp,rExp];

%find the point names and coordinates using named tokens
cyls = regexp(text,exp,'names');

%put names and x y z coords into nx4 cell array
pointCellArray = squeeze(struct2cell(cyls));

cyl.names = pointCellArray(1,:);
cyl.from = str2double(pointCellArray(2:4,:));
cyl.to = str2double(pointCellArray(5:7,:));
cyl.r = str2double(pointCellArray(8,:));

end


function [bodies,bodyParents] = findBodyHierarchy(template)

[opens,bodies] = regexp(template,'<body name="(\w*)"',...
                            'start','tokens');
closes = regexp(template,'</body>');
                        

bodyParents = cell(size(bodies));
[bodyParents{:}] = deal({'worldbody'});
levels = zeros(1,length(bodies));
%the first body has to be child of world, so start on 2
levels(1) = 1;
for b = 2:length(bodies) 
    
    levels(b) = b-sum(opens(b)>closes);
    %fprintf('%s level %i, prev %i', bodies{b}{1}, levels(b), levels(b-1));
    
    if levels(b) > levels(b-1)
        bodyParents{b} = bodies{b-1};
    elseif levels(b) == levels(b-1)
        bodyParents{b} = bodyParents{b-1};
    elseif levels(b) < levels(b-1) 
        %find the most recent body with lower level
        p = find(levels==levels(b)-1,1,'last');
        if p < b
            bodyParents{b} = bodies{find(levels==levels(b)-1,1,'last')};
        end
    end
    
    %fprintf(' (parent: %s)\n', bodyParents{b}{1});
    
end

end



%if so, get the source <body><Origin|XYZ>
%mult by TM.parent, negate appropriate coords
%mult by inv(TM.parent). Save into sites struct
%as <localBody><Origin|XYZ>
    
function sites = addMirroredFrameSites(srcBody,dstBody,mirBody,TM,sites)

%find the points called <srcBody><Something>
siteInd = find(contains(sites.names,srcBody));

for site = siteInd
    name = sites.names{site};
    expr = sprintf('%s(.*)',srcBody);
    suffix = regexp(name,expr,'tokens');
    suffix = suffix{1}{1};
    
    if contains(suffix,'_X')
        suffix = strrep(suffix,'_X','X');
    elseif contains(suffix,'X')
        suffix = strrep(suffix,'X','_X');
    end
    
    newname = strcat(dstBody,suffix);
    
    %only do this if there isn't already a site by that name
    if isempty(find(contains(sites.names,newname),1))
        sites.names{end+1} = newname;
    
        xyz = [xyzFromRowName(name,sites) 1];

        newxyz = xyz*TM.(mirBody)*diag([-1 1 1 1])/TM.(mirBody);

        sites.xyz(:,end+1) = newxyz(1:3).*1000'; 
    
   end
    
end

end


function TM = getTMforBody(body,sites)

%if the body ends with "_copy", treat name as without
if endsWith(body,'_copy')
    body = strrep(body,'_copy','');
end

%find the points called <Body><Something>
frameInd = find(contains(sites.names,body));

if length(frameInd)<3
	warning(['Not enough frame points found for body ',...
          body,'!']);
    TM = eye(4); return
end

%get the origin point
O = xyzFromRowName(strcat(body,'Origin'),sites);

%for each, determine the 3d 'octant'
oct = zeros(3,length(frameInd)); ind=1;
for frame = frameInd
    site = sites.names{frame};
    
    ind=find(frameInd==frame);
    
    if contains(site,'X')
        oct(1,ind)=1;
    end
    if contains(site,'_X')
        oct(1,ind)=-1;
    end
    if contains(site,'Y')
        oct(2,ind)=1;
    end
    if contains(site,'_Y')
        oct(2,ind)=-1;
    end
    if contains(site,'Z')
        oct(3,ind)=1;
    end
    if contains(site,'_Z')
        oct(3,ind)=-1;
    end
    
end

%there is now matrix oct that contains the octant of each point

%choose two oct columns that when ANDed have at most 1 one
%and when ORed have 2 one, 1 zero
combinations = nchoosek(1:length(oct),2);

%cycle through combinations
for comb = combinations'
    
    %skip combinations without axis information
    if (sum(abs(oct(:,comb(1)))))&&(sum(abs(oct(:,comb(2)))))
        
        %check that the pair of points is complimentary
        anded = sum(oct(:,comb(1))&oct(:,comb(2)));
        ored = sum(oct(:,comb(1))|oct(:,comb(2)));

        if (anded<=1 && ored==2)
            chosen = comb;
            break
        end
    end
end

if ~exist('chosen','var')
    warning(['No complimentary frame points found for body ',...
          body,'!']);
    TM = eye(4); return
end

%ax1 = oct(:,chosen(1)); ax2 = oct(:,chosen(2));

%find the point on an axis
if sum(abs(oct(:,chosen(1))))==1
    ax1 = oct(:,chosen(1));
    pl = oct(:,chosen(2));
    pAx1 = normalize(xyzFromRowName(sites.names{frameInd(chosen(1))},...
                     sites)-O);
    pPl = xyzFromRowName(sites.names{frameInd(chosen(2))},sites)-O;
elseif sum(abs(oct(:,chosen(2))))==1
    ax1 = oct(:,chosen(2));
    pl = oct(:,chosen(1));
    pAx1 = normalize(xyzFromRowName(sites.names{frameInd(chosen(2))},...
                                                                sites)-O);
    pPl = xyzFromRowName(sites.names{frameInd(chosen(1))},sites)-O;
else
    warning(['No single axis frame points found for body ',...
          body,'!']);
    TM = eye(4); return
end


%orient ax1 to be positive
pAx1 = pAx1.*sum(ax1);
ax1 = ax1*sum(ax1);


%check what order to cross them to get a positive normal
if sum(cross(ax1,pl))==1
    ax2 = cross(ax1,pl);
    pAx2 = normalize(cross(pAx1,pPl));
else
    ax2 = cross(pl,ax1);
    pAx2 = normalize(cross(pPl,pAx1));
end

%do the same to get the third normal
if sum(cross(ax1,ax2))==1
    ax3 = cross(ax1,ax2);
    pAx3 = normalize(cross(pAx1,pAx2));
else
    ax3 = cross(ax2,ax1);
    pAx3 = normalize(cross(pAx2,pAx1));
end


R = zeros(3,3);

%assign each axis to the rotation matrix
R(:,ax1==1) = pAx1;
R(:,ax2==1) = pAx2;
R(:,ax3==1) = pAx3;
    
%translate to the origin
T = -O*R;

%create the matrix
TM = [[R zeros(3,1)];[T 1]];

end


function TM = getTransformMatrix(O, X, Z)
%%Specify an origin point, a point on the X axis and
% a point on the Z axis (relative to origin), and get
% a transformation matrix

%find axes
XAxis = normalize(X - O);

ZAxis = normalize(Z - O);

YAxis = normalize(cross(ZAxis,XAxis));

%create rotation and translation matrix, combine into transformation mat
R = [XAxis',YAxis',ZAxis'];
T = -O*R;

TM = [[R zeros(3,1)];[T 1]];

end

function [O, X, Z] = getAxesFromCoplanarVecs(O,XZ,Z)

vec1 = normalize(Z - O);
vec2 = normalize(XZ - O);

Y = normalize(cross(vec1,vec2));

X = O + normalize(cross(Y,vec1));

end

function q = quatFromTM(TM)

%adapted from http://www.peterkovesi.com/matlabfns/Rotations/matrix2quaternion.m

%get rotation elements
R = TM(1:3, 1:3);

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

q = [cos(theta/2); axis*sin(theta/2)];
    
end

function xyz = xyzFromRowName(name,data)

cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
ind = find(cellfun(cellfind(name),data.names),1);

if ind
    xyz = data.xyz(:,ind)'./1000;
else
    xyz = NaN;
end

end

function r = radFromRowName(name,data)

cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
ind = cellfun(cellfind(name),data.names);

r = data.r(:,ind)'./1000;

end

function [from,to] = fromtoFromRowName(name,data)

cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
ind = cellfun(cellfind(name),data.names);

from = data.from(:,ind)'./1000;
to = data.to(:,ind)'./1000;

end

function exists = nameExistsHere(name,data)
    cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
    ind = cellfun(cellfind(name),data.names);
    exists = ceil(sum(ind)/length(ind));
end

function ind = findBodyID(body,bodies)
    cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
    ind = find(cellfun(cellfind(body),bodies),1);
end

function [parent,ind] = findBodyParent(body,bodies,bodyParents)
    cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
    ind = find(cellfun(cellfind(body),bodies),1);
    parent = bodyParents{ind};
end