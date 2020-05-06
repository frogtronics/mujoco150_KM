function kassinaInputs(FILENAME, SAVEAS)
%generates joint quaternions from markers


%% Specify trial data, model params and save location

if nargin<1
    FILENAME = 'KM04_HOP_07_xyzPOINTS.dat';
    %FILENAME = 'KM04_HOP_03_xyzPOINTS.dat';
    %FILENAME = 'KM08_RUN_09_xyzPOINTS.dat';
    %FILENAME = ['/Volumes/FrogWork/DATA/FrogKinematics/Kassina/'...
    %            'xyzPOINTS/Jumping/KM07_HOP_11_xyzPOINTS.dat'];
end

%%What kind of model is this?

%does this model have two legs?
%if so, copy and mirror hip, knee and ankle joints 
twoLegs = true; 

%how are inputs allocated to joints?
%spine has type 'free', 'ball' or 'none'
spineType = 'free';

%rest have ball or sequence of hinge axes ('x', 'xy', 'xyz')
urosType = 'xy';
iliosType = 'xy';
hipType = 'ball';
kneeType = 'ball';
ankleType = 'ball';
tmtType = 'none';
toes = 0;

if nargin < 2
    %%Specify where to save output file
    SAVEAS = '../input/sampleInput.txt';
end

%correct pelvis so its mean yaw is 0
centerPelv = false;


%trim frames of the kinematics 

trimStart = 25;
trimEnd = 135;


%% Import the data and transform into joint quaternions

dataraw = importdata(FILENAME);

if exist('trimStart','var') && exist('trimEnd','var')
    data = dataraw(trimStart:trimEnd,:);
elseif exist('trimStart','var')
    data = dataraw(trimStart:end,:);
elseif exist('trimEnd','var')
    data = dataraw(1:trimEnd,:);
else
    data = dataraw;
end


%get position and orientation of spine in world
%   (only if spine has free joint)
if strcmp(spineType,'free')

    sacrum = data(:,(12*3-2):(12*3));
    neck = data(:,(11*3-2):(11*3));

    vecSpine = normalize(sacrum-neck);

    %vecSpine = repmat(normalize([1 1 0]),length(sacrum),1);
    
    pSpine = sacrum./1000;

    %start at 0X, 0Y
    pSpine(:,1) = pSpine(:,1) - pSpine(1,1);
    pSpine(:,2) = pSpine(:,2) - pSpine(1,2);
    
    %move Z so foot is on floor
    pSpine(:,3) = pSpine(:,3) + 0.012 - 0.0061 + 0.0007;

    
    %or, lock position
    pSpine = zeros(size(pSpine));

end

%transform data to make X right, Y up, Z back
% and zeroed at spine base
[data_t,TM] = transformFrogData(data);


sacrum = data_t(:,(12*3-2):(12*3));
vent = data_t(:,(5*3-2):(5*3));
hip = data_t(:,(4*3-2):(4*3));
knee = data_t(:,(3*3-2):(3*3));
ankle = data_t(:,(2*3-2):(2*3));
tmt = data_t(:,(1*3-2):(1*3));

%make the urostyle consistently angled down a bit (11.3 degrees)
vecUrostyle = repmat(normalize([0 -0.2 1]),length(sacrum),1);
vecPelvis = normalize(vent-sacrum);
vecFemur = normalize(knee-hip);
vecTibFib = normalize(ankle-knee);
vecTarsal = normalize(tmt-ankle);

%mirror leg vectors (our model has left leg)
vecPelvis(:,1) = 0; %-vecPelvis(:,1); 
vecPelvis = normalize(vecPelvis);
vecFemur(:,1) = -vecFemur(:,1);
vecTibFib(:,1) = -vecTibFib(:,1);
vecTarsal(:,1) = -vecTarsal(:,1);

%intialize vars
frames = size(sacrum,1);
qSpine = zeros(frames,4);
qUros = zeros(frames,4);
qIS = zeros(frames,4);
qHip = zeros(frames,4);
qKnee = zeros(frames,4);
qAnk = zeros(frames,4);
qTMT = zeros(frames,4);
aToe = zeros(frames,1);


%find average pelvis yaw and correct
if centerPelv
    PelvX = asin(mean(vecPelvis(:,1)./sqrt(sum(vecPelvis(:,[1 3]).^2,2))));
    
    R = [cos(PelvX) -sin(PelvX) 0;...
         sin(PelvX)  cos(PelvX) 0;...
         0           0          1];

    vecPelvis = vecPelvis*R;
end
        
%loop through frames find local orientations
for fr = 1:frames
    
    
    %what is local X in global frame?
    
    R = TM(1:3,1:3,fr);
    world.X = [1 0 0] / R;
    world.Z = [0 0 1] / R;

    
    %first find body orientations in global frame (relative to Z axis)
    
    if any(strcmp(spineType,{'free','ball'}))
        
        %orient spine along vecSpine vector
        ax = normalize(cross([0 0 1],vecSpine(fr,:)));
        ang = acos(dot([0 0 1],vecSpine(fr,:)));
        qSpine(fr,:) = quatFromAxAngle(ax,ang);
        
        %correct long axis rotation
        body.X = quatMultVec(qSpine(fr,:),[1 0 0]);
        body.Z = quatMultVec(qSpine(fr,:),[0 0 1]);
        
        n = normalize(cross(body.X,world.X));
        twist = acos(dot(body.X,world.X)) * dot(body.Z,n);

        qSpine(fr,:) = quatMult(qSpine(fr,:),quatFromAxAngle([0 0 1],twist));
        
        qSpine(fr,:) = [1 0 0 0];
        
    end


    %Urostyle orientation 
    ax = normalize(cross([0 0 1],vecUrostyle(fr,:)));
    ang = acos(dot([0 0 1],vecUrostyle(fr,:)));
    qUros(fr,:) = quatFromAxAngle(ax,ang);


    %Pelvic orientation 
    if strfind(FILENAME,'HOP')
        vecPelvis(fr,1) = 0; %eliminate all yaw
    end
    
    ax = normalize(cross([0 0 1],vecPelvis(fr,:)));
    ang = acos(dot([0 0 1],vecPelvis(fr,:)));
    qPelvis = quatFromAxAngle(ax,ang);
    
    if strfind(FILENAME,'RUN')&&0
        %force the pelvis to be flat (no roll)
        pelv.X = quatMultVec(qPelvis,[1 0 0]);
        pelv.Z = quatMultVec(qPelvis,[0 0 1]);

        n = normalize(cross(pelv.X,[1 0 0]));
        roll = acos(dot(pelv.X,[1 0 0])) * dot(pelv.Z,n);

        qPelvis = quatMult(qPelvis,quatFromAxAngle([0 0 1],roll));
    end

    %Femur
    ax = normalize(cross([0 0 1],vecFemur(fr,:)));
    ang = acos(dot([0 0 1],vecFemur(fr,:)));
    qFemur = quatFromAxAngle(ax,ang);

    %TibFib
    ax = normalize(cross([0 0 1],vecTibFib(fr,:)));
    ang = acos(dot([0 0 1],vecTibFib(fr,:)));
    qTibFib = quatFromAxAngle(ax,ang);
    

    %Tarsal
    ax = normalize(cross([0 0 1],vecTarsal(fr,:)));
    ang = acos(dot([0 0 1],vecTarsal(fr,:)));
    qTarsal = quatFromAxAngle(ax,ang);


    
    %now that each global orientation has been found,
    %   adjust long-axis rotations to line up bones
    
    %get Y and Z axis for each bone
    fem.Y = quatMultVec(qFemur,[0 1 0]);
    fem.Z = quatMultVec(qFemur,[0 0 1]);
    tib.Y = quatMultVec(qTibFib,[0 1 0]);
    tib.Z = quatMultVec(qTibFib,[0 0 1]);
    tar.Y = quatMultVec(qTarsal,[0 1 0]);
    tar.Z = quatMultVec(qTarsal,[0 0 1]);
    
    
    %rotate qFemur around FemurZ such that
    %   FemurY is parallel to cross(FemurZ, TibFibZ)
    kneeAx = normalize(cross(fem.Z,tib.Z));
    n = normalize(cross(fem.Y,kneeAx));
    twist = acos(dot(fem.Y,kneeAx)) * dot(fem.Z,n);
    
    qFemur = quatMult(qFemur,quatFromAxAngle([0 0 1],twist));
    
    
    %rotate qTarsal around TarsalZ so that TarsalY is
    %   parallel to cross(TibFibZ, TarsalZ)
    ankAx = normalize(cross(tar.Z,tib.Z));
    n = normalize(cross(tar.Y,ankAx));
    twist = acos(dot(tar.Y,ankAx)) * dot(tar.Z,n);
    
    qTarsal = quatMult(qTarsal,quatFromAxAngle([0 0 1],twist));
    

    %rotate qTibFib around TibFibZ such that
    %   TibFibY is halfway between cross(FemurZ, TibFibZ)
    %                          and cross(TibFibZ, TarZ)
    range = acos(dot(ankAx,kneeAx));
    rangeAx = normalize(cross(ankAx,kneeAx));
    
    current = acos(dot(tib.Y,kneeAx));
    currentAx = normalize(cross(tib.Y,kneeAx));

    twist = dot(rangeAx,tib.Z)*range/2 + dot(currentAx,tib.Z)*current;
    qTibFib = quatMult(qTibFib,quatFromAxAngle([0 0 1],twist));

    
    
    %%now qTarsal is known, find qFoot and qToe

    %start with qFoot pointing straight (along world X) and flat in the world
    qFoot = quatMult(quatConj(qSpine(fr,:)),quatFromAxAngle([0 1 0],pi/2));


    %rotate around world Z so that foot Y is pointing out 30 degrees to Spine -X
    foot.Y = quatMultVec(qFoot,[0 1 0]);
    spine.X_ = [-1 0 0];
    world.Z = quatMultVec(quatConj(qSpine(fr,:)),[0 0 1]);

    n = normalize(cross(foot.Y,spine.X_));
    twist = acos(dot(foot.Y,spine.X_)) * sign(dot(world.Z,n));

    qFoot = quatMult(quatFromAxAngle(world.Z, twist + pi/6),qFoot);


    %angle down 15 degrees
    qFoot = quatMult(qFoot,quatFromAxAngle([0 1 0],pi/12));

    %then set toe angle to be flat on ground
    aToe(fr) = -pi/12;



    
    %then work through chain to find local joint orientations
    qIS(fr,:) = qPelvis;
    qHip(fr,:) = quatMult(quatConj(qIS(fr,:)),qFemur);
    qKnee(fr,:) = quatMult(quatConj(qFemur),qTibFib);
    qAnk(fr,:) = quatMult(quatConj(qTibFib),qTarsal);
    qTMT(fr,:) = quatMult(quatConj(qTarsal),qFoot);

end


%% Filter the quaternions

%filter params
wnd = 10;
b = (1/wnd)*ones(1,wnd);
a = 1;

qPelvFilt = [ones(wnd,4).*qIS(1,:); qIS;...
                    ones(wnd,4).*qIS(end,:)];

qHipFilt  = [ones(wnd,4).*qHip(1,:); qHip;...
                    ones(wnd,4).*qHip(end,:)];

qKneeFilt = [ones(wnd,4).*qKnee(1,:); qKnee;...
                    ones(wnd,4).*qKnee(end,:)];

qAnkFilt  = [ones(wnd,4).*qAnk(1,:); qAnk;...
                    ones(wnd,4).*qAnk(end,:)];
                
qTMTFilt  = [ones(wnd,4).*qTMT(1,:); qTMT;...
                    ones(wnd,4).*qTMT(end,:)];

for ii = 1:4
    qPelvFilt(:,ii) = filter(b,a,qPelvFilt(:,ii));
    qHipFilt(:,ii)  = filter(b,a,qHipFilt(:,ii));
    qKneeFilt(:,ii) = filter(b,a,qKneeFilt(:,ii));
    qAnkFilt(:,ii)  = filter(b,a,qAnkFilt(:,ii));
    qTMTFilt(:,ii)  = filter(b,a,qTMTFilt(:,ii));
end

qIS = qPelvFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
qHip  = qHipFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
qKnee = qKneeFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
qAnk  = qAnkFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
qTMT  = qTMTFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);

if strcmp(spineType,'free')
    qSpineFilt = [ones(wnd,7).*[pSpine(1,:) qSpine(1,:)];...
                    [pSpine qSpine];...
                        ones(wnd,7).*[pSpine(end,:) qSpine(end,:)]];

    for ii = 1:7
        qSpineFilt(:,ii) = filter(b,a,qSpineFilt(:,ii));
    end

    pSpine = qSpineFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),1:3);
    qSpine = qSpineFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),4:7);
    
elseif strcmp(spineType,'ball')
    qSpineFilt = [ones(wnd,4).*qSpine(1,:); qSpine;...
                        ones(wnd,4).*qSpine(end,:)];

    for ii = 1:4
        qSpineFilt(:,ii) = filter(b,a,qSpineFilt(:,ii));
    end
    
    qSpine = qSpineFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
end


%normalize quats

normalize(qSpine);
normalize(qUros);
normalize(qIS);
normalize(qHip);
normalize(qKnee);
normalize(qAnk);
normalize(qTMT);


%% write the orientations to file

% this part depends on the model...
% for ball joints, write quaternions directly
% for sequence of hinge joints, convert to euler

text = ''; %#ok<*AGROW>

%calculate dofs
nq = 0;
for joint = {spineType,urosType,iliosType,hipType,kneeType,ankleType,tmtType}
    switch(joint{1})
        case 'free'
            nq = nq + 7;
        case 'ball'
            nq = nq + 4;
        case 'none'
            ; %#ok<NOSEM>
        otherwise
            nq = nq + length(joint{1});
    end
    if(exist('toes','var'))
        nq = nq + toes;
    end
end

if twoLegs
    for joint = {hipType,kneeType,ankleType,tmtType}
        switch(joint{1})
            case 'free'
                nq = nq + 7;
            case 'ball'
                nq = nq + 4;
            case 'none'
                ; %#ok<NOSEM>
            otherwise
                nq = nq + length(joint{1});
        end
    end
    if(exist('toes','var'))
        nq = nq + toes;
    end
end

phasedVals = zeros(frames,nq);


for fr = 1:frames
    
    phaseFr = fr + round(frames/2);
    if phaseFr > frames
        phaseFr = phaseFr - frames;
    end

    values = [];
    mirVal = [];
    
    %Spine
    if strcmp(spineType,'free')

        values = [values,pSpine(fr,1),pSpine(fr,2),pSpine(fr,3)];

        values = [values,qSpine(fr,1),qSpine(fr,2),qSpine(fr,3),qSpine(fr,4)];

    elseif strcmp(spineType,'ball')

        values = [values,qSpine(fr,1),qSpine(fr,2),qSpine(fr,3),qSpine(fr,4)];
        
    end


    %override urostyle with pelvic orientation for now
    %qUros = qPelv;
    
    %Urostyle
    if strcmp(urosType,'ball')
        values = [values qUros(fr,:)];
    else
        xyz = quat2eul(qUros(fr,:),'xyz');
        for ii = 1:length(urosType)
            values = [values,xyz(ii)];
        end
    end
    
    %Pelvis
    if strcmp(iliosType,'ball')
        values = [values qIS(fr,:)];
    else
        xyz = quat2eul(qIS(fr,:),'xyz');
        for ii = 1:length(iliosType)
            values = [values,xyz(ii)];
        end
    end
    
    %Hip
    if strcmp(hipType,'ball')
        values = [values qHip(fr,:)];
        mirVal = [mirVal qHip(fr,:).*[-1 -1 1 1]];
    else
        xyz = quat2eul(qHip(fr,:));
        mirXYZ = quat2eul(qHip(fr,:).*[-1 -1 1 1]);
        for ii = 1:length(hipType)
            values = [values,xyz(ii)];
            mirVal = [mirVal mirXYZ(ii)];
        end
    end
    
    %Knee
    if strcmp(kneeType,'ball')
        values = [values qKnee(fr,:)];
        mirVal = [mirVal qKnee(fr,:).*[-1 -1 1 1]];
    else
        xyz = quat2eul(qKnee(fr,:),'xyz');
        mirXYZ = quat2eul(qKnee(fr,:).*[-1 -1 1 1]);
        for ii = 1:length(kneeType)
            values = [values,xyz(ii)];
            mirVal = [mirVal mirXYZ(ii)];
        end
    end
    
    %Ankle
    if strcmp(ankleType,'ball')
        values = [values qAnk(fr,:)];
        mirVal = [mirVal qAnk(fr,:).*[-1 -1 1 1]];
    else
        xyz = quat2eul(qAnk(fr,:),'xyz');
        mirXYZ = quat2eul(qAnk(fr,:).*[-1 -1 1 1]);
        for ii = 1:length(ankleType)
            values = [values,xyz(ii)];
            mirVal = [mirVal mirXYZ(ii)];
        end
    end
    
    
    %TMT
    if strcmp(tmtType,'ball')
        values = [values qTMT(fr,:)];
        mirVal = [mirVal qTMT(fr,:).*[-1 -1 1 1]];
    elseif ~strcmp(tmtType,'none')
        xyz = quat2eul(qTMT(fr,:),'xyz');
        mirXYZ = quat2eul(qTMT(fr,:).*[-1 -1 1 1]);
        for ii = 1:length(tmtType)
            values = [values xyz(ii)];
            mirVal = [mirVal mirXYZ(ii)];
        end
    end

    %toes
    if exist('toes','var')
        for ii = 1:toes
            values = [values aToe(fr)];
            mirVal = [mirVal -aToe(fr)];
        end
    end
    
    if strfind(FILENAME,'RUN')
        phasedVals(fr,1:length(values)) = values;
        phasedVals(phaseFr,length(values)+1:end) = mirVal;
    else
        
        if twoLegs %repeat the limb joints with mirrored orientations
            values = [values mirVal];
        end


        %write the values to text
        for v = 1:length(values)-1
            text = [text, sprintf('%2.6f,',values(v))];
        end

        text = [text, sprintf('%2.6f\n',values(end))];
    
    end

end

if strfind(FILENAME,'RUN')%write the values to text
    for fr = 1:frames
        for d = 1:nq-1
            text = [text, sprintf('%2.6f,',phasedVals(fr,d))];
        end
        text = [text, sprintf('%2.6f\n',phasedVals(fr,end))];
    end
end


%% Print the text to output file

outfile = fopen(SAVEAS,'w');

if ~outfile
    assignin('base','text',text);
    error('Problem opening output file')
else
    fprintf(outfile,text);
end

fclose(outfile);
       