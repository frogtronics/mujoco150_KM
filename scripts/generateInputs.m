function generateInputs

formattedLine = ['%2.6f,%2.6f,%2.6f,%2.6f,%2.6f,%2.6f,%2.6f,'...
                                '%2.6f,%2.6f,%2.6f,%2.6f,%2.6f\n' ];
text = '';



switch('data')
    
    case 'smooth'

        outname = 'sampleInput.txt';
        
        qPitch = quatFromAxAngle([1 0 0],pi/12);
        ax = normalize([0 1 0]);
        ang0 = 0;
        angRange = deg2rad(10);
        samples = 200;


        t = 0:2*pi/(samples-1):2*pi;
        angs = ang0 + angRange.*cos(t)./2;
        axes = [zeros(size(t))' sin(t)' cos(t)'];

        q1 = zeros(length(angs),4);
        q2 = zeros(length(angs),4);
        
        for ang = angs
            a = quatFromAxAngle([1 0 0],deg2rad(5));%(ang - ang0)/4; %urostyle angle
            q1 = quatMult(quatFromAxAngle(ax,ang),qPitch);
            q2 = quatMult(quatFromAxAngle(ax,ang*10 - pi/2),...
                    quatFromAxAngle([1 0 0],pi/12));

            text = [text sprintf(formattedLine,...
                   a(1),a(2),a(3),a(4),...
                   q1(1),q1(2),q1(3),q1(4),...
                   q2(1),q2(2),q2(3),q2(4))]; %#ok<*AGROW>
        end
        
    case 'data'
        
        outname = '../input/sampleInput.txt';
        freejoint = true;
        twoLegs = true;
        
        %FILENAME = 'KM04_HOP_03_xyzPOINTS.dat';
        FILENAME = 'KM04_HOP_07_xyzPOINTS.dat';
        %FILENAME = 'KM08_RUN_09_xyzPOINTS.dat';
        
        %FILENAME = ['/Volumes/FrogWork/DATA/FrogKinematics/Kassina/'...
        %            'xyzPOINTS/Jumping/KM07_HOP_11_xyzPOINTS.dat'];
        data = importdata(FILENAME);
        
        %transform data to make X right, Y up, Z back
        [data_t,TM] = transformFrogData(data);
        
        %get position and orientation of spine in world
        %   (only if free joint)
        
        
        if freejoint
            
            sacrum = data(:,(12*3-2):(12*3));
            neck = data(:,(11*3-2):(11*3));
            vecSpine = normalize(sacrum-neck);

            pSpine = sacrum./1000;
            
            pSpine(:,3) = pSpine(:,3) + 0.005;
            
            pSpine = zeros(size(pSpine));
            
        end
       

        sacrum = data_t(:,(12*3-2):(12*3));
        vent = data_t(:,(5*3-2):(5*3));
        hip = data_t(:,(4*3-2):(4*3));
        knee = data_t(:,(3*3-2):(3*3));
        ankle = data_t(:,(2*3-2):(2*3));
        tmt = data_t(:,(1*3-2):(1*3));

        %make the urostyle consistently angled down a bit
        vecUrostyle = repmat(normalize([0 -0.2 1]),length(sacrum),1);
        vecPelvis = normalize(vent-sacrum);
        vecFemur = normalize(knee-hip);
        vecTibFib = normalize(ankle-knee);
        vecFoot = normalize(tmt-ankle);
        
        %intialize vars
        frames = length(sacrum);
        qSpine = zeros(frames,4);
        qUros = zeros(frames,4);
        qPelv = zeros(frames,4);
        qHip = zeros(frames,4);
        qKnee = zeros(frames,4);
        qAnk = zeros(frames,4);

        %loop through frames to generate text output
        for fr = 1:frames
            
            
            if freejoint
                ax = cross([0 0 1],vecSpine(fr,:));
                ang = acos(dot([0 0 1],vecSpine(fr,:)));
                %qSpine(fr,:) = quatFromAxAngle([1 0 0],pi/2);
                qSpine(fr,:) = quatFromAxAngle(ax,ang);
                
                qX = quatFromAxAngle([1 0 0],pi/2);
                
                qSpine(fr,:) = quatMult(qX,qSpine(fr,:));
                
%                 %correct for twist
%                 spine.X = quatMultVec(qSpine(fr,:),[1 0 0]);
%                 twist = acos(dot(spine.X,[1 0 0])) + pi;
%                 qSpine(fr,:) = quatMult(qSpine(fr,:),...
%                                     quatFromAxAngle([0 0 1],twist));
                                
                                
%                 TMfr = TM(:,:,fr);
%                 qSpine(fr,:) = quatMuquatConj(quatFromTM(TMfr));
%                 
%                 qY = [1 0 0 0]; %quatFromAxAngle([0 1 0],pi/2);
%                 qX = quatFromAxAngle([1 0 0],-pi/2);
                
                %qSpine(fr,:) = quatMult(quatMult(qSpine(fr,:),qY),qX);
            end
            
            
            %Urostyle orientation 
            ax = cross([0 0 1],vecUrostyle(fr,:));
            ang = acos(dot([0 0 1],vecUrostyle(fr,:)));
            qUros(fr,:) = quatFromAxAngle(ax,ang);
            
            
            %Pelvic orientation *ignore yaw/roll for now
            ax = [1 0 0];%cross([0 0 1],vecPelvis(fr,:));
            ang = acos(dot([0 0 1],vecPelvis(fr,:)));
            qPelv(fr,:) = quatFromAxAngle(ax,ang);
            

            %for the hip, the marker is the right leg
            %but our model has a left leg... so mirror
            %axis and angle over the zy plane by negating x
            ax = cross([0 0 1],vecFemur(fr,:));
            ang = acos(dot([0 0 1],vecFemur(fr,:)));
            ax(1) = -ax(1);
            qFemur = quatFromAxAngle(ax,-ang);
            
            ax = cross([0 0 1],vecTibFib(fr,:));
            ang = acos(dot([0 0 1],vecTibFib(fr,:)));
            ax(1) = -ax(1);
            qTibFib = quatFromAxAngle(ax,-ang);
            
            ax = cross([0 0 1],vecFoot(fr,:));
            ang = acos(dot([0 0 1],vecFoot(fr,:)));
            ax(1) = -ax(1);
            qFoot = quatFromAxAngle(ax,-ang);
            
            
            
            %rotate qFemur around FemurZ such that
            %   FemurY is parallel to cross(FemurZ, TibFibZ)
            
            fem.Y = quatMultVec(qFemur,[0 1 0]);
            fem.Z = quatMultVec(qFemur,[0 0 1]);
            tib.Y = quatMultVec(qTibFib,[0 1 0]);
            tib.Z = quatMultVec(qTibFib,[0 0 1]);
            foot.Y = quatMultVec(qFoot,[0 1 0]);
            foot.Z = quatMultVec(qFoot,[0 0 1]);
            
            kneeAx = normalize(cross(fem.Z,tib.Z));
            
            twist = acos(dot(kneeAx,fem.Y));
            
            qFemur = quatMult(qFemur,quatFromAxAngle([0 0 1],twist));
            
            
            %rotate qFoot around FootZ so that FootY is
            %   parallel to cross(TibFibZ, FootZ)
            
            ankAx = normalize(cross(foot.Z,tib.Z));
            
            twist = acos(dot(ankAx,foot.Y));
            
            qFoot = quatMult(qFoot,quatFromAxAngle([0 0 1],twist));
            
            
            
            
            %rotate qTibFib around TibFibZ such that
            %   TibFibY is halfway between cross(FemurZ, TibFibZ)
            %                          and cross(TibFibZ, FootZ)
            
            
            range = acos(dot(kneeAx,ankAx));
            current = acos(dot(kneeAx,tib.Y));
            
            rangeAx = normalize(cross(kneeAx,ankAx));
            currentAx = normalize(cross(kneeAx,tib.Y));
            
            twist = dot(rangeAx,tib.Z)*range/2 - dot(currentAx,tib.Z)*current;
            
            qTibFib = quatMult(qTibFib,quatFromAxAngle([0 0 1],twist));
            
            
            %then work through chain to find local joint orientations
            qHip(fr,:) = quatMult(quatConj(qPelv(fr,:)),qFemur);
            qKnee(fr,:) = quatMult(quatConj(qHip(fr,:)),qTibFib);
            qAnk(fr,:) = quatMult(quatConj(qKnee(fr,:)),qFoot);
                
            
            
        end
        
        
        %filter the quats
        
        %filter params
        wnd = 5;
        b = (1/wnd)*ones(1,wnd);
        a = 1;
        
        qPelvFilt = [ones(wnd,4).*qPelv(1,:); qPelv;...
                            ones(wnd,4).*qPelv(end,:)];
                        
        qHipFilt  = [ones(wnd,4).*qHip(1,:); qHip;...
                            ones(wnd,4).*qHip(end,:)];

        qKneeFilt = [ones(wnd,4).*qKnee(1,:); qKnee;...
                            ones(wnd,4).*qKnee(end,:)];

        qAnkFilt  = [ones(wnd,4).*qAnk(1,:); qAnk;...
                            ones(wnd,4).*qAnk(end,:)];
        
        for ii = 1:4
            qPelvFilt(:,ii) = filter(b,a,qPelvFilt(:,ii));
            qHipFilt(:,ii)  = filter(b,a,qHipFilt(:,ii));
            qKneeFilt(:,ii) = filter(b,a,qKneeFilt(:,ii));
            qAnkFilt(:,ii)  = filter(b,a,qAnkFilt(:,ii));
        end
        
        qPelv = qPelvFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
        qHip  = qHipFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
        qKnee = qKneeFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
        qAnk  = qAnkFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),:);
        
        if freejoint
            qSpineFilt = [ones(wnd,7).*[pSpine(1,:) qSpine(1,:)];...
                          [pSpine qSpine];...
                            ones(wnd,7).*[pSpine(end,:) qSpine(end,:)]];
                        
            for ii = 1:7
                qSpineFilt(:,ii) = filter(b,a,qSpineFilt(:,ii));
            end
            
            pSpine = qSpineFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),1:3);
            qSpine = qSpineFilt(wnd+ceil(wnd/2):end-ceil(wnd/2),4:7);
        end
        
        
        
        for fr = 1:frames
        
            %print to file
            
            if freejoint
                
                text = [text, sprintf('%2.6f,%2.6f,%2.6f,',...
                    pSpine(fr,1),pSpine(fr,2),pSpine(fr,3))];
                
                %Spine orientation
                %ax = cross([0 0 1],vecSpine(fr,:));
                %ang = acos(dot([0 0 1],vecSpine(fr,:)));
                %qSpine = quatFromAxAngle(ax,ang);
                
                text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                    qSpine(fr,1),qSpine(fr,2),qSpine(fr,3),qSpine(fr,4))];
                
            end
            
            
            %override urostyle with pelvic orientation for now
            qUros = qPelv;
            
            
            
            text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                    qUros(fr,1),qUros(fr,2),qUros(fr,3),qUros(fr,4))];
            text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                    qPelv(fr,1),qPelv(fr,2),qPelv(fr,3),qPelv(fr,4))];
            text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                    qHip(fr,1),qHip(fr,2),qHip(fr,3),qHip(fr,4))];
            text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                    qKnee(fr,1),qKnee(fr,2),qKnee(fr,3),qKnee(fr,4))];
            text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f',...
                    qAnk(fr,1),qAnk(fr,2),qAnk(fr,3),qAnk(fr,4))];
                
            if twoLegs
                
                text = [text, sprintf(',%2.6f,%2.6f,%2.6f,%2.6f,',...
                        -qHip(fr,1),-qHip(fr,2),qHip(fr,3),qHip(fr,4))];
                text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f,',...
                        -qKnee(fr,1),-qKnee(fr,2),qKnee(fr,3),qKnee(fr,4))];
                text = [text, sprintf('%2.6f,%2.6f,%2.6f,%2.6f',...
                        -qAnk(fr,1),-qAnk(fr,2),qAnk(fr,3),qAnk(fr,4))];
                
            end
            
            text = [text, sprintf('\n')];
        
        end
        
    case 'oneHinge'
        
        outname = 'oneHinge.txt';

        a = pi/6.*sin(0:0.01*pi:2*pi);
        text = '';
        for ang = a
            text = [text sprintf('%2.6f,0.0\n',ang)];
        end

end

%%Finally, output the result to file

FID = fopen(outname,'w');

if FID==-1
    error('Could not open output file');
end

fprintf(FID,text);

fclose(FID);

end
