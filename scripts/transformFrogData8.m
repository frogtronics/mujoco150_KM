function [data_t, TM] = transformFrogData8(data)

%horizontal xz is on best fit plane through ilia and shoulders
%vertical y is normal to that plane
%z axis is sacrum to neck
%sacrum is origin

%sacrum, neck and iliac/shoulder points
% originPoint = 12;
% zPoint = 11;
% planarPoints = [6 7 8 9];
originPoint = 7; %sacrum, for 2016 data
zPoint = 6; %snout

%extract xyz data for each point (all frames)
O = data(:,originPoint*3-2:originPoint*3);
zPoint = data(:,zPoint*3-2:zPoint*3);

data_t = data;

TM = zeros(4,4,length(O));

%go through each frame
for fr = 1:size(O,1)
    
    %create Z axis from spine points
    zAx = [normalize(O(fr,:) - zPoint(fr,:))];
    
    %reconstruct orthogonal vectors 
    xAx = normalize(cross([0 0 1],zAx));
    yAx = normalize(cross(xAx,zAx));

    %create transformation matrix
    R = [xAx',yAx',zAx'];
    T = -O(fr,:)*R;

    TM(:,:,fr) = [[R zeros(3,1)];[T 1]];
    
    %apply TM to every point
    for j = 3:3:24 
         out = [data(fr,j-2:j) 1]*TM(:,:,fr);
         data_t(fr,j-2:j) = out(1:3);
    end
end

end



