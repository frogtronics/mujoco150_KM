function [data_t, TM] = transformFrogData12(data)

%horizontal xz is on best fit plane through ilia and shoulders
%vertical y is normal to that plane
%z axis is sacrum to neck
%sacrum is origin

%sacrum, neck and iliac/shoulder points
originPoint = 12;
zPoint = 11;
planarPoints = [6 7 8 9];

%extract xyz data for each point (all frames)
O = data(:,originPoint*3-2:originPoint*3);
zPoint = data(:,zPoint*3-2:zPoint*3);

data_t = data;

TM = zeros(4,4,length(O));

%go through each frame
for fr = 1:size(O,1)
    
    %create Z axis from spine points
    zAx = normalize(O(fr,:) - zPoint(fr,:));
    
    %make a 4x3 matrix of the planar points x, y, z
    frPoints = [data(fr,planarPoints*3-2)',data(fr,planarPoints*3-1)',...
                data(fr,planarPoints*3)'];
            
    %find average x, y and z of points        
    meanP = mean(frPoints,1);
    
    %move points so mean is at (0,0,0)
    
    %frPoints(:,1) = frPoints(:,1) - mean(1)
    %frPoints(:,2) = frPoints(:,2) - mean(2)
    %frPoints(:,3) = frPoints(:,3) - mean(3)
    %but more elegantly,
    R = bsxfun(@minus,frPoints,meanP);
    
    %linear algebra bit: make the point matrix
    %symmetric 3x3 by multiplying by the transpose
    %then compute principal directions of eigenvectors
    [V,~] = eig(R'*R);
    %extract the vector normal to the plane
    yAx = V(:,1)';
    
    
    %check that normal is facing the right way
    %construct approximate x and y vectors from left/right iliac points
    xAxApprox = normalize(data(fr,7*3-2:7*3) - data(fr,6*3-2:6*3));
    yAxApprox = normalize(cross(zAx,xAxApprox));
    
    if(dot(yAxApprox,yAx) < 0)
        yAx = -yAx;
    end
    
    %reconstruct orthogonal vectors using rectified plane normal
    xAx = normalize(cross(yAx,zAx));
    yAx = normalize(cross(zAx,xAx));

    %create transformation matrix
    R = [xAx',yAx',zAx'];
    T = -O(fr,:)*R;

    TM(:,:,fr) = [[R zeros(3,1)];[T 1]];
    
    %apply TM to every point
    for j = 3:3:36
         out = [data(fr,j-2:j) 1]*TM(:,:,fr);
         data_t(fr,j-2:j) = out(1:3);
    end
end

end



