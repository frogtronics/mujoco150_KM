text='';

printXY = @(xy)(sprintf('0, 0, 0, 0.7071, 0.7071, 0, 0, 0.17, 0, %1.4f, %1.4f\n',xy(1),xy(2)));


t = 0:0.1:2*pi;

x = 0.34 + 0.17*sin(t);
y = 0.1*sin(t);

for a = y
    xy(1) = 0.3;
    xy(2) = a;
    text = [text printXY(xy)]; %#ok<AGROW>
end

FID = fopen('../input/sinY.txt','w');
if FID==-1
    error('Could not open output file');
end

fprintf(FID,text);

fclose(FID);