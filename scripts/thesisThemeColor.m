function [c, col] = thesisThemeColor(ind)

C = [0 0.5 0.2; %dark green
     0 0.2 0.5; %dark blue
     0.7 0.2 0.2; %dark red
     0.4 0.2 0; %dark brown
     0.8 0.6 0; %dark yellow
     0.5 0 0.7]; %dark purple
             
Col =  {'dark green',...
        'dark blue',...
        'dark red',...
        'dark brown',...
        'dark yellow',...
        'dark purple'};
 
if nargin < 1
    c = C;
    col = Col;
else
    c = C(mod(ind-1,size(C,1))+1,:);
    col = Col{mod(ind-1,size(C,1))+1};
end



end