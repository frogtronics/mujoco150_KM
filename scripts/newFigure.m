function f = newFigure(fname)
%%NEWFIGURE - avoid overwriting or duplicating figures
%
%   f = NEWFIGURE(fname) checks if a valid figure handle called fname
%   exists in the base workspace. If so, it returns that handle and makes
%   that figure active. If not, it creates a new figure and named handle.
%
%   
%   Usage:
%   Enter desired handle name as string, and return it to a variable
%   by that name, e.g. varName = NEWFIGURE('varName')
%
%   Example: 
%   
%   %The first time this script is run, it opens a new figure. While that
%   figure remains open (and the handle is not deleted), running the script
%   again overwrites the existing figure (i.e. no scripts overwriting each
%   other with 'figure(1)', and no endless new figures with just 'figure').
%   
%   x = 0:0.1:pi; y = sin(x);
%   sinFig = newFigure('sinFig');
%   plot(x,y);
%
%

try
    f=evalin('base',fname);

    if ~isvalid(f)
        f = figure;
    end
catch
    f = figure;
end

clf(f); figure(f);

end