function exportThesisFigure(fig, chapter, filename, format)


if nargin < 3
    error('Please supply a figure handle, a chapter number and a filename')
elseif nargin < 4
    format = 'pdf';
end


if ~isnumeric(chapter)
    error('Please a chapter number as second argument')
end


switch format
    case 'pdf'
    	opt = {'-dpdf', '-r0'};
    case 'png'
    	opt = {'-dpng', '-r500'};
    otherwise
        error('Format must either be "pdf" or "png"')
end



str = sprintf('Chapter%i', round(chapter));

print(fig, ...
    ['~/Documents/RVC/Writing/Thesis/Tex/' str '/images/' filename],...
    opt{1}, opt{2})


end