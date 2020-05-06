function fig = formatFigure(fig,fsize,style,linewidth,linecol)
%FORMATFIGURE
%
%   fig = FORMATFIGURE(fig,size,style) is used to format a figure
%       to a consistent and predetermined style.
%
%   'fig' is the handle to an existing figure.
%
%   'size' determines the dimensions of the figure:
%     - 'small' | 'medium' (default) | 'large'
%     - 'a3' - 'a7' (landscape)
%     - 'a3p' - 'a7p' (portrait)
%     - 'a3sq' - 'a7sq' (square that fits within page)
%     - If size is a 2 element array (e.g. [10 5]) it is treated as 
%       x y dimensions in centimetres.
%
%

%parse params

if nargin < 2
    fsize = 'medium';
end
if nargin < 4 || isempty(linewidth)
    linewidth = 0;
end
if nargin < 5
    linecol = 0;
end

pagemargin = 0.9;

if(ischar(fsize))
    switch(fsize)
        case 'small'
            xsiz = 10; ysiz = 8;
        case 'medium'
            xsiz = 20; ysiz = 16;
        case 'large'
            xsiz = 30; ysiz = 24;
        case 'a3'
            xsiz = 42; ysiz = 29.7;
        case 'a3p'
            ysiz = 42; xsiz = 29.7;
        case 'a3sq'
            ysiz = 29.7; xsiz = 29.7;
        case 'a4'
            xsiz = 29.7; ysiz = 21;
        case 'a4p'
            ysiz = 29.7; xsiz = 21;
        case 'a4sq'
            ysiz = 21; xsiz = 21;
        case 'a5'
            xsiz = 21; ysiz = 14.8;
        case 'a5p'
            ysiz = 21; xsiz = 14.8;
        case 'a5sq'
            ysiz = 14.8; xsiz = 14.8;
        case 'a6'
            xsiz = 14.8; ysiz = 10.5;
        case 'a6p'
            ysiz = 14.8; xsiz = 10.5;
        case 'a6sq'
            ysiz = 10.5; xsiz = 10.5;    
        case 'a7'
            xsiz = 10.5; ysiz = 7.4;
        case 'a7p'
            ysiz = 10.5; xsiz = 7.4;
        case 'a7sq'
            ysiz = 7.4; xsiz = 7.4;
        case 'us1col'
            xsiz = 8.6; ysiz = 6.1;
        case 'us2col'
            xsiz = 17.8; ysiz = 6.1;
        case 'us2coltall'
            xsiz = 17.8; ysiz = 12.6;
        case 'a41col'
            xsiz = 8.6; ysiz = 8.6;
        case 'a42col'
            xsiz = 17.8; ysiz = 6.1;
        case 'a42coltall'
            xsiz = 17.8; ysiz = 12.6;
        case 'a4pinset' %a4 portrait, but with margins
            ysiz = 29.7*0.8333; xsiz = 21*0.8333;
            
        otherwise
            
            
    end
elseif(numel(fsize) == 2)
    xsiz = fsize(1)/pagemargin;
    ysiz = fsize(2)/pagemargin;
end


if nargin < 3
    style = 'EE';
end



%hide figure elements
fig.MenuBar = 'none';
fig.ToolBar = 'none';
fig.Resize  = 'off';

%set figure sizes
fig.Units = 'centimeters';
fig.Position = [1 1 xsiz*pagemargin ysiz*pagemargin];

%set paper size
set(fig,'PaperUnits','centimeters','PaperSize',[xsiz ysiz])

%movegui(fig, 'center')
fig.Units = 'pixels';



%get all axes in figure
axes = gobjects(0);
for childN = 1:numel(fig.Children)
    child = fig.Children(childN);
    
    if(strcmp(child.Type, 'axes'))
        axes(end+1) = child; %#ok<AGROW>
    end
end

switch(style)
    
    case 'EE'
        for ax = axes
            
            set(axes, 'FontName', 'Helvetica');
            set(axes, 'FontSize', 12);
            set(axes, 'LineWidth', 1);
            set(axes, 'LabelFontSizeMultiplier', 1.5);
            set(axes, 'FontWeight', 'normal');
            set(axes, 'TitleFontSizeMultiplier', 2);
            set(axes, 'TitleFontWeight', 'bold');
            
            if(isempty(ax.Title.String))
                ax.Title.String = 'Placeholder Title';
            end
            
            %format axis labels
            labels = strcat({'X','Y','Z'},'Label');
            for label = labels
                if(isempty(ax.(label{1}).String))
                    ax.(label{1}).String = label{1}(1);
                end
                ax.(label{1}).Units = 'normalized';
            end
            
            %make all plotted lines a bit thicker
            if linewidth
                set(ax.Children, 'LineWidth', linewidth);
            end
        end
        
    case 'IEEE'
        
        set(axes, 'FontName', 'TimesNewRoman');
        set(axes, 'FontSize', 8);
        set(axes, 'LineWidth', 1);
        set(axes, 'LabelFontSizeMultiplier', 1);
        set(axes, 'FontWeight', 'normal');
        set(axes, 'TitleFontSizeMultiplier', 1.2);
        set(axes, 'TitleFontWeight', 'bold');

        for ax = axes
            %format axis labels
            labels = strcat({'X','Y','Z'},'Label');
            for label = labels
                if(isempty(ax.(label{1}).String))
                    ax.(label{1}).String = label{1}(1);
                end
                ax.(label{1}).Units = 'normalized';
            end
            
            %make all plotted lines a bit thicker
            if linewidth
                set(ax.Children, 'LineWidth', linewidth);
            end
        end
    
     case 'RSOS'
        
        set(axes, 'FontName', 'Arial');
        set(axes, 'FontSize', 16);
        set(axes, 'LineWidth', 1);
        set(axes, 'LabelFontSizeMultiplier', 1);
        set(axes, 'FontWeight', 'normal');
        set(axes, 'TitleFontSizeMultiplier', 1.2);
        set(axes, 'TitleFontWeight', 'bold');

        for ax = axes
            %format axis labels
            labels = strcat({'X','Y','Z'},'Label');
            for label = labels
                if(isempty(ax.(label{1}).String))
                    ax.(label{1}).String = label{1}(1);
                end
                ax.(label{1}).Units = 'normalized';
            end
            
            %make all plotted lines a bit thicker
            if linewidth
                set(ax.Children, 'LineWidth', linewidth);
            end
        end
        
    case 'Thesis'
        
        set(axes, 'FontName', 'Helvetica');
        set(axes, 'FontSize', 8);
        set(axes, 'LineWidth', 1);
        set(axes, 'LabelFontSizeMultiplier', 1);
        set(axes, 'FontWeight', 'normal');
        set(axes, 'TitleFontSizeMultiplier', 1.2);
        set(axes, 'TitleFontWeight', 'bold');

        for ax = axes
            %format axis labels
            labels = strcat({'X','Y','Z'},'Label');
            for label = labels
                if(isempty(ax.(label{1}).String))
                    %ax.(label{1}).String = label{1}(1);
                end
                ax.(label{1}).Units = 'normalized';
            end
            
            %make all plotted lines a bit thicker if specified
            if linewidth
                set(ax.Children, 'LineWidth', linewidth);
            end
            
            %and set linecolors to theme colors 
        
            if linecol
                children = ax.Children';
                children = children(end:-1:1);
                chnum = 1;
                for child = children
                    if strcmp(child.Type, 'line')
                       child.Color = thesisThemeColor(chnum); 
                       chnum = chnum + 1;
                    end
                end
            end
        end
        
    case 'pure' %no axes, just white background
        for ax = axes
            
            ax.Title.String = '';
            
            %remove axis lines and labels
            labels = {'X','Y','Z'};
            for label = labels
                ax.([label{1} 'Tick']) = [];
                ax.([label{1} 'Grid']) = 'off';
                ax.([label{1} 'Color']) = [1 1 1];
                ax.([label{1} 'Label']).String = '';
            end
            
            %make all plotted lines a bit thicker
            if linewidth
                set(ax.Children, 'LineWidth', linewidth);
            end
        end
        
    otherwise

    
end
