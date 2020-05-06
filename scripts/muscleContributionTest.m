

contribFig = newFigure('contribFig');


l = plot([0 1],[0 1], 'k--');
hold on;
sc = scatter([0.25 0.5 0.75],[0.75 0.5 0.25], 'o');
hold off;

t(1) = text(0.27, 0.73,'A');
t(2) = text(0.52, 0.48,'B');
t(3) = text(0.77, 0.23,'C');

ax = gca; ax.YTick = []; ax.XTick = [];

ax.YLabel.String = 'FFS Volume Contribution (N^3)';
ax.XLabel.String = 'Muscle f_{max} (N)';

sc.CData = thesisThemeColor(1);
for txt = t
    txt.FontWeight = 'bold';
end


formatFigure(contribFig,'a7','Thesis',1.5)