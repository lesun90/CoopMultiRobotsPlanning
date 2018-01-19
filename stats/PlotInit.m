function [] = PlotInit(pw, ph)
clf;
cla;

hold on;
grid on;
box  on;

set(gcf, 'paperunits', 'inches');
set(gcf, 'papersize', [pw, ph]);
set(gcf, 'paperposition', [0.0, 0.0, pw, ph]);

set(gca, 'fontname', 'Helvetica', 'fontsize', 6);
end

