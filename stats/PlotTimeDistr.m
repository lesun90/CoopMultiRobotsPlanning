function [] = PlotTimeDistr()

I_NR_ROBOTS  = 1;
I_TOTAL      = 2;
I_PRE        = 4;
I_COST       = 6;
I_ASTAR      = 8;
I_SIMULATE   = 10;
I_COLLISION  = 12;

dataCoop5 = LoadData('tables/Table_CoopDepth5.txt');
[nrRows, nrCols] = size(dataCoop5);


last   = [6, 12, 18, 19, 20, 21];
labels = {'1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '3', '3', '3'};

fontsize   = 9;
linewidth  = 1.0;
markersize = 8;
c1 = [1 1 0.8];
c2 = [0.2 0.2 0.6];
c3 = [1 0.6 1];
c4 = [0.4 0.8 0.4];
c5 = [0.5 0.5 0.5];
markers={'-o','-s','-d','-x'};

PlotInit(3.5, 1.4);
xstart = 0.1;
xdim   = (0.995 - xstart); 
set(gca, 'position', [xstart, 0.18, xdim, 0.81]);

set(gca, "xgrid", "off");
set(gca, "ygrid", "on");

ymin = 0;
ymax = 100;
yvals = 0 : 20 : 100;

hold on;

set(gca, 'xlim', [0.5, length(labels) + 0.5]);
set(gca, 'xtick', []);
set(gca, 'xticklabel', {});

set(gca, 'ylim', [ymin, 112]);
set(gca, 'ytick', yvals);
set(gca, 'yticklabel', {});

w = 0.7;
which = 1;
ylimit = Inf;

for k = 1 : 1 : nrRows
  total = dataCoop5(k, I_TOTAL);
  pre   = 100 * dataCoop5(k, I_PRE) / total;
  astar = 100 * dataCoop5(k, I_ASTAR) / total;
  sim   = 100 * dataCoop5(k, I_SIMULATE) / total;
  coll  = 100 * dataCoop5(k, I_COLLISION) / total;

  MyBar(k, 0, pre, 0.0, w, c1, gca, which, ylimit); 
  MyBar(k, pre, pre + astar, 0.0, w, c2, gca, which, ylimit); 
  MyBar(k, pre+astar, pre + astar + sim, 0.0, w, c3, gca, which, ylimit); 
  MyBar(k, pre+astar+sim, pre + astar + sim+coll, 0.0, w, c4, gca, which, ylimit); 
  MyBar(k, pre+astar+sim+coll, 100, 0.0, w, c5, gca, which, ylimit); 
end

for k = 1 : 1 : length(last)
line([last(k) + 0.5, last(k) + 0.5], get(gca, 'ylim'), 'linewidth', linewidth);
end

for k = 1 : 1 : length(labels)
    text(k, -4, labels{k}, 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center', ...
        'fontsize', fontsize, 'fontname', 'Helvetica');
end
text(0.5, -0.15, 'nr. of robots', 'units', 'normalized', 
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');

for k = 1 : 1 : length(yvals)
    text(0.32, yvals(k), num2str(yvals(k)), 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', ...
        'fontsize', fontsize, 'fontname', 'Helvetica');
end
 text(-0.1, 0.42, 'runtime distribution [%]', 'units', 'normalized', 
      'HorizontalAlignment', 'center', 'rotation', 90, ...
      'fontsize', fontsize, 'fontname', 'Helvetica');

scenes = {'scene 1', 'scene 2', 'scene 3', 's4', 's5', 's6'};
scenes_xpos = [3.5, 9.5, 15.5, 19, 20, 21];
for k = 1 : 1 : length(scenes)
text(scenes_xpos(k), 108, scenes{k}, ...
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');
end



saveas(gcf, 'kot.pdf', 'pdf');
system(['pdfcrop kot.pdf ../figs/figResTimeDistr', '.pdf']);


function [] = MyPlot(x, y, ystd, marker, rgb)
plot(x, y, marker, 'color', rgb, 'markersize', markersize, 'markerfacecolor', rgb, 'markeredgecolor', rgb, 'linewidth', 1.0);
for k = 1 : 1 : length(ystd)
line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
end
end

end


