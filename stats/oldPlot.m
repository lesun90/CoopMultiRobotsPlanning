function [] = PlotTime()

I_NR_ROBOTS  = 1;
I_TOTAL      = 2;
I_PRE        = 4;
I_COST       = 6;
I_ASTAR      = 8;
I_SIMULATE   = 10;
I_COLLISION  = 12;

dataCoop2 = LoadData('tables/Table_CoopDepth2.txt');
dataCoop5 = LoadData('tables/Table_CoopDepth5.txt');
dataCoop10= LoadData('tables/Table_CoopDepth10.txt');
dataCRRT  = LoadData('tables/Table_cRRT.txt');
dataPRRT  = LoadData('tables/Table_pRRT.txt');
dataPEP   = LoadData('tables/Table_pEP.txt');

[nrRows, nrCols] = size(dataCoop5);


last   = [6, 12, 18, 19, 20, 21];
labels = {'1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '3', '3', '3'};

data = {dataCoop10};


fontsize   = 9;
linewidth  = 1.0;
markersize = 8;
c1 = [1 0 1];
c2 = [0.5 0.5 0.5];
c3 = [0.6 0.6 1];
c4 = [0.4 0.8 0.4];
markers={'-o','-s','-d','-v'};

PlotInit(6.9, 1.4);
xstart = 0.054;
xdim   = (0.995 - xstart); 
set(gca, 'position', [xstart, 0.18, xdim, 0.81]);

set(gca, "xgrid", "off");
set(gca, "ygrid", "on");
  
ymin = 1/8;
ymax = 64;
yvals= [1/8,1/4,1/2, 1, 2, 4, 8, 16, 32, 64];
ylabs= {'1/8', '1/4', '1/2', '1', '2', '4', '8', '16', '32', '64'};

hold on;
set(gca, 'yscale', 'log');

set(gca, 'xlim', [0.5, length(labels) + 0.5]);
set(gca, 'xtick', 1:1:length(labels));
set(gca, 'xticklabel', labels);

set(gca, 'ylim', [ymin, 70]);
set(gca, 'ytick', yvals);
set(gca, 'yticklabel', {});

w = 0.22;
which = 3; ylimit=64;

%for k = 1 : 1 : nrRows
%  w = 0.22;
%  %MyBar(k - 1.5 * w, ymin, Value(dataCoop5(k, I_TOTAL), dataCoop5(k, I_PRE)), dataCoop5(k, I_TOTAL+1), w, c1, gca, which, ylimit); 
%  MyBar(k - 0.5 * w, ymin, Value(dataPEP(k, I_TOTAL), dataPEP(k, I_PRE)),     dataPEP(k, I_TOTAL+1), w, c2, gca, which, ylimit); 
%  MyBar(k + 0.5 * w, ymin, Value(dataPRRT(k, I_TOTAL), dataPRRT(k, I_PRE)), dataPRRT(k, I_TOTAL+1),w, c3, gca, which, ylimit); 
%  MyBar(k + 1.5 * w, ymin, Value(dataCRRT(k, I_TOTAL), dataCRRT(k, I_PRE)), dataCRRT(k, I_TOTAL+1),w, c4, gca, which, ylimit); 
%end

rstart = 1;
for k = 1 : 1 : length(last)
rend = last(k);
MyPlot(rstart:1:rend, dataCRRT(rstart:rend, I_TOTAL), dataCRRT(rstart:rend, I_TOTAL+1), markers{4}, c4);
MyPlot(rstart:1:rend, dataPRRT(rstart:rend, I_TOTAL), dataPRRT(rstart:rend, I_TOTAL+1), markers{3}, c3);
MyPlot(rstart:1:rend, dataPEP(rstart:rend, I_TOTAL), dataPEP(rstart:rend, I_TOTAL+1), markers{2}, c2);
MyPlot(rstart:1:rend, dataCoop5(rstart:rend, I_TOTAL), dataCoop5(rstart:rend, I_TOTAL+1), markers{1}, c1);
rstart = rend + 1;
end

for k = 1 : 1 : length(last)
line([last(k) + 0.5, last(k) + 0.5], get(gca, 'ylim'), 'linewidth', linewidth);
end

%for k = 1 : 1 : length(labels)
%    text(k, 0.7*ymin, labels{k}, 
%	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center', ...
%        'fontsize', fontsize, 'fontname', 'Helvetica');
%end
%text(0.5, -0.15, 'nr. of robots', 'units', 'normalized', 
%      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');

for k = 1 : 1 : length(yvals)
    text(0.32, yvals(k), ylabs{k}, 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', ...
        'fontsize', fontsize, 'fontname', 'Helvetica');
end
 text(-0.05, 0.5, 'runtime [s]', 'units', 'normalized', 
      'HorizontalAlignment', 'center', 'rotation', 90, ...
      'fontsize', fontsize, 'fontname', 'Helvetica');

scenes = {'scene 1', 'scene 2', 'scene 3', 's4', 's5', 's6'};
scenes_xpos = [3.5, 9.5, 15.5, 19, 20, 21];
for k = 1 : 1 : length(scenes)
text(scenes_xpos(k), 1/4, scenes{k}, ...
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');
end

%legend
h = fill([17.2 18.1 18.1 17.2],[2^(-2.7), 2^(-2.7), 2^(2.1), 2^(2.1)], [1 1 1]);
set(h, 'facecolor', [1 1 1]);
MyPlot(17.5, 2^1.5, [], markers{1}, c1);
text(17.8, 2^1.7, 'a', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.5, 2^(0.3), [], markers{2}, c2);
text(17.8, 2^0.5, 'b', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.5, 2^(-0.9), [], markers{3}, c3);
text(17.8, 2^(-0.7), 'c', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.5, 2^(-2.1), [], markers{4}, c4);
text(17.8, 2^(-1.9), 'd', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');

function [] = MyPlot(x, y, ystd, marker, rgb)
plot(x, y, marker, 'color', rgb, 'markersize', markersize, 'markerfacecolor', rgb, 'markeredgecolor', rgb, 'linewidth', 1.0);
for k = 1 : 1 : length(ystd)
line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
end
end

end

