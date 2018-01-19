
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

scene_scale = [sqrt(84*84+84*84), sqrt(120*120+140*140), sqrt(100*100+100*100), sqrt(30*30+35*35), sqrt(60*60+34*34),sqrt(90*90+30*30)];

[nrRows, nrCols] = size(dataCoop5);


last   = [6, 12, 18, 19, 20, 21];
labels = {'1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '1', '2', '4', '6', '8', '10', '3', '3', '3'};

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
  
ymin = 0;
ymax = 64;
yvals= 0:0.5:2.5;
ylabs= {'0', '1/2', '1', '1.5', '2', '2.5'};

hold on;

set(gca, 'xlim', [0.5, length(labels) + 0.5]);
set(gca, 'xtick', 1:1:length(labels));
set(gca, 'xticklabel', {});

set(gca, 'ylim', [ymin, 2.9]);
set(gca, 'ytick', yvals);
set(gca, 'yticklabel', {});

rstart = 1;
for k = 1 : 1 : length(last)
s = scene_scale(k);
rend = last(k);
pos = FindEnd(dataCRRT(rstart:rend, I_TOTAL));
MyPlot(rstart:1:rstart+pos, dataCRRT(rstart:rstart+pos, I_COST)/s, dataCRRT(rstart:rstart+pos, I_COST+1)/s, markers{4}, c4);
pos = FindEnd(dataPRRT(rstart:rend, I_TOTAL));
MyPlot(rstart:1:rstart+pos, dataPRRT(rstart:rstart+pos, I_COST)/s, dataPRRT(rstart:rstart+pos, I_COST+1)/s, markers{3}, c3);
pos = FindEnd(dataPEP(rstart:rend, I_TOTAL));
MyPlot(rstart:1:rstart+pos, dataPEP(rstart:rstart+pos, I_COST)/s, dataPEP(rstart:rstart+pos, I_COST+1)/s, markers{2}, c2);
MyPlot(rstart:1:rend, dataCoop5(rstart:rend, I_COST)/s, dataCoop5(rstart:rend, I_COST+1)/s, markers{1}, c1);
rstart = rend + 1;
end

for k = 1 : 1 : length(last)
line([last(k) + 0.5, last(k) + 0.5], get(gca, 'ylim'), 'linewidth', linewidth);
end

for k = 1 : 1 : length(labels)
    text(k, -0.14, labels{k}, 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center', ...
        'fontsize', fontsize, 'fontname', 'Helvetica');
end
text(0.5, -0.15, 'nr. of robots', 'units', 'normalized', 
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');

for k = 1 : 1 : length(yvals)
    text(0.45, yvals(k), ylabs{k}, 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', ...
        'fontsize', fontsize, 'fontname', 'Helvetica');
end
 text(-0.05, 0.42, 'solution length [diag(W)]', 'units', 'normalized', 
      'HorizontalAlignment', 'center', 'rotation', 90, ...
      'fontsize', fontsize, 'fontname', 'Helvetica');

scenes = {'scene 1', 'scene 2', 'scene 3', 's4', 's5', 's6'};
scenes_xpos = [3.5, 9.5, 15.5, 19, 20, 21];
for k = 1 : 1 : length(scenes)
text(scenes_xpos(k), 2.3, scenes{k}, ...
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');
end

%legend
h = fill([17.1 18.0 18.0 17.1],[0.9 0.9 2.5 2.5], [1 1 1]);
set(h, 'facecolor', [1 1 1]);
MyPlot(17.4, 2.3, [], markers{1}, c1);
text(17.7, 2.35, 'a', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.4, 1.9, [], markers{2}, c2);
text(17.7, 1.95, 'b', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.4, 1.5, [], markers{3}, c3);
text(17.7, 1.55, 'c', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(17.4, 1.1, [], markers{4}, c4);
text(17.7, 1.15, 'd', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');

function [pos] = FindEnd(rtime)
pos = -1;
for k = 1 : 1 : length(rtime)
  if rtime(k) < 68;
     pos = k-1; 
   else
     return;
   end
end
end

function [] = MyPlot(x, y, ystd, marker, rgb)
plot(x, y, marker, 'color', rgb, 'markersize', markersize, 'markerfacecolor', 'none', 'markeredgecolor', rgb, 'linewidth', 1.0);
for k = 1 : 1 : length(ystd)
line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
end
end



