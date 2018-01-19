function [] = PlotLengthSearch()

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
c1 = [138	43	226]/ 255;
c2 = [171	130	255] / 255;
c3 = [71	60	139] / 255;
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
for m = rstart : 1 : rend
dataCoop5(m, I_COST) = dataCoop5(m, I_COST) / s;
dataCoop5(m, I_COST+1) = dataCoop5(m, I_COST+1) / s;
dataCoop2(m, I_COST) = dataCoop2(m, I_COST) / s;
dataCoop2(m, I_COST+1) = dataCoop2(m, I_COST+1) / s;
dataCoop10(m, I_COST) = dataCoop10(m, I_COST) / s;
dataCoop10(m, I_COST+1) = dataCoop10(m, I_COST+1) / s;
end
rstart = rend + 1;
end

w = 0.28;
which = 3; ylimit=2.7;
for k = 1 : 1 : nrRows
  MyBar(k - w, ymin, dataCoop2(k, I_COST), dataCoop2(k, I_COST+1), w, c2, gca, which, ylimit); 
  MyBar(k    , ymin, dataCoop5(k, I_COST), dataCoop5(k, I_COST+1), w, c1, gca, which, ylimit); 
  MyBar(k + w, ymin, dataCoop10(k, I_COST), dataCoop10(k, I_COST+1), w, c3, gca, which, ylimit); 
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
scenes_xpos = [1.5, 9.5, 14.5, 19, 20, 21];
for k = 1 : 1 : length(scenes)
text(scenes_xpos(k), 2.3, scenes{k}, ...
      'HorizontalAlignment', 'center', 'fontsize', fontsize, 'fontname', 'Helvetica');
end

%legend
h = fill([7.2 8.1 8.1 7.2],[1.25 1.25 2.5 2.5], [1 1 1]);
set(h, 'facecolor', [1 1 1]);
MyPlot(7.5, 2.3, [], 's', c2);
text(7.8, 2.35, 'a', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(7.5, 1.9, [], 's', c1);
text(7.8, 1.95, 'b', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');
MyPlot(7.5, 1.5, [], 's', c3);
text(7.8, 1.55, 'c', 'HorizontalAlignment', 'left', 'fontsize', fontsize, 'fontname', 'Helvetica');


saveas(gcf, 'kot.pdf', 'pdf');
system(['pdfcrop kot.pdf ../figs/figResLengthSearch', '.pdf']);


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
plot(x, y, marker, 'color', rgb, 'markersize', markersize, 'markerfacecolor', rgb, 'markeredgecolor', rgb, 'linewidth', 1.0);
for k = 1 : 1 : length(ystd)
line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
end
end

end


