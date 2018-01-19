clc;
clear all;

hold on;

I_NR_ROBOTS       = 1;
I_TOTAL_TIME      = 2;
I_TOTAL_TIME_STD  = 3;
I_PRE             = 4;
I_COST            = 6;
I_ASTAR           = 8;
I_SIMULATE        = 10;
I_COLLISION       = 12;

CRRT    = 1;
PRRT    = 2;
PGUST   = 3;
COOP0   = 4;
COOP2   = 6;
COOP5   = 5;
COOP10  = 7;

nrPlanner = 7;
SCAR = 1;
BCAR = 2;
robotType = SCAR;
%xlabels = {'1', '2', '4', '6', '8', '10'};
xlabels = {};
plannerlabels = {'cRRT','pRRT','pGUST','Coop0','Coop2','Coop5','Coop10' };
markersColor = [1 0 1;0.5 0.5 0.5;0.6 0.6 1;0.4 0.8 0.4;0.1 0.4 0.6;0.1 0.4 0.6;0.1 0.4 0.6;0.3 0.1 0.2;0.5 0.2 0.2];
markers={'o','s','d','v','x','o','s'};
markerSize = 8;
fontsize   = 15;
linewidth  = 3.0;

ymin = 1/8;
ymax = 64;
yvals= [1/8,1/4,1/2, 1, 2, 4, 8, 16, 32, 64];
ylabs= {'1/8', '1/4', '1/2', '1', '2', '4', '8', '16', '32', '64'};

set(gca, "xgrid", "off");
set(gca, "ygrid", "on");
%set(gca, 'yscale', 'log');
%set(gca, 'ylim', [ymin, 70]);
%set(gca, 'ytick', yvals);
%set(gca, 'yticklabel', ylabs);

sceneName   = {'sceneA', 'sceneB', 'sceneC','case2', 'case3'};
sceneNamePath = {'/res_cRRT_0.txt', '/res_pRRT_0.txt', '/res_Priority_0.txt', '/res_Coop_0_0.txt', '/res_Coop_0_2.txt', '/res_Coop_0_5.txt', '/res_Coop_0_10.txt'
                 '/res_cRRT_1.txt', '/res_pRRT_1.txt', '/res_Priority_1.txt', '/res_Coop_1_0.txt', '/res_Coop_1_2.txt', '/res_Coop_1_5.txt', '/res_Coop_1_10.txt';};
                 
rstart = 1;
for i = 4:size(sceneName,2)
   StatData = {};
   StatData(1) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,4})))};
   StatData(2) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,5})))};
   StatData(3) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,6})))};
   StatData(4) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,7})))};
   rend = rstart+1;
    
   for k = 1:size(StatData{1},1)
      xlabels{1,size(xlabels,2)+1} = num2str(StatData{1}(k,I_NR_ROBOTS));
   end
    rstart
   for p = 1:4
     plot ([rstart],StatData{p}(1:size(StatData{1},1), I_TOTAL_TIME),...
     'marker',markers{p},...
     'color',markersColor(p,:),...
     'markeredgecolor', markersColor(p,:),...
     'linestyle','-',...
     'markersize', 10,...
     'linewidth',linewidth);
   end
   
%    for p = 1:5
%      y = StatData{p}(1:size(StatData{1},1), I_TOTAL_TIME);
%      x = rstart:1:rend;
%      ystd = StatData{p}(1:size(StatData{1},1), I_TOTAL_TIME_STD);
%      for k = 1 : 1 : length(ystd)
%        line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', linewidth);
%      end
%    end
   line([rend + 0.5, rend + 0.5], get(gca, 'ylim'), 'linewidth', linewidth);
   text(rstart, 1/4, sceneName(1,i),...
        'HorizontalAlignment', 'center',... 
        'fontsize', fontsize,...
        'fontname','Helvetica');
        
   rstart = rend;

  
end

legend('CoopPAS','CoopAS2','CoopAS5','CoopAS10')
legend('Location','bestoutside')

set(gca, 'xlim', [0.5, size(xlabels,2)+ 0.5]);
set(gca, 'xtick', 1:1:length(xlabels));
set(gca, 'xticklabel', xlabels);

ylabel('runtime [s]')
xlabel('nr.of robots')





