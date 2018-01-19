function [] = PlotLength()
hold on;

I_NR_ROBOTS       = 1;
I_TOTAL_TIME      = 2;
I_PRE             = 4;
I_PCOST            = 6;
I_ASTAR           = 8;
I_SIMULATE        = 10;
I_COLLISION       = 12;

CRRT    = 1;
PRRT    = 2;
PGUST   = 3;
COOP0   = 4;
COOP2   = 5;
COOP5   = 6;
COOP10  = 7;

nrPlanner = 7;
SCAR = 1;
BCAR = 2;
robotType = BCAR;
%xlabels = {'1', '2', '4', '6', '8', '10'};
xlabels = {};
plannerlabels = {'cRRT','pRRT','pGUST','CoopPaS','CoopCA2','CoopCA5','CoopCA10' };
markersColor = [1 0 1;0.5 0.5 0.5;0.6 0.6 1;0.4 0.8 0.4;0.1 0.4 0.6;0.1 0.4 0.6;0.1 0.4 0.6;0.3 0.1 0.2;0.5 0.2 0.2];
markers={'o','s','d','v','^','h','x'};
markerSize = 10;
fontsize   = 15;
linewidth  = 2.0;

scene_scale = [sqrt(128*128+128*128), sqrt(90*90+90*90), sqrt(80*80+80*80), sqrt(60*60+30*30), sqrt(60*60+30*30),sqrt(60*60+30*30)];


ymin = 1/8;
ymax = 64;
yvals= [1/8,1/4,1/2, 1, 2, 4, 8, 16, 32, 64];
ylabs= {'0', '1/2', '1', '1.5', '2', '2.5'};

set(gca, "xgrid", "off");
set(gca, "ygrid", "on");
set(gca, 'yscale', 'log');
set(gca, 'ylim', [ymin, 2.9]);
set(gca, 'ytick', yvals);
set(gca, 'yticklabel', ylabs);

%sceneName   = ['sceneA', 'sceneB', 'sceneC', 'case1', 'case2', 'case3'];
sceneName   = {'sceneA', 'sceneB', 'sceneC', 'case2', 'case3'};
sceneNamePath = {'/res_cRRT_0.txt', '/res_pRRT_0.txt', '/res_Priority_0.txt', '/res_Coop_0_0.txt', '/res_Coop_0_2.txt', '/res_Coop_0_5.txt', '/res_Coop_0_10.txt'
                 '/res_cRRT_1.txt', '/res_pRRT_1.txt', '/res_Priority_1.txt', '/res_Coop_1_0.txt', '/res_Coop_1_2.txt', '/res_Coop_1_5.txt', '/res_Coop_1_10.txt';};
                 
rstart = 1;
rend = 0;
for i = 1:size(sceneName,2)
  s = scene_scale(i);
  if strcmp(sceneName{1,i}(1,1),'s') == 1
    StatData = {};
    StatData(1) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,CRRT})))};
    StatData(2) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,PRRT})))};
    StatData(3) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,PGUST})))};
    StatData(4) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,COOP0})))};
    StatData(5) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,COOP5})))};

    rend = rstart+size(StatData{1},1)-1;
    
    for k = 1:size(StatData{1},1)
       xlabels{1,size(xlabels,2)+1} = num2str(StatData{1}(k,I_NR_ROBOTS));
    end
    
    for p = 1:5
      plot (rstart:1:rend,StatData{p}(1:size(StatData{1},1), I_PCOST)/s,...
      'marker',markers{p},...
      'color',markersColor(p,:),...
      'markeredgecolor', markersColor(p,:),...
      'linestyle','-',...
      'markersize', 10,...
      'linewidth',linewidth);
    end

    for p = 1:5
      y = StatData{p}(1:size(StatData{1},1), I_PCOST)/s;
      x = rstart:1:rend;
      ystd = StatData{p}(1:size(StatData{1},1), I_PCOST+1)/s;
      for k = 1 : 1 : length(ystd)
        line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', linewidth);
      end
    end
    line([rend + 0.5, rend + 0.5], get(gca, 'ylim'), 'linewidth', linewidth+1);
    text(rstart+2.5, 1/4, sceneName(1,i),...
        'HorizontalAlignment', 'center',... 
        'fontsize', fontsize,...
        'fontname','Helvetica');
        
    rstart = rend+1;
  end
  if strcmp(sceneName{1,i}(1,1),'c') == 1
    sname = char(strcat('s',sceneName{1,i}(1,5)));
    StatData = {};
    StatData(1) = {0};
    StatData(2) = {0};
    StatData(3) = {0};
    StatData(4) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,COOP0})))};
    StatData(5) = {ReadResults(char(strcat(sceneName(1,i),sceneNamePath{robotType,COOP5})))};
    rend = rstart;
    
    for k = 1:size(StatData{5},1)
       xlabels{1,size(xlabels,2)+1} = num2str(StatData{5}(k,I_NR_ROBOTS));
    end
    
    for p = 4:5
      plot ([rstart],StatData{p}(1:size(StatData{p},1), I_PCOST),...
      'marker',markers{p},...
      'color',markersColor(p,:),...
      'markeredgecolor', markersColor(p,:),...
      'linestyle','-',...
      'markersize', 10,...
      'linewidth',linewidth);
    end
    
    for p = 4:5
      y = StatData{p}(1:size(StatData{p},1), I_PCOST);
      x = rstart:1:rend;
      ystd = StatData{p}(1:size(StatData{p},1), I_PCOST+1);
      for k = 1 : 1 : length(ystd)
        line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
      end
    end
    line([rend + 0.5, rend + 0.5], get(gca, 'ylim'), 'linewidth', linewidth);
    text(rstart, 1/4, sname,...
        'HorizontalAlignment', 'center',... 
        'fontsize', fontsize,...
        'fontname','Helvetica');
        
    rstart = rend+1;
  end
  
end

legend(plannerlabels{1,CRRT},plannerlabels{1,PRRT},plannerlabels{1,PGUST},plannerlabels{1,COOP0},plannerlabels{1,COOP5});
legend('Location','bestoutside')

set(gca, 'xlim', [0.5, rend+ 0.5]);
set(gca, 'xtick', 1:1:length(xlabels));
set(gca, 'xticklabel', xlabels);

ylabel('solution length [diag(W)]')
xlabel('nr.of robots')

end




