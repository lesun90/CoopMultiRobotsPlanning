clc
clear all
hold on;
n = 1;
id = 4;
%cRRT      = ReadResults('case1/res_cRRT_0.txt');
%pRRT      = ReadResults('case1/res_pRRT_0.txt');
%pEP       = ReadResults('case1/res_Priority_0.txt');
coop0     = ReadResults('sceneB/res_Coop_0_0.txt');
coop2     = ReadResults('sceneB/res_Coop_0_2.txt');
coop5     = ReadResults('sceneB/res_Coop_0_5.txt');
coop10    = ReadResults('sceneB/res_Coop_0_10.txt');


%cRRT(1:n,id)
%pEP(1:n,id)
coop0(1:n,id)
coop2(1:n,id)
coop5(1:n,id)
coop10(1:n,id)
subplot(3,1,1)       
bar([coop0(1:n,id) coop2(1:n,id) coop5(1:n,id) coop10(1:n,id)])
legend('cRRT','pRRT','pGUST','CoopPAS','CoopAS2','CoopAS5','CoopAS10')

legend('Location','bestoutside')
xlabel('nrRobot');
ylabel('avergage tine');

%n=1;
%coop0     = ReadResults('case2/res_Coop_1_0.txt');
%coop2     = ReadResults('case2/res_Coop_1_2.txt');
%coop5     = ReadResults('case2/res_Coop_1_5.txt');
%coop10    = ReadResults('case2/res_Coop_1_10.txt');    
%subplot(3,1,1)
%bar([[coop0(1,2) coop0(1,4)] [coop10(1,2) coop10(1,4)]] );
%
%
%legend('Location','bestoutside')
%xlabel('nrRobot');
%ylabel('avergage tine');