clc
clear all

cRRT      = ReadResults('sceneA/res_cRRT_0.txt');
pRRT      = ReadResults('sceneA/res_pRRT_0.txt');
pEP       = ReadResults('sceneA/res_Priority_0.txt');
coop0     = ReadResults('sceneA/res_Coop_0_0.txt');
coop2     = ReadResults('sceneA/res_Coop_0_2.txt');
coop5     = ReadResults('sceneA/res_Coop_0_5.txt');
coop10    = ReadResults('sceneA/res_Coop_0_10.txt');
hold on;
n = 6;
cRRT(1:n,2)
pRRT(1:n,2)
figure
subplot(3,1,1)       
bar(coop2(1:n,1),[cRRT(1:n,2) pRRT(1:n,2) pEP(1:n,2) coop0(1:n,2) coop2(1:n,2) coop5(1:n,2) coop10(1:n,2)])
subplot(3,1,2)       
bar(coop2(1:n,1),[cRRT(1:n,4) pRRT(1:n,4) coop0(1:n,4) coop2(1:n,4) coop5(1:n,4) coop10(1:n,4) pEP(1:n,4)    ])
subplot(3,1,3)       
bar(coop2(1:n,1),[cRRT(1:n,6) pRRT(1:n,6) coop0(1:n,6) coop2(1:n,6) coop5(1:n,6) coop10(1:n,6) pEP(1:n,6)    ])
legend('cRRT0000','pRRT','pGUST','CoopPAS','CoopAS2','CoopAS5','CoopAS10')

legend('Location','bestoutside')
xlabel('nrRobot');
ylabel('avergage tine');