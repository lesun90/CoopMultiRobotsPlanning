function [stat] = ReadResults(fnameStats)
cutoff = 5;
res = load(fnameStats);
nrRobots = res(:,3);
nrRobots = unique(nrRobots);

stat = [];

for r = 1 : 1 : length(nrRobots)
  runTime = [];
  totalTime = [];
  preprocessTime = [];
  avgPathCost = [];
  pathFindingTime = [];
  simulateTime = [];
  collisionTime = [];
  for i = 1 : 1 : size(res,1)
    if res(i,3) == nrRobots(r)
      if(res(i,1) == 1)
        runTime = [runTime res(i,2)];
      end
      if(res(i,1) == 0)
        runTime = [runTime 90];
      end
      preprocessTime = [preprocessTime res(i,4)];
      avgPathCost = [avgPathCost res(i,14)];
      pathFindingTime = [pathFindingTime res(i,7)];
      simulateTime = [simulateTime res(i,5)];
      collisionTime = [collisionTime res(i,6)];
    end
  end
  totalTime = runTime + preprocessTime;
  totalTime = sort(totalTime);
  minTotalTime = min(totalTime);
  totalTime = totalTime(cutoff+1 : length(totalTime)-cutoff);
  AvgTotalTime = mean(totalTime);
  StdTotalTime = std(totalTime);

  if (minTotalTime >= 89)
  AvgTotalTime = 100;
  end

  preprocessTime = sort(preprocessTime);
  preprocessTime = preprocessTime(cutoff+1 : length(preprocessTime)-cutoff);
  AvgPpreprocessTime = mean(preprocessTime);
  StdPpreprocessTime = std(preprocessTime);

  avgPathCost = sort(avgPathCost);
  maxPathCost = max(avgPathCost);
  if maxPathCost < 0
    maxPathCost = 1000;
  end
  for k = 1 : 1 : length(avgPathCost)
    if avgPathCost(k) == -1 % not solved
      avgPathCost(k) = maxPathCost;
    end
  end
  avgPathCost = avgPathCost(cutoff+1 : length(avgPathCost)-cutoff);
  AvgOfAvgPathCosts = mean(avgPathCost);
  StdAvgPathCosts = std(avgPathCost);

  pathFindingTime = sort(pathFindingTime);
  pathFindingTime = pathFindingTime(cutoff+1 : length(pathFindingTime)-cutoff);
  AvgPathFindingTime = mean(pathFindingTime);
  StdPathFindingTime = std(pathFindingTime);

  simulateTime = sort(simulateTime);
  maxsimulateTime = max(simulateTime);
  if maxsimulateTime < 0
    maxsimulateTime = 45;
  end
  for k = 1 : 1 : length(simulateTime)
    if simulateTime(k) == -1 % not solved
      simulateTime(k) = maxsimulateTime;
    end
  end
  simulateTime = simulateTime(cutoff+1 : length(simulateTime)-cutoff);
  avgSimulateTime = mean(simulateTime);
  stdSimulateTime = std(simulateTime);

  collisionTime = sort(collisionTime);
  maxcollisionTime = max(collisionTime);
  if maxcollisionTime < 0
    maxcollisionTime = 45;
  end
  for k = 1 : 1 : length(collisionTime)
    if collisionTime(k) == -1 % not solved
      collisionTime(k) = maxcollisionTime;
    end
  end
  collisionTime = collisionTime(cutoff+1 : length(collisionTime)-cutoff);
  avgCollisionTime = mean(collisionTime);
  stdCollisionTime = std(collisionTime);

  if (AvgTotalTime >= 70)
  AvgOfAvgPathCosts = 1000;
  end

  stat = [stat; [nrRobots(r) AvgTotalTime StdTotalTime AvgPpreprocessTime StdPpreprocessTime AvgOfAvgPathCosts StdAvgPathCosts AvgPathFindingTime StdPathFindingTime avgSimulateTime stdSimulateTime avgCollisionTime stdCollisionTime]];

end
end
