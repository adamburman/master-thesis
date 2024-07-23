%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameter specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numberOfRuns = 100;                % Do NOT change
populationSize = 100;              % Do NOT change
maximumVariableValue = 5;          % Do NOT change (x_i in [-a,a], where a = maximumVariableValue)
numberOfGenes = 50;                % Do NOT change
numberOfVariables = 2;		   % Do NOT change
numberOfGenerations = 300;         % Do NOT change
tournamentSize = 2;                % Do NOT change
tournamentProbability = 0.75;      % Do NOT change
crossoverProbability = 0.8;        % Do NOT change


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Batch runs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mutationProbability = 0.0;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList00 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList00(i,1) = maximumFitness;
end

mutationProbability = 0.01;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList001 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList001(i,1) = maximumFitness;
end

% Define more runs here (pMut < 0.02) ...

mutationProbability = 0.02;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList002 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList002(i,1) = maximumFitness;
end


% ... and here (pMut > 0.02)

mutationProbability = 0.04;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList004 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList004(i,1) = maximumFitness;
end

mutationProbability = 0.08;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList008 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList008(i,1) = maximumFitness;
end

mutationProbability = 0.16;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList016 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList016(i,1) = maximumFitness;
end

mutationProbability = 0.32;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList032 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList032(i,1) = maximumFitness;
end

mutationProbability = 0.5;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList05 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList05(i,1) = maximumFitness;
end

mutationProbability = 0.64;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList064 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList064(i,1) = maximumFitness;
end

mutationProbability = 1.0;
sprintf('Mutation rate = %0.5f', mutationProbability)
maximumFitnessList10 = zeros(numberOfRuns,1);
for i = 1:numberOfRuns 
 [maximumFitness, bestVariableValues]  = RunFunctionOptimization(populationSize, numberOfGenes, numberOfVariables, maximumVariableValue, tournamentSize, ...
                                       tournamentProbability, crossoverProbability, mutationProbability, numberOfGenerations);
 sprintf('Run: %d, Score: %0.10f', i, maximumFitness)
  maximumFitnessList10(i,1) = maximumFitness;
end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Summary of results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

average00 = mean(maximumFitnessList00);
median00 = median(maximumFitnessList00);
std00 = sqrt(var(maximumFitnessList00));
sprintf('PMut = 0.0: Median: %0.10f, Average: %0.10f, STD: %0.10f', median00, average00, std00)

average001 = mean(maximumFitnessList001);
median001 = median(maximumFitnessList001);
std001 = sqrt(var(maximumFitnessList001));
sprintf('PMut = 0.01: Median: %0.10f, Average: %0.10f, STD: %0.10f', median001, average001, std001)

% Add more results summaries here (pMut < 0.02) ...

average002 = mean(maximumFitnessList002);
median002 = median(maximumFitnessList002);
std002 = sqrt(var(maximumFitnessList002));
sprintf('PMut = 0.02: Median: %0.10f, Average: %0.10f, STD: %0.10f', median002, average002, std002)

% ... and here (pMut > 0.02)

average004 = mean(maximumFitnessList004);
median004 = median(maximumFitnessList004);
std004 = sqrt(var(maximumFitnessList004));
sprintf('PMut = 0.04: Median: %0.10f, Average: %0.10f, STD: %0.10f', median004, average004, std004)

average008 = mean(maximumFitnessList008);
median008 = median(maximumFitnessList008);
std008 = sqrt(var(maximumFitnessList008));
sprintf('PMut = 0.08: Median: %0.10f, Average: %0.10f, STD: %0.10f', median008, average008, std008)

average016 = mean(maximumFitnessList016);
median016 = median(maximumFitnessList016);
std016 = sqrt(var(maximumFitnessList016));
sprintf('PMut = 0.16: Median: %0.10f, Average: %0.10f, STD: %0.10f', median016, average016, std016)

average032 = mean(maximumFitnessList032);
median032 = median(maximumFitnessList032);
std032 = sqrt(var(maximumFitnessList032));
sprintf('PMut = 0.32: Median: %0.10f, Average: %0.10f, STD: %0.10f', median032, average032, std032)

average05 = mean(maximumFitnessList05);
median05 = median(maximumFitnessList05);
std05 = sqrt(var(maximumFitnessList05));
sprintf('PMut = 0.5: Median: %0.10f, Average: %0.10f, STD: %0.10f', median05, average05, std05)

average064 = mean(maximumFitnessList064);
median064 = median(maximumFitnessList064);
std064 = sqrt(var(maximumFitnessList064));
sprintf('PMut = 0.64: Median: %0.10f, Average: %0.10f, STD: %0.10f', median064, average064, std064)

average10 = mean(maximumFitnessList10);
median10 = median(maximumFitnessList10);
std10 = sqrt(var(maximumFitnessList10));
sprintf('PMut = 1.0: Median: %0.10f, Average: %0.10f, STD: %0.10f', median10, average10, std10)