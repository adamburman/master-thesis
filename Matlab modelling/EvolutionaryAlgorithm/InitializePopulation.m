function population = InitializePopulation(populationSize,numberOfGenes)
    
    % Instead of using a slow for-loop we initialize the entire matrix with
    % one line. 
    % P.S. I am actively making the decision that this way of
    % initializing the population should be simple enough to understand to
    % not warrant issues.
    % population = randi([0, 1], populationSize, numberOfGenes); % Original
    population = zeros(populationSize,numberOfGenes);
    population(:, 1) = 40*rand(populationSize, 1);
    for i = 2:numberOfGenes
        population(:, i) = population(:, i-1) + 5*(rand(populationSize, 1) - 0.5);
    end
    population(population > 40) = 40;
    population(population < 0) = 0;
end
