function selectedIndividualIndex = TournamentSelect(fitnessList, tournamentProbability, tournamentSize)
    
    populationSize = length(fitnessList);
    for i = 1:populationSize
        tournamentIndividuals = randperm(populationSize, tournamentSize);
        tournamentIndividualsFitnessValues = fitnessList(tournamentIndividuals);
        [~, sortedIndeces] = sort(tournamentIndividualsFitnessValues, 'descend');
        sortedTournamentIndividuals = tournamentIndividuals(sortedIndeces);
        probabilityVector = tournamentProbability * (1- tournamentProbability).^(0:tournamentSize-1);
        probabilityVector = probabilityVector / sum(probabilityVector);
        cumulativeProbability = [0 cumsum(probabilityVector)];
        r = rand;
        winnerIndex = find(r > cumulativeProbability, 1 ,'last');
        selectedIndividualIndex = sortedTournamentIndividuals(winnerIndex);
    end
end
