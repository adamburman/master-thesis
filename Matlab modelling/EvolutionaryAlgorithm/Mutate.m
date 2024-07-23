function mutatedIndividual = Mutate(individual, mutationProbability)

    mutatedIndividual = individual;
    numberOfGenes = size(individual, 2);
    for i = 1:numberOfGenes
        if (rand <= mutationProbability)
            mutatedIndividual(i) = 1 - individual(i);
        end
    end
end
