function newIndividuals = Cross(individual1, individual2)
    
    chromosomeLength = length(individual1);
    newIndividuals = zeros(2, chromosomeLength);
    crossoverPoint = 1 + fix(rand() * (chromosomeLength-1));
    for i = 1:chromosomeLength
        if (i < crossoverPoint)
            newIndividuals(1, i) = individual2(i);
            newIndividuals(2, i) = individual1(i);
        else
            newIndividuals(1, i) = individual1(i);
            newIndividuals(2, i) = individual2(i);
        end
    end
end