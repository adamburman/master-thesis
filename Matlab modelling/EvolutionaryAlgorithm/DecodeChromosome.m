% Note: Each component of x should take values in [-a,a], where a = maximumVariableValue.

function x = DecodeChromosome(chromosome,numberOfVariables,maximumVariableValue)

    x = zeros(1, numberOfVariables);
    m = length(chromosome);
    k = m / numberOfVariables;
    for variableIndex = 1:numberOfVariables
        startingIndex = (variableIndex - 1) * k + 1;
        stopIndex = startingIndex + k - 1;
        for j = startingIndex:stopIndex
            x(variableIndex) = x(variableIndex) + chromosome(j) * 2^(-(j - startingIndex + 1));
        end
        x(variableIndex) = -maximumVariableValue + (2 * maximumVariableValue * x(variableIndex)) / (1 - 2^(-k));
    end
end

