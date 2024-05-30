function uout = ExplicitMPCControllerForCellSim(currentr, currentx,t)

    assign(x{1}, explicit_states(:, k));
    explicit_inputs(k) = value(Optimizer);
end