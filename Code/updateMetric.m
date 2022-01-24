function metric = updateMetric(metric, drone, currentNbOfSteps, methodName, method)
    [~, ~, Widx, Widy] = Arredondamento(metric.originalgmPDF, drone.State(1), drone.State(1));

    metric.visitedValuesArray(currentNbOfSteps) = metric.originalgmPDF.values(Widy, Widx);
    metric.meanVisiting(currentNbOfSteps) = mean(nonzeros(metric.visitedValuesArray));
    metric.stdVisiting(currentNbOfSteps) = std(nonzeros(metric.visitedValuesArray));
    metric.sum(currentNbOfSteps) = sum(metric.visitedValuesArray);
    if(methodName == "Adaptive")
        metric.alpha(1, currentNbOfSteps) = method.alpha_x;
        metric.alpha(2, currentNbOfSteps) = method.alpha_y;
    end
    metric.u(:, currentNbOfSteps) = drone.u';
end