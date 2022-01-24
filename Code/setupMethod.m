
function method = setupMethod(methodName, Options, drone)
% SETUPMETHOD This function defines the minimization method that will be
%       used to generate the waypoints
% Input parameter:
%      -> methodName - name of the choosen method. ("Hybrid", "Adaptive",
%      "")
%      -> methodOption - class with the needed parameter for the choosen 
%       'methodName'
% Output:
%      -> method - class that keeps the method information.

    if(methodName == "Hybrid")
        localStrategy.name = Options.localName;
        localStrategy.option = Options.local;
        globalStrategy.name = Options.globalName;
        globalStrategy.option = Options.global;
        method = hybridStrategyClass(localStrategy, globalStrategy, 0.6, -0.6, 10^-5, drone); 
    end
    
    if(methodName == "Adaptive")
        up_x = Options(1);
        up_y = Options(2);
        down_x = Options(3); 
        down_y = Options(4);
        min_x = Options(5); 
        min_y = Options(6);
        max_x = Options(7);
        max_y = Options(8);
        method = adaptiveStepStrategyClass(1.4, 1.4, up_x, up_y, down_x, down_y, min_x, min_y, max_x, max_y);
    end
        
    if(methodName == "GradAscent Opt")
        methodOption.optimum = true;
        methodOption.alphaFunc = @(L, m, k) 2/(L+m);
        methodOption.betaFunc = @(L, m, k) 0;
        methodOption.gamaFunc = @(L, m, k) 0;
        methodOption.deltaFunc = @(L, m, k) 0;
        method = generalFirstOrderStrategyClass(drone.State(1), drone.State(2), methodOption);
    end
    
    if(methodName == "GradDesc")
        methodOption.optimum = false;
        methodOption.alpha = Options(1);
        methodOption.beta = 0;
        methodOption.gama = 0;
        methodOption.delta = 0;
        
        method = generalFirstOrderStrategyClass(drone.State(1), drone.State(2), methodOption);
    end
        
    if(methodName == "Heavy Opt")
        methodOption.optimum = true;
        methodOption.alphaFunc = @(L, m, k) 4/(sqrt(L) + sqrt(m))^2;
        methodOption.betaFunc = @(L, m, k) ((sqrt(k)-1)/(sqrt(k)+1))^2;
        methodOption.gamaFunc = @(L, m, k) 0;
        methodOption.deltaFunc = @(L, m, k) 0;
        method = generalFirstOrderStrategyClass(drone.State(1), drone.State(2), methodOption);
    end
    
    if(methodName == "Triple Opt")
        methodOption.optimum = true;
        methodOption.alphaFunc = @(L, m, k) (1+1-1/sqrt(k))/(L);
        methodOption.betaFunc = @(L, m, k) (1-1/sqrt(k))^2/(2-(1-1/sqrt(k)));
        methodOption.gamaFunc = @(L, m, k) (1-1/sqrt(k))^2/((1+(1-1/sqrt(k)))*(2-((1-1/sqrt(k)))));
        methodOption.deltaFunc = @(L, m, k) (1-1/sqrt(k))^2/(1-((1-1/sqrt(k)))^2);
        
        method = generalFirstOrderStrategyClass(drone.State(1), drone.State(2), methodOption);
    end
        
    if(methodName == "General")
        methodOption.optimum = false;
        methodOption.alpha = Options(1);
        methodOption.beta = Options(2);
        methodOption.gama = Options(3);
        methodOption.delta = Options(4);
        method = generalFirstOrderStrategyClass(drone.State(1), drone.State(2), methodOption);
    end
    
    if(methodName == "Greedy")
        method = greedyStrategyClass(2);
    end
    
    

end