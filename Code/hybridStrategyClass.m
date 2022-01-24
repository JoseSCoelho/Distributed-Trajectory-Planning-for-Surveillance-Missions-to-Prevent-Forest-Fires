classdef hybridStrategyClass
    properties
        V1
        V0
        c0
        c10
        q = 1; %Inicialização em regime global
        avg;
        path_sum;
        last_pos_idx = inf;
        last_pos_idy = inf;
        currentNbrOfIteration = 0;
        
        localStrategy;
        globalStrategy;
    end

    methods
        function obj = hybridStrategyClass(localOpt, globalOpt, c0, c10, initialAvg, drone)
            obj.c0 = c0;
            obj.c10 = c10;
            obj.avg = initialAvg;
            obj.path_sum = initialAvg;
            obj.V1 = @(avg, currentValue) (avg - currentValue)/avg;
            obj.V0 = @(avg, currentValue) (avg - currentValue)/avg;
                   
            obj.localStrategy = setupMethod(localOpt.name, localOpt.option, drone);
            obj.globalStrategy = setupMethod(globalOpt.name, globalOpt.option, drone);
        end
        
        function [obj, Wx, Wy] = step(obj, gmPDF, drone)
            %First the supervisor updates the algorithm to be
            %used (local or global)
            obj = supervisor(obj, gmPDF, drone);
            
            if(obj.q == 1)
                % Regime global
                [obj.globalStrategy, Wx, Wy] = obj.globalStrategy.step(gmPDF, drone);
            end

            if(obj.q == 0)
                % Regime local
                [obj.localStrategy, Wx, Wy] = obj.localStrategy.step(gmPDF, drone);
            end        
            
        end
        
        function obj = supervisor(obj, gmPDF, drone)            
            if(obj.q == 1)
                raio=4;
                neighbors = getNeighbors(gmPDF.values, drone, raio);
                currentNeighborhoodValues = sum(sum(neighbors));
                currentNeighborhoodValues=currentNeighborhoodValues/(size(neighbors, 1)*size(neighbors, 2));
                aa = obj.V1(obj.avg, currentNeighborhoodValues);
                if(aa < obj.c10)
                    %A estimativa está proxima e o algoritmo está em regime global
                    obj.q = 0; %Altera para regime local
                    %disp("Supervisor changed from 1-->0 ");
                    obj.localStrategy = obj.localStrategy.updateState(drone.State) ;
                    return;
                end   
            end
            
            if(obj.q == 0)
                raio=4;
                neighbors = getNeighbors(gmPDF.values, drone, raio);
                currentNeighborhoodValues = sum(sum(neighbors));
                currentNeighborhoodValues=currentNeighborhoodValues/(size(neighbors, 1)*size(neighbors, 2));
                aa = obj.V1(obj.avg, currentNeighborhoodValues);
                
                if(aa > obj.c0)
                    %A estimativa está longe e o algoritmo está em regime local
                    obj.q = 1; %Altera para regime global
                    %disp("Supervisor changed from 0-->1 ");
                    return;
                end   
            end
        end
        
        function obj = updateData(obj, gmPDF, drone)            
            [~, ~, pos_idx, pos_idy] = Arredondamento(gmPDF, drone.State(1), drone.State(2));
            
            if(obj.last_pos_idx~=pos_idx || obj.last_pos_idy~=pos_idy)
                obj.currentNbrOfIteration = obj.currentNbrOfIteration + 1;
                obj.path_sum=obj.path_sum+gmPDF.values(pos_idy, pos_idx);
                obj.avg=obj.path_sum/obj.currentNbrOfIteration;
                
            end
            obj.last_pos_idx=pos_idx; obj.last_pos_idy=pos_idy;
        end
    end
end