classdef greedyStrategyClass
% greedyStrategyClass 
    properties
        neighborhoodLevel
    end

    methods
        function obj = greedyStrategyClass(neighborhoodLevel)
            obj.neighborhoodLevel = neighborhoodLevel;
        end
        
        function [obj, Wx, Wy] = step(obj, gmPDF, drone)
            neighbors = getNeighbors(gmPDF.values, drone, obj.neighborhoodLevel);
            
            aux = neighbors-gmPDF.values(drone.idy, drone.idx);
            [droneNeighbors_idy, droneNeighbors_idx]=find(aux==0);
            
            xx=max(droneNeighbors_idx-1, 1):min(droneNeighbors_idx+1, 2*obj.neighborhoodLevel+1);
            yy=max(droneNeighbors_idy-1, 1):min(droneNeighbors_idy+1, 2*obj.neighborhoodLevel+1);
            neighbors(yy, xx) = -inf;
            
            
            maximum = max(max(neighbors));
            [maxIdy, maxIdx]=find(neighbors==maximum);

            %Se existir mais do que uma celula com o maximo, escolhe-se a primeira
            %(possivel problema se todos forem iguais ao do centro/pos atual)
            if(length(maxIdy)~=1 || length(maxIdx)~=1)
                maxIdy=maxIdy(1); maxIdx=maxIdx(1); 
            end

            deltaIdy=maxIdy-droneNeighbors_idy; deltaIdx=maxIdx-droneNeighbors_idx; %Isto tem de se fazer sempre
            
            Wy=gmPDF.supporty(drone.idy+deltaIdy);
            Wx=gmPDF.supportx(drone.idx+deltaIdx);
        end
        
        function obj = updateData(obj, ~, ~)     
    
        end
    end
   
end