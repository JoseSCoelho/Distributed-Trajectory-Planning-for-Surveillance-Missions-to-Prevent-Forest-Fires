classdef generalFirstOrderStrategyClass
% generalFirstOrderStrategyClass 
    properties
        state;
        alphaFunc;
        betaFunc;
        gamaFunc;
        deltaFunc;
        nonOptAlpha;
        nonOptBeta;
        nonOptGama;
        nonOptDelta;
        optimum;
    end

    methods
        function obj = generalFirstOrderStrategyClass(x, y, methodOption)
            obj.state = [x, y, x, y]';
            obj.optimum = methodOption.optimum;
            if(obj.optimum)
                obj.alphaFunc = methodOption.alphaFunc;
                obj.betaFunc = methodOption.betaFunc;
                obj.gamaFunc = methodOption.gamaFunc;
                obj.deltaFunc = methodOption.deltaFunc;
            else
                obj.nonOptAlpha = methodOption.alpha;
                obj.nonOptBeta = methodOption.beta;
                obj.nonOptGama = methodOption.gama;
                obj.nonOptDelta = methodOption.delta;
            end
        end
        
        function [obj, Wx, Wy] = step(obj, gmPDF, drone)
            obj.state(1) = drone.x; obj.state(2) = drone.y; %Update the internal state,
                            %because the drone might not get totaly aligned
                            %with the desired waypoint
            if(obj.optimum)
                [L, m, k, ~] = getQuadricParameters(gmPDF, drone);

                alpha = obj.alphaFunc(L, m, k);
                beta = obj.betaFunc(L, m, k);
                gama = obj.gamaFunc(L, m, k);
                delta = obj.deltaFunc(L, m, k);
            else
                alpha = obj.nonOptAlpha;
                beta = obj.nonOptBeta;
                gama = obj.nonOptGama;
                delta = obj.nonOptDelta;
            end
            
            %------ State Space --------            
            A = [(1+beta)*eye(2) -beta*eye(2);
                    eye(2) 0*eye(2);];

            B = [alpha*eye(2); 0*eye(2)];

            C = [(1+gama)*eye(2) -gama*eye(2);
                    (1+delta)*eye(2) -delta*eye(2);];

            %--------------------

            output = C*obj.state; %Get the point where the gradient will be computed
            [drone.x, drone.y, drone.idx, drone.idy] = Arredondamento(gmPDF, output(1), output(2));
            
            if(obj.optimum)
                [~, ~, ~, gmPDF] = getQuadricParameters(gmPDF, drone);                
                raio = 4;
                xx=max(drone.idx-raio, 1):min(drone.idx+raio, gmPDF.sizex);
                yy=max(drone.idy-raio, 1):min(drone.idy+raio, gmPDF.sizey);
                neighbors = gmPDF.values(yy, xx);
            else                
                raio=3;
                xx=max(drone.idx-raio, 1):min(drone.idx+raio, gmPDF.sizex);
                yy=max(drone.idy-raio, 1):min(drone.idy+raio, gmPDF.sizey);
                neighbors = gmPDF.values(yy, xx); 
            end
            
            grad_y = getGradient(neighbors); %gradient of gmPDF at position (x, y)
            if(~obj.optimum), grad_y=grad_y./norm(grad_y); end
            
            obj.state = A*obj.state + B*grad_y';
            output = C*obj.state;

            Wx=output(3);
            Wy=output(4);        
            
            
        end

        function obj = updateData(obj, ~, ~)     
    
        end
        
        function obj = updateState(obj, droneState)     
            obj.state = [droneState(1), droneState(2), droneState(1), droneState(2)]';
        end
    end
   
end