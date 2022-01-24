classdef adaptiveStepStrategyClass
% ADAPTATIVESTEPSTRATEGYCLASS This class keeps all the needed properties
% and methods for a minimization algorithm based on grandient descent with
% adapative step. The step updating uses the Almeida&Silva learnign method.
% In each iteration, if the current gradient direction is the same as
% the one in the previous one, then step size increases. Otherwise it will
% decrease. The method may use 
    properties
        localGradPrev = [0 0]; % Gradient value in the prevous iteration
        alpha_x; % Current step size in the x axis
        alpha_y; % Current step size in the y axis
        up_x;
        up_y;
        down_x;
        down_y;
        min_x;
        min_y;
        max_x;
        max_y;
    end

    methods
        function obj = adaptiveStepStrategyClass(alpha_x, alpha_y, up_x, up_y, down_x, down_y, min_x, min_y, max_x, max_y)
            obj.alpha_x = alpha_x;
            obj.alpha_y = alpha_y;
            obj.up_x = up_x;
            obj.up_y = up_y;
            obj.down_x = down_x;
            obj.down_y = down_y;
            obj.min_x = min_x;
            obj.min_y = min_y;
            obj.max_x = max_x;
            obj.max_y = max_y;
        end
        
        function [obj, Wx, Wy] = step(obj, gmPDF, drone)
            raio=3;
            xx=max(drone.idx-raio, 1):min(drone.idx+raio, gmPDF.sizex);
            yy=max(drone.idy-raio, 1):min(drone.idy+raio, gmPDF.sizey);
            neighbors = gmPDF.values(yy, xx); 
            
            grad_z1 = getGradient(neighbors);
            grad_z1 = grad_z1./norm(grad_z1); %Normalizing the gradient

            %Almeida&Silva learnign method
            if grad_z1(1)*obj.localGradPrev(1) >= 0
                obj.alpha_x = min(obj.up_x*obj.alpha_x, obj.max_x);
            else
                obj.alpha_x = max(obj.down_x*obj.alpha_x, obj.min_x);
            end

            if grad_z1(2)*obj.localGradPrev(2) >= 0
                obj.alpha_y = min(obj.up_y*obj.alpha_y, obj.max_y);
            else
                obj.alpha_y =  max(obj.down_y*obj.alpha_y, obj.min_y);
            end

            Wx=drone.x+obj.alpha_x*grad_z1(1);
            Wy=drone.y+obj.alpha_y*grad_z1(2);
            
            obj.localGradPrev = grad_z1;
        end
        
        function obj = updateData(obj, ~, ~)     
    
        end
    end
   
end