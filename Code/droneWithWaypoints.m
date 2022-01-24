function [metric, graphicsClass, dataLogClass] = droneWithWaypoints(methodName, ...
    methodOption,funcH,waypointRefreshing, numberOfGeneratedWaypoints, ...
    verbose, graphics, videoOn)
% DRONEWITHWAYPOINTS Simulating a drone path inside a map created as a
% probability density function. The higher the function value is

    %close all
    
    % Total number of iterations needed
    nbrOfIterations = waypointRefreshing*numberOfGeneratedWaypoints;
    
    % Defining PDF (map where the drone will travel)
    gmPDF = getFuncH(funcH);

    % 
    drone = setupDrone(gmPDF, verbose);
    method = setupMethod(methodName, methodOption, drone);
    graphicsClass = setupGraphics(graphics, gmPDF, drone.State, methodName);
    dataLogClass = setupDataLog(videoOn, gmPDF, drone, nbrOfIterations, numberOfGeneratedWaypoints);
    metric = setupMetric(gmPDF, nbrOfIterations);
    
    samplingPeriodsCounter = waypointRefreshing;
    for i=1:nbrOfIterations
        % Geting a new waypoint
        if(samplingPeriodsCounter == waypointRefreshing)
            samplingPeriodsCounter = 0;
            [method, waypoint] = getWaypoint(gmPDF, drone, method);          
            
            if(methodName ~= "Greedy")
                [waypoint, metric] = detectStops(gmPDF, metric, waypoint, floor(i/waypointRefreshing)+1);
               
            end
            
            if(graphics == true)  
                %Prepare data to plot
                data.waypoint = waypoint; data.method = method; 
                graphicsClass = graphicsPlot(graphicsClass, data, methodName, "wpMap"); 
            end
        end

        % Drone dinamics simulation
        [drone, gmPDF] = droneSimulation(drone, gmPDF, waypoint);
        
        % Updating some method data that need to be updated in evey drone
        % step
        method = method.updateData(gmPDF, drone);
        
        % Updating the map values
        gmPDF = updateFunc(gmPDF, drone, "pathPDF");
     
        metric = updateMetric(metric, drone, i,methodName, method);
        
        samplingPeriodsCounter = samplingPeriodsCounter + 1;
        
        if(graphics == true)
            %Prepare data to plot
            data.gmPDF = gmPDF; data.drone = drone; 
            graphicsClass = graphicsPlot(graphicsClass, data, methodName, "pathMap"); 
        end
        
        if(videoOn == true),  dataLogClass = dataLogUpdate(dataLogClass, i); end
        
        if(~videoOn && graphics), pause(drone.Ts/30000); end
    end
    
    metric.RMSux = sqrt(1/length(metric.u)*sum(metric.u(1, :).^2));
    metric.RMSuy = sqrt(1/length(metric.u)*sum(metric.u(2, :).^2));
    if(~graphics)
        setupGraphics(1, gmPDF, drone.State, methodName);
    end
    clear global;
end

function drone = setupDrone(gmPDF, verbose)
% SETUPDRONE This function setups the drone parameter: 
%      -> Initial position
%      -> Discrete state space representation
%      -> LQR controller
% Input parameter:
%      -> gmPDF - class that keeps map where the drone will fly. The
%      function uses the following map parameter: gridx, gridy, sizex, sizey
%      supportx and supporty
%      -> verbose - Bollean that allows textual information at cmd window
% Output:
%      -> drone - class that keeps the drone information.
    
    % Generating an initial position for the drone
    drone.idx = randi([1 length(gmPDF.gridx)]); drone.idy = randi([1 length(gmPDF.gridy)]);
    %drone.idx = 71; drone.idy = 71;
    drone.idx = 1; drone.idy = length(gmPDF.supporty);
    %Verifica se o ponto inicial está dentro do tabuleiro
    if((drone.idx > gmPDF.sizex || drone.idy > gmPDF.sizey) && verbose)
        error("The random initial point is outside the map")
    end
    
    drone.x = gmPDF.supportx(drone.idx); drone.y = gmPDF.supporty(drone.idy);
    if verbose, disp(['Drone initial position:   x=', num2str(drone.x), '   y=',num2str(drone.y)]); end
    
    % Get drone dynamics as a double integrator
    % double integrator dynamics in continuous time
    A = kron([0 1;0 0], eye(2));
    B = kron([0;1],eye(2));
    C = eye(4);
    
    drone.Ts = 0.1;

    G = c2d(ss(A,B,C,0),drone.Ts);

    drone.A = G.A;
    drone.B = G.B;
    drone.C = G.C;

    drone.K = dlqr(drone.A, drone.B, 100*eye(4), 0.1*eye(2), 0);
    drone.State = [drone.x; drone.y; 0; 0];
end

function dataLogClass = setupDataLog(dataLog, gmPDF, drone, ...
    nbrOfIterations, numberOfGeneratedWaypoints)
    if(dataLog == true)
        dataLogClass.path = zeros(4,nbrOfIterations);
        dataLogClass.waypoints = zeros(2, numberOfGeneratedWaypoints);
    else
        dataLogClass = [];
    end
end

function graphicsClass = setupGraphics(graphics, gmPDF, droneState, name)

    if(graphics == true)
        graphicsClass.gmPDFWaypoints = gmPDF;
               
        fig1 = figure;
        set(fig1,'position',[150,100,1000,500])
        pathHandle = axes('Parent',fig1); %Acho que se pode tirar esta linha
        
        % Left plot
        graphicsClass.wpHandle = subplot(1,2, 1);
        graphicsClass.wpSurf = surf(graphicsClass.wpHandle, gmPDF.supportx, gmPDF.supporty, gmPDF.values);
        graphicsClass.wpTitle = title("Generated Waypoints");
        hold on;
        graphicsClass.droneScatter = scatter3(droneState(1), droneState(2), 1, 'filled', 'r');
        %colormap(flipud(colormap))
        axis square
        view(0,90);

        %Rigth plot
        graphicsClass.pathHandle = subplot(1,2, 2);
        graphicsClass.pathSurf = surf(graphicsClass.pathHandle, gmPDF.supportx, gmPDF.supporty, gmPDF.values);
        graphicsClass.pathtitle = title("Drone Path");
        axis square
        view(0,90)
        
        set(graphicsClass.wpHandle, 'Position', [-0.1400 0.100 0.7800 0.7800]);
        set(graphicsClass.pathHandle, 'Position', [0.3603 0.100 0.7800 0.7800]);
        sgtitle(name); 
    else
        graphicsClass = [];
    end
end

function metric = setupMetric(gmPDF, nbrOfIterations)
    metric.originalgmPDF = gmPDF;
    metric.visitedValuesArray = zeros(1, nbrOfIterations);
    metric.meanVisiting = zeros(1, nbrOfIterations);
    metric.stdVisiting = zeros(1, nbrOfIterations);
    metric.sum = zeros(1, nbrOfIterations);
    metric.alpha = zeros(2, nbrOfIterations);
    metric.detectStopsUsed = 0;
    metric.u = zeros(2, nbrOfIterations);
end

function [method, waypoint] = getWaypoint(gmPDF, drone, method)
% GETWAYPOINT This function gets a new waypoint for the drone
% Input parameter:
%      -> gmPDF - class that keeps map where the drone will fly.
%      -> drone - class with the drone information.
%      -> method - class with the method methods. This class must have a
%      ".step" method, thar computes a method iteration.
% Output:
%      -> method - updated method class.
    [drone.x, drone.y, drone.idx, drone.idy] = Arredondamento(gmPDF, drone.State(1), drone.State(2));
        
    [method, waypoint.x, waypoint.y] = method.step(gmPDF, drone);
end

