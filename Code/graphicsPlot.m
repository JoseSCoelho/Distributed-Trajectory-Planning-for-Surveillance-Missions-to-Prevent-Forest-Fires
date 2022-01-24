function graphicsClass = graphicsPlot (graphicsClass, data, methodName, option)
% GETWAYPOINT This function gets a new waypoint for the drone
% Input parameter:
%      -> gmPDF - class that keeps map where the drone will fly.
%      -> drone - class with the drone information.
%      -> method - class with the method methods. This class must have a
%      ".step" method, thar computes a method iteration.
% Output:
%      -> method - updated method class.

    gmPDFWaypoints = graphicsClass.gmPDFWaypoints;
    
    if(option == "wpMap")
        % Updating the image that keeps all the computed waypoints and the
        % drone current position
        gmPDFWaypoints = updateFunc(gmPDFWaypoints, data.waypoint, "waypointsPDF");
        set(graphicsClass.wpSurf, 'ZData', gmPDFWaypoints.values)
        if(methodName == "Hybrid")
            set(graphicsClass.wpTitle, 'String', {"Mode: " + data.method.q + "  Avg = " + data.method.avg});
        end
    end
   
   	if(option == "pathMap")
        % Updating the image that keeps the current state of the probably
        % distribution function
        set(graphicsClass.pathSurf, 'ZData', data.gmPDF.values)
        
        % Updating the drone positon in the 'wpMap' image
        set(graphicsClass.droneScatter, 'XData', data.drone.State(1), 'YData', data.drone.State(2))
    end
    
    graphicsClass.gmPDFWaypoints = gmPDFWaypoints;
end