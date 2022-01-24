function [waypoint, metric] = detectStops(gmPDF, metric, waypoint, i)
    global WpsSquare
    global WpBuffer
    
    bufferSize = 10;
    WpBuffer(1:2, mod(i, bufferSize)+1) = [waypoint.x waypoint.y]';
    
    if(size(WpBuffer, 2) < bufferSize-1)
        WpsSquare.stuckIterations = 0;
        return;
    end
    
    WpsSquare.max_x = max(WpBuffer(1, 2:end));
    WpsSquare.min_x = min(WpBuffer(1, 2:end));
    WpsSquare.max_y = max(WpBuffer(2, 2:end));
    WpsSquare.min_y = min(WpBuffer(2, 2:end));
    
    area = abs(WpsSquare.max_x-WpsSquare.min_x)*abs(WpsSquare.max_y-WpsSquare.min_y);
    
    if(area < 30*gmPDF.stepSize^2)
        jump = 2+WpsSquare.stuckIterations;
        waypoint.x = waypoint.x+(jump*rand()-jump/2)*gmPDF.stepSize;
        waypoint.y = waypoint.y+(jump*rand()-jump/2)*gmPDF.stepSize;
        WpsSquare.stuckIterations = WpsSquare.stuckIterations + 1;
        metric.detectStopsUsed = metric.detectStopsUsed + 1;
    else
        WpsSquare.stuckIterations = 0;
    end
    

end