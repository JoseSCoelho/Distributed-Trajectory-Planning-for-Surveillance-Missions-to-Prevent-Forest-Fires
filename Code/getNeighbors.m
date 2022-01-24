function [neighbors, xx, yy] = getNeighbors(gmPDFValues, drone, raio)
    xx=max(drone.idx-raio, 1):min(drone.idx+raio, size(gmPDFValues, 2));
    yy=max(drone.idy-raio, 1):min(drone.idy+raio, size(gmPDFValues, 1));
    neighbors = gmPDFValues(yy, xx);    
end