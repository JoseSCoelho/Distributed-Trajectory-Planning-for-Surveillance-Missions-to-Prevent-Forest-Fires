function gmPDF = updateFunc(gmPDF, drone, option)
    if (option=="pathPDF")
        [drone.x, drone.y, drone.idx, drone.idy] = Arredondamento(gmPDF, drone.State(1), drone.State(2));
        updateRatio=[1.2 1.3 1.25; 1.1 1.5 1.3; 1.1 1.2 1.25];
        
        raio = 1;
        [prev, xx, yy] = getNeighbors(gmPDF.values, drone, raio);
        updateRatio_ = updateRatio(yy-drone.idy+2, xx-drone.idx+2);
            
        gmPDF.values(yy, xx) = prev./updateRatio_;
    end
    if(option=="waypointsPDF")
        [drone.x, drone.y, drone.idx, drone.idy] = Arredondamento(gmPDF, drone.x, drone.y);
        
        raio = 0;
        gmPDF.values(max(drone.idy-raio, 1):min(drone.idy+raio, gmPDF.sizey), ...
            max(drone.idx-raio, 1):min(drone.idx+raio, gmPDF.sizex)) = 0;
    end

    gmPDF.values = gmPDF.values./abs(sum(sum(gmPDF.values))); % %Normalizing pdf    
end