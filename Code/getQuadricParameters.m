function [L, m, k, gmPDF] = getQuadricParameters(gmPDF, drone)
    raio = 10;
    xx=max(drone.idx-raio, 1):min(drone.idx+raio, gmPDF.sizex);
    yy=max(drone.idy-raio, 1):min(drone.idy+raio, gmPDF.sizey);
    neighbors = gmPDF.values(yy, xx);
    
    [gridx, gridy] = meshgrid(xx, yy);
    
    %Fit
    sf = fit([gridx(:), gridy(:)] ,neighbors(:),'poly22');
    gmPDF.sfValues = reshape(sf([gridx(:), gridy(:)]), [length(yy),length(xx)]);
    
    QFit = [sf.p20 sf.p11/2; sf.p11/2 sf.p02];
    
    %Conclusão
    [~, eVals] = eig(QFit);

    L = max(max(abs(eVals)));
    m = min(abs(eVals(abs(eVals)>0)));
    k = L/m;

%     figure; surf(gmPDF.sfValues)
%     figure; surf(gmPDF.values(yy, xx))

end