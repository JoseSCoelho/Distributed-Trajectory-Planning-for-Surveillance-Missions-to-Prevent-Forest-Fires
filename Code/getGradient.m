function fGrad = getGradient(values)
    xx=1:1:size(values, 2);
    yy=1:1:size(values, 1);
    %values = gmPDF.values(yy, xx);

    %Ajusta uma superficie à proximidade do ponto atual
    [meshx, meshy] = meshgrid(xx, yy);
    subgrid = [meshx(:), meshy(:)];
    sf_ = fit(subgrid ,values(:),'poly11'); %Superficie continua
    sfvalues = reshape(sf_(subgrid), [length(yy),length(xx)]);
    [Nx, Ny, ~] = surfnorm(meshx,meshy,sfvalues);
    
    fGrad(1) = -Nx(1,1);
    fGrad(2) = -Ny(1,1);
    
%     figure; surf(values)
%     figure;surfnorm(meshx,meshy,sfvalues)
end