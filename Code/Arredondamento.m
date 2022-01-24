function [Wx, Wy, Widx, Widy] = Arredondamento(gmPDF, Wx, Wy)

    [minValue,closestIndex_x] = min(abs(gmPDF.supportx - Wx));
    closestValue = gmPDF.supportx(closestIndex_x);
    Wx = closestValue;
    Widx = closestIndex_x;

    [minValue,closestIndex_y] = min(abs(gmPDF.supporty - Wy));
    closestValue = gmPDF.supporty(closestIndex_y);
    Wy = closestValue; 
    Widy = closestIndex_y;
        
end




