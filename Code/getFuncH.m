function [gmPDF] = getFuncH(option)
% GETFUNCH This function creates a discretized gaussian misture distribution
% vertical = y axis ; horizontal = x axis
% 
% Input parameter:
%       option -> choose one of the predefined distributions
% Outputs:
%       gmPDF -> Class with the distribution information
%           model -> gmdistribution model object using the specified means 
%               mu and covariances sigma with equal mixing proportions.
%           values -> discretized values [m by n]
%           grid -> m*n by 2 array with all the coorditanes
%           gridx -> m by n matrix, where each column has a diferent value
%               of x (each row is a copy of x)
%           gridy -> m by n matrix, where each row has a diferent value
%               of y (each column is a copy of x)
%           supportx -> 1 by n array with x values
%           supporty -> 1 by m array with y values
%           sizex -> size of the x axis range
%           sizey -> size of the y axis range
    
    if(option == 1)
        % Distribution with 6 gaussians
        mu = [3 3;-6 6; 0 0; 6 -4; -4 6; -5 -5];
        sigma = cat(3,[2 5],[2 5], [2 2], [5 2], [2 5], [10 1]); % 1-by-2-by-2 array
        gm = gmdistribution(mu,sigma); 

        %Creating a grid matrix with all the desired coordinates
        stepSize = 0.25;
        supportx = (-10:stepSize:10);
        supporty = (-10:stepSize:10);
        [gridx, gridy] = meshgrid(supportx, supporty);
        grid = [gridx(:) gridy(:)]; %This is the grid in a (m*n) by 2 array
    end
        
    if(option == 2)
        % Distribution with 4 gaussians
        mu = [-7 0; 8 0; 0 9; 0 -7; 12 12];
        sigma = cat(3,[3 3], [4 1], [1 4], [7 1], [4 4]); % 1-by-2-by-2 array
        gm = gmdistribution(mu,sigma); 

        %Creating a grid matrix with all the desired coordinates
        stepSize = 0.25;
        supportx = (-10:stepSize:15);
        supporty = (-10:stepSize:15);
        [gridx, gridy] = meshgrid(supportx, supporty);
        grid = [gridx(:) gridy(:)]; %This is the grid in a (m*n) by 2 array
    end

    if(option == 3)
        % Distribution with 4 gaussians
        mu = [-7 0; 7 0; 0 7; 0 -7];
        sigma = cat(3,[2 2], [5 2], [2 5], [10 1]); % 1-by-2-by-2 array
        gm = gmdistribution(mu,sigma); 

        %Creating a grid matrix with all the desired coordinates
        stepSize = 0.25;
        supportx = (-10:stepSize:10);
        supporty = (-10:stepSize:10);
        [gridx, gridy] = meshgrid(supportx, supporty);
        grid = [gridx(:) gridy(:)]; %This is the grid in a (m*n) by 2 array
    end
    
    if(option == 4)
        % Distribution with 6 gaussians
        mu = [15 15;-30 30; 0 0; 30 -20; -20 30; -25 -25];
        sigma = cat(3,[8 20],[8 20], [8 8], [20 8], [8 20], [30 5]); % 1-by-2-by-2 array
        gm = gmdistribution(mu,sigma); 

        %Creating a grid matrix with all the desired coordinates
        stepSize = 1;
        supportx = (-50:stepSize:50);
        supporty = (-50:stepSize:50);
        [gridx, gridy] = meshgrid(supportx, supporty);
        grid = [gridx(:) gridy(:)]; %This is the grid in a (m*n) by 2 array
    end
    
    if(option == 5)
        % Distribution with 6 gaussians
        mu = [4 4;-6 6; 0 0; 6 -4; -4 6; -7 -8];
        sigma = cat(3,[1 3],[1 3], [1 1], [3 1], [1 3], [5 0.5]); % 1-by-2-by-2 array
        gm = gmdistribution(mu,sigma); 

        %Creating a grid matrix with all the desired coordinates
        stepSize = 0.25;
        supportx = (-10:stepSize:10);
        supporty = (-10:stepSize:10);
        [gridx, gridy] = meshgrid(supportx, supporty);
        grid = [gridx(:) gridy(:)]; %This is the grid in a (m*n) by 2 array
    end
    
    gmPDFValues = pdf(gm, grid); %pdf evaluated in all the grid points ( (m*n) by 1 array)
    gmPDFValues = gmPDFValues./sum(sum(gmPDFValues)); % %Normalizing pdf
    gmPDFValues = reshape(gmPDFValues,length(supporty),length(supportx)); %reshape from (m*n) by 1 to m by n 

    % Creating a class with all the distribution information
    gmPDF.model = gm;
    gmPDF.values = gmPDFValues;
    gmPDF.originalValues = gmPDFValues;
    gmPDF.stepSize = stepSize;
    gmPDF.grid = grid;
    gmPDF.gridx = gridx;
    gmPDF.gridy = gridy;
    gmPDF.supportx = supportx;
    gmPDF.supporty = supporty;
    gmPDF.sizex = size(gmPDF.values,2);
    gmPDF.sizey = size(gmPDF.values,1);
end