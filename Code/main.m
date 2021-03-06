function metricResults = main()
    clear all
    %Choose the set of test to be performed
%     testList = {'Greedy', 'GradDesc', 'GradDesc', 'GradDesc'};     
%     methodOption = [0, 1.4, 2, 5];
%     %testList = {'Greedy', 'GradDesc 1.4', 'GradDesc 2', 'GradDesc 5'};

%     testList = {'GradDesc', 'Adaptive', 'Adaptive', 'Adaptive'};  
%     methodOption = [1.4 0   0   0   0   0   0   0; 
%                     1.2 1.2 0.8 0.8 1   1   3   3;
%                     1.2 1.2 0.8 0.8 0.9 0.9 2.5 2.5; 
%                     1.2 1.2 0.7 0.7 1.4 1.4 2.5 2.5;];
   %testList = {'GradDesc 1.4', 'Adaptive [1.2 0.8 1 3]', 'Adaptive [1.2 0.8 0.9 2.5]', 'Adaptive [1.2 0.7 1.4 2.5]'};
   %testList = {'GradDesc 1.4', 'Adaptive [1.2 0.8 1 3]', 'Adaptive [1.2 0.7 1 3]', 'Adaptive [1.2 0.7 1.3 4]'};

    testList = {'GradDesc', 'General', 'General', 'General'};
    methodOption = [1.4 0   0   0; 
                    1.5 0.8 0   0;
                    1.4 0.9 0.9 0; 
                    1   0.6 0.6 0.4];
    %testList = {'GradDesc 1.4', 'Heavy [1.5 0.8]', 'Nesterov [1.4 0.9 0.9]', 'Triple [1 0.6 0.6 0.4]'};

%     testList = {'GradDesc', 'Hybrid', 'Hybrid', 'Hybrid'};
%     
%     Nglobal = {'General', 'General', 'General', 'General'};
%     Pglobal = [1.4   0   0   0   0   0   0   0; 
%                1.2   0.6 0.6 0.5   0   0   0   0; %TM
%                2   0.9   0.9   0   0   0   0   0; %Nesterov
%                2.2   0.7 0.7   0.1 0   0   0   0]; %TM
%     
%     Nlocal = {'General', 'General', 'General', 'General'}  ;    
%     Plocal = [1.4 0    0   0 0 0 0 0; 
%               1.4 0  0   0 0 0 0 0;
%               1.3 0.8  0   0 0 0 0 0; 
%               1.4 0.1  0.1  0  0 0   0   0];
%                      
%     %testList = {'GradDesc 1.4', 'Hybrid 1', 'Hybrid 2', 'Hybrid 3'};

    videoOn = 0;
    graphicsOn = 0*[0, 0, 0, 1];
    waypointRefreshing = 15; numberOfGeneratedWaypoints = 1000;

    for i=1:length(testList)
        if(~isempty(find(testList == "Hybrid")))
            if(testList(i) == "GradDesc")
                methodOption = Pglobal(i);
            else
                clear methodOption;
                methodOption.local = Plocal(i, :);
                methodOption.localName = Nlocal{i};
                methodOption.global = Pglobal(i, :);
                methodOption.globalName = Nglobal{i};
            end
        end
        
        [metric, ~, dataLogClass] = droneWithWaypoints(testList(i), ...
                methodOption(i, :), 2,waypointRefreshing, numberOfGeneratedWaypoints, 1, graphicsOn(i), videoOn);
        if(videoOn)
            clf
            figure(gcf); annotation('textbox', [0.3, 0.4, 0.4, 0.2], 'String', strcat("Mean = ", num2str(metric.meanVisiting)))
            dataLogClass.video(end:end+20) = repmat(getframe(gcf), [1, 21]);
            saveMovie(dataLogClass.video, testList(i));
        end

        metricResults(i) = metric;

    end
    plotMetric(metricResults, numberOfGeneratedWaypoints*waypointRefreshing);
end



function plotMetric(metricResults, totalNumIt)
    %//////////////////////////////////////////////////////////////////
    testList = {'GradDesc 1.4', 'Adaptive [1.2 0.8 1 3]', 'Adaptive [1.2 0.8 0.9 2.5]', 'Adaptive [1.2 0.7 1.4 2.5]'};
    %//////////////////////////////////////////////////////////////////
    
    %sum
    figure;
    set(gca,'FontSize',16)
    ax = gca;
    ax.GridAlpha = 0.4;
    for i=1:length(metricResults)
        plot(1:totalNumIt, metricResults(i).sum,'LineWidth',1);
        hold on;
    end
    ylabel("Path Sum");
    xlabel("Iteration");
    grid on;
    grid minor
    legend(testList, 'Location', 'Best')
    print(gcf, 'C:\Users\zecoe\Desktop\Tese\Imagens\Tempo\SumT4M2.jpg', '-djpeg')
    
    %meanVisiting
    figure;
    set(gca,'FontSize',16)
    ax = gca;
    ax.GridAlpha = 0.4;
    for i=1:length(metricResults)
        plot(1:totalNumIt, metricResults(i).meanVisiting,'LineWidth',1);
        hold on;
    end
    %yline(1/(metricResults(1).originalgmPDF.sizex*metricResults(1).originalgmPDF.sizey), '-- k');
    ylabel("Mean Step Value");
    xlabel("Iteration");
    %testList{end+1} = "Reference";
    grid minor
    legend(testList, 'Location', 'Best')
    print(gcf, 'C:\Users\zecoe\Desktop\Tese\Imagens\Tempo\MeanStepT4M2.jpg', '-djpeg')
    
end