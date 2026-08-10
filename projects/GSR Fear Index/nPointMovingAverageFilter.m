%The following function creates a n-point moving average filter for data set. It
%accepts two arguments: N-the moving average order and GSRData- the GSR data 3D cell array.
%and returns a filtered 3D cell array

function filteredGSRData = nPointMovingAverageFilter(N, GSRData)
    filteredGSRData = GSRData;

    %Calculate N-point moving avergae for other data elements
    for i = 1:length(GSRData{2})
    
         % Determine the subset based on the specified order
        subset = zeros(1,N); % Reserves memory for window GSR values
        k = 1; %Index variable for subset array
        for j = -floor((N-1)/2):ceil((N-1)/2)
            index = i + j;
            index = max(1, min(index, length(GSRData{2}))); % Ensure index is within bounds
            subset(k) = GSRData{2}(index);
            k = k+1;
        end
        filteredGSRData{2}(i) = mean(subset);
     end

    % Display the unfiltered GSR data
    figure;
    subplot(2, 1, 1);
    plot(GSRData{1}, GSRData{2});
    title('GRAPH OF GSR DATA BEFORE N-POINT MOVING AVERAGE FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    % Display the filtered GSR data
    subplot(2, 1, 2);
    plot(filteredGSRData{1}, filteredGSRData{2}, 'r');
    title('GRAPH OF GSR DATA AFTER N-POINT MOVING AVERAGE FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    %Saves current figure
    saveas(gcf, 'Moving_Average', 'jpg');
end