%The following function creates a nth order median filter for data set. It
%accepts two arguments: N-the median order and GSRData-the GSR data 3D cell
%array and returns a filtered 3D cell array.

function filteredGSRData = nthOrderMedianFilter(N, GSRData)
    %Copy GSRData to filtered data array
    filteredGSRData = GSRData;
    %Loops through each GSR data value and replaces it with the median of
    %teh values within the order window
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
        filteredGSRData{2}(i) = median(subset);
    end

    % Display the unfiltered GSR data
    figure;
    subplot(2, 1, 1);
    plot(GSRData{1}, GSRData{2});
    title('GRAPH OF GSR DATA BEFORE MEDIAN FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');

    % Display the filtered GSR data
    subplot(2, 1, 2);
    plot(filteredGSRData{1}, filteredGSRData{2}, 'r');
    title('GRAPH OF GSR DATA AFTER MEDIAN FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    %Saves current figure
    saveas(gcf, 'Med_filt', 'jpg');
end