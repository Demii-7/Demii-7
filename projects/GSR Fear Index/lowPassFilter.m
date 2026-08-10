%The following function creates a lowpass filter for data set. It
%accepts two arguments: Fp-the passband frequency and GSRData-the GSR data 3D cell array
%and returns a filtered 3D cell array.

function filteredGSRData = lowPassFilter(fp, GSRData)

    % GSRData{1} contains datetime values and GSRData{2} contains GSR values
    filteredGSRData = GSRData;
    
    % Calculate the time difference between consecutive time points
    timeDifference = diff(GSRData{1});
    
    % Define the window size in seconds
    meanWindowSize = 1/fp;  % Fp Hz for a 1/Fp s window
    
    % Calculate the number of data points for the window
    meanWindowPoints = round(meanWindowSize / median(timeDifference));
    
    % Apply mean filter
    filteredGSRData{2} = smoothdata(GSRData{2}, 'movmean', meanWindowPoints);

    % Display the unfiltered GSR data
    figure;
    subplot(2, 1, 1);
    plot(GSRData{1}, GSRData{2});
    title('GRAPH OF GSR DATA BEFORE LOW PASS FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    % Display the filtered GSR data
    subplot(2, 1, 2);
    plot(filteredGSRData{1}, filteredGSRData{2}, 'r');
    title('GRAPH OF GSR DATA AFTER LOW PASS FILTRATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    %Saves current figure
    saveas(gcf, 'Lowpass', 'jpg');

end