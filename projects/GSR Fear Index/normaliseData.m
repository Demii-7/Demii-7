%The following function normalizes GSR values to fit a 0-100 range. It
%accepts one arguments: GSRData-the GSR data 3D cell array
%and returns a filtered 3D cell array.

function normalisedData = normaliseData(GSRData)
    %Copy GSRData to normalised data array
    normalisedData = GSRData;

    for i = 1:length(GSRData{2})
        normalisedData{2}(i) = ((GSRData{2}(i) - min(GSRData{2})) / (max(GSRData{2})-min(GSRData{2})))* 100 ;
    end

    % Display the unfiltered GSR data
    figure;
    subplot(2, 1, 1);
    plot(GSRData{1}, GSRData{2});
    title('GRAPH OF GSR DATA BEFORE NORMALISATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    % Display the filtered GSR data
    subplot(2, 1, 2);
    plot(normalisedData{1}, normalisedData{2}, 'r');
    title('GRAPH OF GSR DATA AFTER NORMALISATION');
    xlabel('Time (s)');
    ylabel('GSR Values (uS)');
    
    %Saves current figure
    saveas(gcf, 'Normalised', 'jpg');

end