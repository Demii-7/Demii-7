%The following program analyzes the relationship between the GSR signal and 
%the physiological response to fear and returns a corresponding fear index.

file = 'GSR_FEAR.csv';
med_order = 3; %Median filter order
n_order = 10; %N-point moiving average order
fp = 20; %Passband frequency in Hz

%Loads GSR Data
GSRData = loadGSRData(file);

%Filters GSR Data 
filteredGSRData = nPointMovingAverageFilter(n_order, lowPassFilter(fp, nthOrderMedianFilter(med_order, GSRData)));

%Normalises GSR Data to fit 100 range
normalisedGSRData = normaliseData(filteredGSRData);

%Extract Features from GSR Data and store in vector F
F = gsrFeatures(normalisedGSRData);

%Calculates Fear Index based on extracted features
FearIndex = calculateFearIndex(F);

%Displays the Fear Index
fprintf('\nThe fear index is: %.2f \n\n', FearIndex);