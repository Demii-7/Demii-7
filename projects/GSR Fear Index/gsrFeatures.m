%The following function finds 10 features using GSR Data values from a file, passed as 
%an argument and returns a vector containing the data.

function F = gsrFeatures(GSRData)
    %Mean of GSR signal (F1)
    F(1) = mean(GSRData{2}); 
    %Variance of GSR signal (F2)
    F(2) = var(GSRData{2}); 
    
    %Find maximum and minimum values and their corresponding times;
    max_pk = islocalmax(GSRData{2});
    min_pk = islocalmin(GSRData{2});
    
    %Maximum and minimum values
    maxima = GSRData{2}(max_pk);
    minima = GSRData{2}(min_pk);

    %Corresponding Maximum and minimum time values
    max_t = GSRData{1}(max_pk);
    min_t = GSRData{1}(min_pk); 

    
    % Trough-peak seperation threshold, calculated using GSR_FEAR data trough-peak separation values at 90% Percentile
    tr_pk_sep_prctl = 16.305;

    %Intitialise peak energy sum variubale to zero 
    F(3) = 0;
    F(5) = 0;
    
    %Preallocates memory to store peak ampltitudes and rise tiems
    SIZE = length(maxima);
    risetime = zeros(1,SIZE);
    amplitude = zeros(1,SIZE);

    %Finds the peak ampltidues and the risetimes using  trough-peak separation thereshold 
    for i = 1:SIZE
        %Find closet trough to peak at current position
        idx = find(min_t < max_t(i), 1, 'last');
        if ~isempty(idx)

            %Checks if ampltidues are above threshold
            if ((maxima(i) - minima(idx)) > tr_pk_sep_prctl)
                %Stores peak amplitides within range
                amplitude(i) = maxima(i);
              
                risetime(i) = max_t(i) - min_t(idx);
                
                %Calculates peak amplitude sum
                F(3) = F(3) + risetime(i);
                %Calculates peak energy sum
                F(5) = F(5) + (0.5 * amplitude(i) * risetime(i));
            end
        end
    end
    
    %Calculates peak amplitude sum
    F(4) = sum(amplitude);
    %Finds highest peak amplitude sum
    [F(6), pk_idx] = max(amplitude);
    
    %Finds corresponding rise time for highest peak amplitude 
    F(7) = risetime(pk_idx);

    %Finds number of peak amplitude 
    %Counts non-zero elements because array was preallocated with zeros of
    %size greater than the number of 'important' peak amplitudes
    F(8) = nnz(amplitude); 

    %Calculates mean power of signal
    F(9) = mean(GSRData{2}.^2);
    %Calculates Bandwidth of signals
    dSquares = sum((diff(GSRData{2})).^2);
    squares = sum(GSRData{2}.^2);
    F(10) = sqrt(dSquares/ squares)/(2*pi);
    
end