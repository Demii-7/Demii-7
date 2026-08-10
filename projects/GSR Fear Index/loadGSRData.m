%The following function loads GSR Data values from a file, passed as 
%an argument and returns a matrix containing the data.

function GSRData = loadGSRData(data_file)
    %Opens the file for reading
    file = fopen(data_file, 'r');
    
    %Check if file fails to open
    if(file == -1)
        error('Error! Could not open file!');
    end
    
    %Skips File header
    fscanf(file, '%s', 2);

    % Read the time data from the file using fscanf
    data = textscan(file, '%{mm:ss:SSS}D %f', 'Delimiter', ',' );

    %Convert time to seconds by offesting all vlues by the first time
    %reading
    time = seconds(data{1}-data{1}(1));
           
    %Close file
    fclose(file);
    
    %Loads GSR Data with time data from file into a single 2 column
    %matrix
    GSRData = {time, data{2}};
end