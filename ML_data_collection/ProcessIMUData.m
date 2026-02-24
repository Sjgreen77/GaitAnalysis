%% 1. Setup Import Options (Shared for both sets)
columnNames = {'label', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'};
opts = delimitedTextImportOptions("NumVariables", 7);
opts.VariableNames = columnNames;
opts.VariableTypes = {'char', 'double', 'double', 'double', 'double', 'double', 'double'};
opts.DataLines = [2, Inf]; 
opts.Delimiter = ",";

fs = 50;                     
windowLengthInSeconds = 1;   
samplesPerWindow = fs * windowLengthInSeconds; 

%% 2. Process TRAINING Data
disp('Processing training data...');
trainFiles = {'stationary.csv', 'fidgeting.csv', 'walking.csv'};
featuresTable = extractFeatures(trainFiles, opts, samplesPerWindow);

%% 3. Process TESTING Data
disp('Processing testing data...');
testFiles = {'test_stationary.csv', 'test_fidgeting.csv', 'test_walking.csv'};
testFeaturesTable = extractFeatures(testFiles, opts, samplesPerWindow);

disp('Done! Both tables are ready in your Workspace.');

%% --- HELPER FUNCTION ---
% This function contains your original logic. 
% Note: In MATLAB scripts, functions must go at the very end of the file.
function featureTable = extractFeatures(fileList, opts, samplesPerWindow)
    featureTable = table();
    
    for i = 1:length(fileList)
        currentData = readtable(fileList{i}, opts);
        numSamples = height(currentData);
        numCompleteWindows = floor(numSamples / samplesPerWindow);
        
        for w = 1:numCompleteWindows
            startRow = (w - 1) * samplesPerWindow + 1;
            endRow = w * samplesPerWindow;
            windowData = currentData(startRow:endRow, :);
            
            % Calculate Features (Your original logic)
            newRow = table({windowData.label{1}}, ...
                mean(windowData.ax), mean(windowData.ay), mean(windowData.az), ...
                mean(windowData.gx), mean(windowData.gy), mean(windowData.gz), ...
                std(windowData.ax),  std(windowData.ay),  std(windowData.az), ...
                std(windowData.gx),  std(windowData.gy),  std(windowData.gz));
            
            featureTable = [featureTable; newRow];
        end
    end
    
    % Format the table
    featureTable.Properties.VariableNames = {'Activity', ...
        'mean_ax', 'mean_ay', 'mean_az', 'mean_gx', 'mean_gy', 'mean_gz', ...
        'std_ax', 'std_ay', 'std_az', 'std_gx', 'std_gy', 'std_gz'};
    featureTable.Activity = categorical(featureTable.Activity);
end