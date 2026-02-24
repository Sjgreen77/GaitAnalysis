% 1. Robust Data Loading
disp('Loading data...');

% Define the names exactly as they appear in your Arduino Serial.print
columnNames = {'label', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'};

% Create import options to skip the first row (the header) and use our names
opts = delimitedTextImportOptions("NumVariables", 7);
opts.VariableNames = columnNames;
opts.VariableTypes = {'char', 'double', 'double', 'double', 'double', 'double', 'double'};
opts.DataLines = [2, Inf]; % Start reading from line 2 to skip the header row
opts.Delimiter = ",";

stationary = readtable('stationary.csv', opts);
fidgeting = readtable('fidgeting.csv', opts);
walking = readtable('walking.csv', opts);

disp('Data loaded successfully with assigned headers.');

% Put them in a cell array so we can loop through them easily
datasets = {stationary, fidgeting, walking};

% 2. Setup our Windowing parameters
fs = 50;                     % Your sample rate is 50 Hz
windowLengthInSeconds = 1;   % Let's use 1-second windows
samplesPerWindow = fs * windowLengthInSeconds; % 50 samples per window

% 3. Create an empty table to hold our final features
featuresTable = table(); 

disp('Extracting features...');

% 4. Loop through each of the 3 datasets
for d = 1:length(datasets)
    currentData = datasets{d};
    numSamples = height(currentData);
    
    % Figure out how many full 1-second windows fit in this CSV
    numCompleteWindows = floor(numSamples / samplesPerWindow);
    
    % Loop through each window one by one
    for w = 1:numCompleteWindows
        
        % Define the start and end row for the current window
        startRow = (w - 1) * samplesPerWindow + 1;
        endRow = w * samplesPerWindow;
        
        % Extract just those 50 rows of data
        windowData = currentData(startRow:endRow, :);
        
        % --- CALCULATE FEATURES ---
        % Calculate the Mean (average) for all 6 axes
        feat_mean_ax = mean(windowData.ax);
        feat_mean_ay = mean(windowData.ay);
        feat_mean_az = mean(windowData.az);
        feat_mean_gx = mean(windowData.gx);
        feat_mean_gy = mean(windowData.gy);
        feat_mean_gz = mean(windowData.gz);
        
        % Calculate the Standard Deviation (movement intensity) for all 6 axes
        feat_std_ax = std(windowData.ax);
        feat_std_ay = std(windowData.ay);
        feat_std_az = std(windowData.az);
        feat_std_gx = std(windowData.gx);
        feat_std_gy = std(windowData.gy);
        feat_std_gz = std(windowData.gz);
        
        % Grab the label (e.g., "walking") from the first row of this window
        windowLabel = windowData.label{1}; % Using {} because it reads as text
        
        % Create a new row with our calculated features
        newRow = table({windowLabel}, feat_mean_ax, feat_mean_ay, feat_mean_az, ...
                       feat_mean_gx, feat_mean_gy, feat_mean_gz, ...
                       feat_std_ax, feat_std_ay, feat_std_az, ...
                       feat_std_gx, feat_std_gy, feat_std_gz);
                       
        % Stack this new row onto our master features table
        featuresTable = [featuresTable; newRow];
    end
end

% 5. Clean up the final table
% Rename the first column to 'Activity'
featuresTable.Properties.VariableNames{1} = 'Activity';

% Convert the text label into a "Categorical" type (Machine Learning requires this)
featuresTable.Activity = categorical(featuresTable.Activity);

disp('Done! Your featuresTable is ready.');
% Display the first 5 rows to make sure it looks right
head(featuresTable, 5)