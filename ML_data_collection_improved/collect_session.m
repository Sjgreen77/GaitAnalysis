%% collect_session.m
% Run this after each activity session to pull collect.csv via BLE

activityName = 'walking';   % <-- change each time: stationary/fidgeting/walking/running
isTestSession = false;       % set true for test_ files

b = ble('YOUR_DEVICE_ADDRESS');
servUUID = '12345678-1234-5678-1234-56789ABCDEF0';
charUUID = '12345678-1234-5678-1234-56789ABCDEF1';
c = characteristic(b, servUUID, charUUID);

allData = '';
write(c, uint8('SYNC'));
pause(0.3);

fprintf('Receiving %s data...\n', activityName);
tStart = tic;
while toc(tStart) < 120
    chunk = char(read(c));
    if contains(chunk, '[EOF]')
        before = char(extractBefore(string(chunk), '[EOF]'));
        if ~isempty(strtrim(before))
            allData = [allData, before];
        end
        break;
    end
    if ~isempty(strtrim(chunk)) && ~ismember(strtrim(chunk), {'SYNC','NEXT'})
        allData = [allData, chunk];
    end
    write(c, uint8('NEXT'));
    pause(0.25);
end

% Save with activity label as filename
if isTestSession
    fname = sprintf('test_%s.csv', activityName);
else
    fname = sprintf('%s.csv', activityName);
end

fid = fopen(fname, 'w');
fprintf(fid, '%s', allData);
fclose(fid);
fprintf('Saved %d bytes to %s\n', length(allData), fname);

% Don't send DONE — keep the file on SD in case you need to re-pull
clear b c