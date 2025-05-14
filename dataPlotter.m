clear; clc; close all;

% Options:
saveFile = 0; % Enter 1 to save to csv, 0 for off

% Constants:
NUMPOINTS = 200;

% Find the Arduino serial port
% If this breaks, open device manager and find the name of the device
% under ports, then replace dev_name with the name shown
dev_name = 'Arduino Uno';
[~,res]=system('wmic path Win32_SerialPort');
ind = strfind(res,dev_name);

if (~isempty(ind))
    % If arduino found, save the name and print
matchLine = extractBetween(res, ind(1), newline);
tokens = regexp(matchLine{1}, 'COM\d+', 'match');
port_name = tokens{1};

    fprintf('Arduino is on %s\n',port_name);
    % Attempt to open the port
    try
        % Connect to the port with a 9600 baud rate
        sro = serialport(port_name,9600);
        fprintf('%s opened\n',port_name);
    catch err
        % If connection fails, print why and exit
        fprintf('%s\n%s\n',err.identifier,err.message);
        return
    end
else
    fprintf('COM-port not found\n');
    return
end


% Flush the serial port
flush(sro)

% Set up tiled layout
window = figure;
x = tiledlayout(2,1);

% Create position tile
% input position line
pos_input_ax = nexttile;
pos_input = animatedline('Marker','o','MarkerSize',3,'Color','g','MaximumNumPoints',NUMPOINTS+1);

% Create actual position line
pos_act = animatedline('Marker','o','MarkerSize',3,'Color','b', 'MaximumNumPoints',NUMPOINTS+1);
xlim([0,NUMPOINTS])
ylim([-2,6])
ylabel("Position(cm)")
legend('Input', 'Actual')

% Create pressure tile
pressure_ax = nexttile;
pressure = animatedline('Marker','o','MarkerSize',3,'MaximumNumPoints',NUMPOINTS+1);
xlim([0,NUMPOINTS])
ylim([-100, 300])
ylabel("Pressure (kPa)")

% Start counter of the number of readings
count = 0;

% CSV Data saving
if saveFile == 1
    filename = "VentriclePump_"+string(datetime("now","Format","uuuuMMdd_HH_mm_ss"))+".csv"
    writematrix(["Count" "Matlab Time (s)" "Arduino Time (ms)" "Input Position (cm)" "Actual Position (cm)" "Pressure (kPa)"],filename)
end

start = tic;


% Begin readings
while ishandle(window) % Run this loop until user closes window
    % Read data from Arduino
    data = str2num(readline(sro));
    count = count+1;

    % Add data to filef
    if saveFile == 1
        time = string(datetime("now","Format","HH:mm:ss.SSS"));
        writematrix([count time data],filename,'WriteMode','append');
    end

    % Add points to graph
    addpoints(pos_input,count,data(2));
    addpoints(pos_act,count,data(3));
    addpoints(pressure,count,data(4));

    % Update limits
    xlim(pos_input_ax,[max(count-NUMPOINTS,0) max(count,NUMPOINTS)])
    xlim(pressure_ax,[max(count-NUMPOINTS,0) max(count,NUMPOINTS)])
    
    % Draw the plot
    drawnow;
end




