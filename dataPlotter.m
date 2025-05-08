x = tiledlayout(2,1);

% Create position tile
% input position line
pos_input_ax = nexttile;
pos_input = animatedline('Marker','o','MarkerSize',3,'Color','g','MaximumNumPoints',NUMPOINTS+1);

% Create actual position line
pos_act = animatedline('Marker','o','MarkerSize',3,'Color','b', 'MaximumNumPoints',NUMPOINTS+1);
xlim([0,NUMPOINTS])
ylim([0,3])
ylabel("Position(cm)")
legend('Input', 'Actual')

% Create pressure tile
pressure_ax = nexttile;
pressure = animatedline('Marker','o','MarkerSize',3,'MaximumNumPoints',NUMPOINTS+1);
xlim([0,NUMPOINTS])
ylim([0, 150])
ylabel("Pressure (kPa")

% Start counter of the number of readings
count = 0;

% CSV Data saving
if saveFile == 1
    filename = "VentriclePump_"+string(datetime("now","Format","uuuuMMdd_HH_mm_ss"))+".csv"
    writematrix(["Count" "Time" " " "Input Position" "Actual Position" "Pressure"],filename)
end

start = tic;


% Begin readings
while ishandle(window) % Run this loop until user closes window
    % Read data from Arduino
    data = str2num(readline(sro));
    count = count+1;

    % Add data to file
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


