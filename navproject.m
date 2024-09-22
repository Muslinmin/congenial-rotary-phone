% Set up serial communication
% serialPort = 'COM3';  % Arduino serial port
% baudRate = 9600;
% s = serial(serialPort, 'BaudRate', baudRate);
% fopen(s);  % Open serial connection

% Parameters for the radar plot
theta = [];  % Array to store angles
r = [];  % Array to store distances
maxDistance = 200;  % Maximum range of sensor
fadeTime = 50;  % Number of steps before a point fades

% Initialize figure
figure;
hold on;
axis equal;
xlim([-maxDistance maxDistance]);
ylim([-maxDistance maxDistance]);
title('Radar Visualization');
xlabel('X (cm)');
ylabel('Y (cm)');

% Infinite loop to keep reading data from the Arduino
while true
    if s.BytesAvailable > 0
        % Read data from the serial port
        data = fscanf(s, '%s');
        
        % Split the data into angle and distance
        values = strsplit(data, ',');
        if length(values) == 2
            angle = str2double(values{1});
            distance = str2double(values{2});
            
            % Store the angle and distance in arrays
            theta(end+1) = deg2rad(angle);  % Convert angle to radians
            r(end+1) = distance;
            
            % Polar to Cartesian conversion
            x = r .* cos(theta);
            y = r .* sin(theta);
            
            % Plot the points (X, Y)
            plot(x, y, 'go', 'MarkerSize', 4);  % Plot detected points as green circles
            drawnow;  % Update the plot in real time
            
            % Fade older points
            if length(theta) > fadeTime
                theta = theta(2:end);  % Remove oldest angle
                r = r(2:end);  % Remove oldest distance
            end
        end
    end
end

% Close the serial connection when finished
% fclose(s);
% delete(s);
% clear s;

%%
% Parameters for the radar plot
% For testing
theta = [];  
r = [];  
maxDistance = 300;  
fadeTime = 50;  

% Create polar axes
pax = polaraxes;  
title(pax, 'Map of the Environment');  

% Set the radial limits (distance limits) and angle limits for the radar
rlim([0 maxDistance]);  % Set the radial limit for the distance
thetalim([0 180]);  % Limit the angle to 0-180 degrees (half-circle radar)

% Simulate data without Arduino
while true
    % Simulate random angle and distance values
    angle = randi([0, 180]);  % Random angle between 0 and 180 degrees
    distance = randi([1, maxDistance]);  % Random distance between 1 cm and maxDistance

    % Convert angle to radians and store angle and distance
    theta(end+1) = deg2rad(angle);  % Convert angle to radians
    r(end+1) = distance;
    
    % Plot the points on a traditional radar using polarplot
    polarplot(pax, theta, r, 'b.');  % Polar plot with blue dots for points
    drawnow;  % Update the plot in real time
    
    % Fade older points
    if length(theta) > fadeTime
        theta = theta(2:end);  % Remove oldest angle
        r = r(2:end);  % Remove oldest distance
    end
    
    pause(0.1);  % Small pause for a real-time effect (simulates radar scanning speed)
end

%%
% A different radar layout
% Set up serial communication
delete(instrfind);  % Clear existing serial connections
s = serial('COM3', 'BaudRate', 9600);  % Update COM3 to your Arduino port
fopen(s);

% Create polar axes
figure;
pax = polaraxes;

while true
    if s.BytesAvailable > 0
        data = fscanf(s, '%s');
        values = strsplit(data, ',');
        if length(values) == 2
            angle = str2double(values{1});
            distance = str2double(values{2});
            % Convert angle to radians for polar plot
            theta = deg2rad(angle);
            % Update the polar plot
            polarplot(pax, theta, distance, 'b.');
            rlim(pax, [0 300]);  % Set the maximum radial limit to 300 cm
            thetalim(pax, [0 180]);  % Set the angle limit from 0 to 180 degrees
            drawnow;
        end
    end
end

% Close the serial port when done
fclose(s);
delete(s);
clear s;

