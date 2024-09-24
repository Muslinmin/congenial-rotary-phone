%% Initialization
delete(instrfind);
% MATLAB script to simulate the creation of a point cloud

%Ultrasound sensor
% Establish a connection to the Arduino
a = arduino('COM5', 'Leonardo', 'Libraries', {'Ultrasonic', 'Servo'});
% Define pins for the ultrasonic sensor
trigPin = 'A2';  % Trigger pin on Digital Pin 20
echoPin = 'A3';  % Echo pin on Digital Pin 21
warningLedPin = 'D7'; % RED LED to warn the user
ultrasonicObj = ultrasonic(a, trigPin, echoPin);
servoX = servo(a, 'D6', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
%servoY = servo(a, 'D9', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
%% Polar plot for live updating
% Create the polar plot for live updating
h = polarplot(0,0);
title('Map of the Environment');
thetalim([30 150]);
grid on;
obstacleThreshold = 40;

%% Main program
% Loop indefinitely
while true
    i = 30;
    table = zeros(120,2);
    % Sweep from 30 to 150 degrees
    for theta = 30/180 : 1/180 : 150/180
        writePosition(servoX, theta);
        dist1 = ultrasonicObj.readDistance() * 100;
        pause(.04);
        dist2 = ultrasonicObj.readDistance() * 100;
        dist = (dist1 + dist2) / 2;
        table(i, 1) = (i-1);         % Angle in degrees
        table(i, 2) = round(dist, 2); % Distance in cm
        
        % Update the polar plot as the servo moves
        set(h, 'ThetaData', table(1:i, 1)*pi/180, 'RData', table(1:i, 2));
        drawnow;  % Update the figure in real-time
        
        % Check if the obstacle is within the threshold distance
        if dist < obstacleThreshold
            fprintf('Warning: Obstacle detected at %.2f cm at angle %.0f degrees\n', dist, theta * 180);
            % writeDigitalPin(a, warningLedPin, 1);  % Turn LED on
        else
            % writeDigitalPin(a, warningLedPin, 0);  % Turn LED  Off
        end
        
        i = i + 1;
    end
    
    % Sweep back from 180 to 0 degrees
    j = 1;
    for theta = 150/180 : -1/180 : 30/180
        writePosition(servoX, theta);
        dist1 = ultrasonicObj.readDistance() * 100;
        pause(.04);
        dist2 = ultrasonicObj.readDistance() * 100;
        dist = (dist1 + dist2) / 2;
        table(i-j, 2) = (table(i-j, 2) + round(dist, 2)) / 2; % Averaging
        
        % Update the entire plot to replace readings without removing lines
        set(h, 'ThetaData', table(:, 1)*pi/180, 'RData', table(:, 2));
        drawnow;  % Real-time plot update
        
        % Check if the obstacle is within the threshold distance
        if dist < obstacleThreshold
            fprintf('Warning: Obstacle detected at %.2f cm at angle %.0f degrees\n', dist, theta * 180);
            % writeDigitalPin(a, warningLedPin, 1);  % Turn LED on
        else
            % writeDigitalPin(a, warningLedPin, 0);  % Turn LED Off
        end
        j = j + 1;
    end
end
