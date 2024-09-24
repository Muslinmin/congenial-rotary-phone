%%
% Point cloud script, : as delimiter

% Close existing serial port connections
delete(instrfind);

% Set up serial communication
serialPort = 'COM5'; 
baudRate = 9600;
s = serialport(serialPort, baudRate);  % Open serial port

% Variables to store point cloud data
x = [];
y = [];
z = [];

% Infinite loop to read data and build the point cloud
disp('Reading data from Arduino...');
while true
    if s.NumBytesAvailable > 0
        % Read the line of data from Arduino
        data = readline(s);
        
        % Split the data using the colon as the delimiter
        parsedData = strsplit(data, ':');
        
        if length(parsedData) == 2
            % Extract the pan angle, tilt angle, and hypotenuse (distance)
            values = strsplit(parsedData{1}, ',');
            if length(values) == 3
                xAxisDeg = str2double(values{1});  % Pan angle
                zAxisDeg = str2double(values{2});  % Tilt angle
                hypotenuse = str2double(values{3});  % Distance
                
                % Convert angles from degrees to radians
                panRad = deg2rad(xAxisDeg);
                tiltRad = deg2rad(zAxisDeg);
                
                % Convert spherical coordinates to Cartesian coordinates
                % Assuming pan is the azimuth and tilt is the elevation
                x(end+1) = hypotenuse * cos(tiltRad) * cos(panRad);  % X-coordinate
                y(end+1) = hypotenuse * cos(tiltRad) * sin(panRad);  % Y-coordinate
                z(end+1) = hypotenuse * sin(tiltRad);  % Z-coordinate

                % Plot the point cloud dynamically
                scatter3(x, y, z, 'filled');  % 3D scatter plot of the point cloud
                xlabel('X (cm)');
                ylabel('Y (cm)');
                zlabel('Z (cm)');
                title('3D Point Cloud from Ultrasonic Sensor');
                grid on;
                axis equal;
                drawnow;
            end
        end
    end
end
