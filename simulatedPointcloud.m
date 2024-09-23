%% point cloud
% MATLAB script to simulate the creation of a point cloud

% Parameters for the simulation
panMin = 0;       % Minimum pan angle (degrees)
panMax = 180;     % Maximum pan angle (degrees)
tiltMin = 0;      % Minimum tilt angle (degrees)
tiltMax = 90;     % Maximum tilt angle (degrees)
panStep = 5;      % Step size for pan angle (degrees)
tiltStep = 5;     % Step size for tilt angle (degrees)
maxDistance = 100;  % Maximum distance (cm)

% Create arrays to store the point cloud data
x = [];
y = [];
z = [];

% Simulate the creation of the point cloud
figure;
hold on;
grid on;
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Simulated 3D Point Cloud');
axis equal;
view(3);

% Loop through tilt angles (elevation)
for tiltAngle = tiltMin:tiltStep:tiltMax
    % Loop through pan angles (azimuth)
    for panAngle = panMin:panStep:panMax
        % Simulate the distance for this pan and tilt angle
        % For simulation purposes, we'll generate random distances
        % In reality, this would come from the ultrasonic sensor
        distance = maxDistance * (0.8 + 0.2 * rand(1));  % Random distance (80% to 100% of max distance)
        
        % Convert angles from degrees to radians
        panRad = deg2rad(panAngle);
        tiltRad = deg2rad(tiltAngle);
        
        % Convert spherical coordinates to Cartesian coordinates
        x_new = distance * cos(tiltRad) * cos(panRad);  % X-coordinate
        y_new = distance * cos(tiltRad) * sin(panRad);  % Y-coordinate
        z_new = distance * sin(tiltRad);                % Z-coordinate
        
        % Append the new point to the point cloud data
        x = [x, x_new];
        y = [y, y_new];
        z = [z, z_new];
        
        % Plot the current point in the 3D space
        scatter3(x_new, y_new, z_new, 'filled', 'MarkerFaceColor', 'r');
        
        % Update the plot
        drawnow;
    end
end

% Final plot of the entire point cloud
scatter3(x, y, z, 'filled', 'MarkerFaceColor', 'r');
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Final Simulated 3D Point Cloud');
axis equal;
grid on;
view(3);
