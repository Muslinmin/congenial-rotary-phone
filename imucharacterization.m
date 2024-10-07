%% Sample Routine - DOES NOT WORK/ A SCAM
% Create the serial object (board)
board = serialport("COM5", 9600);  % Adjust the COM port and baud rate

% Call the getData function
data = getData(board);

% Update the stripchart for data
figure(1); 
StripChart('Update',data(1:end)); title('Channel 1'); drawnow;
title('Channel 1');
drawnow;

%% With the Quiver to visualize the orientation of IMU
% Create the serial object (board)
board = serialport("COM5", 9600);  % Adjust the COM port and baud rate

% Set up the figure for the 3D plot
figure;
axis([-1 1 -1 1 -1 1]);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

% Initialize quiver objects for X, Y, Z axes
hx = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2); % X-axis (red)
hy = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2); % Y-axis (green)
hz = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2); % Z-axis (blue)

% Loop to update arrows based on IMU data
while true
    % Get data from the IMU (pitch, roll, yaw)
    imuData = getData(board);  % Modify getData to return pitch, roll, yaw
    
    % Assuming getData returns angles in degrees: [pitch, roll, yaw]
    pitch = imuData(7);
    roll = imuData(8);
    yaw = 0; % Yaw is fixed as per your requirement
    
    % Convert pitch and roll to radians
    pitch = deg2rad(pitch);
    roll = deg2rad(roll);
    
    % Create rotation matrices for pitch and roll
    R_pitch = [1, 0, 0; 0, cos(pitch), -sin(pitch); 0, sin(pitch), cos(pitch)];
    R_roll = [cos(roll), 0, sin(roll); 0, 1, 0; -sin(roll), 0, cos(roll)];
    
    % Apply rotations to the base vectors
    Rx = R_roll * R_pitch * [1; 0; 0];
    Ry = R_roll * R_pitch * [0; 1; 0];
    Rz = R_roll * R_pitch * [0; 0; 1];
    
    % Update quiver objects to show the new orientation
    set(hx, 'UData', Rx(1), 'VData', Rx(2), 'WData', Rx(3));
    set(hy, 'UData', Ry(1), 'VData', Ry(2), 'WData', Ry(3));
    set(hz, 'UData', Rz(1), 'VData', Rz(2), 'WData', Rz(3));
    
    drawnow;
end

% Flush the serial buffer before starting
flush(board);
