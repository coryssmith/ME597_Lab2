clc; clear; close all;

% Run the plots
plotOdometry("odom_content_circle.csv", "Circle", 'r');
plotOdometry("odom_content_line.csv", "Line", 'g');
plotOdometry("odom_content_spiral.csv", "Spiral", 'b');
plotIMU("imu_content_circle.csv", "Circle");
plotIMU("imu_content_line.csv", "Line");
plotIMU("imu_content_spiral.csv", "Spiral");

% Load CSV data and manually assign column names
function data = loadData(file, colNames)
    data = readtable(file, 'ReadVariableNames', false); % Read without assuming headers
    data.Properties.VariableNames = colNames; % Assign correct column names
end

% Plot odometry data
function plotOdometry(file, label, color)
    data = loadData(file, {'x', 'y', 'th', 't'});
    figure;
    subplot(1, 2, 1);
    plot(data.x, data.y, color);
    title(["Odometry: X vs. Y - " label]);
    xlabel("X Position (m)");
    ylabel("Y Position (m)");
    grid on;
    subplot(1, 2, 2);
    plot(data.t, data.x, 'b', data.t, data.y, 'r', data.t, data.th, 'k');
    title(["Odometry: X, Y, Theta vs. Time - " label]);
    xlabel("Time (s)");
    ylabel("Values");
    legend("X", "Y", "Theta");
    grid on;
end

% Plot IMU data 
function plotIMU(file, label)
    data = loadData(file, {'a_x', 'a_y', 'w_z', 't'});
    figure;
    plot(data.t, data.a_x, 'b', data.t, data.a_y, 'r', data.t, data.w_z, 'k');
    title(["IMU: Acceleration and Angular Velocity - " label]);
    xlabel("Time (s)");
    ylabel("Values");
    legend("A_x [m/s^2]", "A_y [m/s^2]", "W_z [rad/s]");
    grid on;
end
