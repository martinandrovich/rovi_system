close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
DIR_DATA = get_experiment_data_dir("planning_interpolation", "example_waypoints")
DIR_IMGS = get_img_dir("planning_interpolation")

% load data
traj_lin = readmatrix(DIR_DATA + "/traj_lin.csv");
traj_par = readmatrix(DIR_DATA + "/traj_par.csv");
traj_dur = readmatrix(DIR_DATA + "/dur.txt");
waypoints = readmatrix(DIR_DATA + "/waypoints.csv");

% example color-order
% N=6;
% X = linspace(0,pi*3,1000);
% Y = bsxfun(@(x,n)sin(x+2*n*pi/N), X.', 1:N);
% plot(X,Y,'linewidth',5)
% colororder(COLOR.MAP)
% ylim([-1.1 1.1]);

%% linear interpolation

close all; clc;

% compute variables
data = traj_lin;

dur = traj_dur(1);
dt = 0.01;
t = 0:dt:dur;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

xdot = gradient(x(:)) ./ gradient(t(:));
ydot = gradient(y(:)) ./ gradient(t(:));

% plot
figure
colororder(COLOR.MAP);
plot(t, x, "LineWidth", 3)
hold on
plot(t, y, "LineWidth", 3)
xline([9, 20], "--", "Color", COLOR.GRAY)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/example-traj-lin-pos.pdf")

figure
colororder(COLOR.MAP);
plot(t, xdot, "LineWidth", 3)
hold on
plot(t, ydot, "LineWidth", 3)
xline([9, 20], "--", "Color", COLOR.GRAY)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/example-traj-lin-vel.pdf")

figure
colororder(COLOR.MAP);
plot3(x, y, z, "LineWidth", 3)
hold on
plot3(waypoints(1, 4), waypoints(1, 8), waypoints(1, 12), "O", "MarkerFaceColor", COLOR.GREEN, "Color", "white")
plot3(waypoints(2, 4), waypoints(2, 8), waypoints(1, 12), "O", "MarkerFaceColor", COLOR.GRAY, "Color", "white")
plot3(waypoints(3, 4), waypoints(3, 8), waypoints(3, 12), "O", "MarkerFaceColor", COLOR.GRAY, "Color", "white")
plot3(waypoints(4, 4), waypoints(4, 8), waypoints(4, 12), "O", "MarkerFaceColor", COLOR.RED, "Color", "white")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
% view(-30, 15);
view(-37.5, 30);
legend(["", "Initial", "", "", "Final"], "Location", "northeast")

export_fig(DIR_IMGS + "/example-traj-lin.pdf")

%% parabolic interpolation

close all; clc;

% compute variables
data = traj_par;

dur = traj_dur(2);
dt = 0.01;
t = 0:dt:dur;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

xdot = gradient(x(:)) ./ gradient(t(:));
ydot = gradient(y(:)) ./ gradient(t(:));

% plot
figure
colororder(COLOR.MAP);
plot(t, x, "LineWidth", 3)
hold on
plot(t, y, "LineWidth", 3)
xline([6.7, 13.8], "--", "Color", COLOR.GRAY)
pbaspect([1 1 1])
legend(["x", "y"]);
xlim([0 Inf])
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/example-traj-par-pos.pdf")

figure
colororder(COLOR.MAP);
plot(t, xdot, "LineWidth", 3)
hold on
plot(t, ydot, "LineWidth", 3)
xline([6.7, 13.8], "--", "Color", COLOR.GRAY)
pbaspect([1 1 1])
xlim([0 Inf])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/example-traj-par-vel.pdf")

figure
colororder(COLOR.MAP);
plot3(x, y, z, "LineWidth", 3)
hold on
plot3(waypoints(1, 4), waypoints(1, 8), waypoints(1, 12), "O", "MarkerFaceColor", COLOR.GREEN, "Color", "white")
plot3(waypoints(2, 4), waypoints(2, 8), waypoints(1, 12), "O", "MarkerFaceColor", COLOR.GRAY, "Color", "white")
plot3(waypoints(3, 4), waypoints(3, 8), waypoints(3, 12), "O", "MarkerFaceColor", COLOR.GRAY, "Color", "white")
plot3(waypoints(4, 4), waypoints(4, 8), waypoints(4, 12), "O", "MarkerFaceColor", COLOR.RED, "Color", "white")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(-37.5, 30);
legend(["", "Initial", "", "", "Final"], "Location", "northeast")

export_fig(DIR_IMGS + "/example-traj-par.pdf")