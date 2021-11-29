close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
GRASP_POS = "top"; % "top" or "side"
DIR_DATA = get_experiment_data_dir("planning_interpolation", "20211128_163809")
DIR_IMGS = get_img_dir("planning_interpolation")

% load data
traj_lin = readmatrix(DIR_DATA + "/traj_lin.csv");
traj_par = readmatrix(DIR_DATA + "/traj_par.csv");
traj_pts = readmatrix(DIR_DATA + "/waypoints.csv");
plan_lin = readmatrix(DIR_DATA + "/plan_lin.csv");
plan_par = readmatrix(DIR_DATA + "/plan_par.csv");

%% linear interpolation

data = traj_lin;
x = data(:, 4); y = data(:, 8); z = data(:, 12);

figure
plot3(x, y, z, "LineWidth", 2, "Color", COLOR.BLUE, "DisplayName", "Path")
hold on
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COLOR.ORANGE, "DisplayName", "Waypoints")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
ax = gca;
ax.SortMethod = "childorder";
view(150, 25);

export_fig(DIR_IMGS + "/traj-lin.pdf", "-painters")

%% parabolic interpolation

data = traj_par;
x = data(:, 4); y = data(:, 8); z = data(:, 12);

figure
plot3(x, y, z, "LineWidth", 2, "Color", COLOR.BLUE, "DisplayName", "Path")
hold on
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COLOR.ORANGE, "DisplayName", "Waypoints")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
ax = gca;
ax.SortMethod = "childorder";
view(150, 25);

export_fig(DIR_IMGS + "/traj-par.pdf", "-painters")

%% histograms (both)

figure("Position", [0 0 700 500])

subplot(1,2,1)
histogram(plan_lin(:, 2), 40)
title("Linear interpolation")
mean(plan_lin(:, 2))
xlabel("Time [ms]")
ylabel("Count")
xlim([0 0.03])
pbaspect([1 0.7 1])
% yticklabels(yticks*100)

subplot(1,2,2)
histogram(plan_par(:, 2), 20)
title("Parabolic interpolation")
mean(plan_lin(:, 2))
xlabel("Time [ms]")
ylabel("Count")
xlim([0 0.03])
pbaspect([1 0.7 1])
% yticklabels(yticks*100)

export_fig(DIR_IMGS + "/plan-time.pdf", "-painters")