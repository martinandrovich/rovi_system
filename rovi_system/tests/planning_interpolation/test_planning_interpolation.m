close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
DIR_DATA = get_experiment_data_dir("planning_interpolation", "20211215_162448")
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
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COLOR.RED, "Color", COLOR.RED, "DisplayName", "Waypoints")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
ax = gca;
ax.SortMethod = "childorder";
view(150, 25);

export_fig(DIR_IMGS + "/planning-interpolation-traj-lin.pdf", "-painters")

%% parabolic interpolation

data = traj_par;
x = data(:, 4); y = data(:, 8); z = data(:, 12);

figure
plot3(x, y, z, "LineWidth", 2, "Color", COLOR.BLUE, "DisplayName", "Path")
hold on
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COLOR.RED, "Color", COLOR.RED, "DisplayName", "Waypoints")
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
ax = gca;
ax.SortMethod = "childorder";
view(150, 25);

export_fig(DIR_IMGS + "/planning-interpolation-traj-par.pdf", "-painters")

%% histograms (both)

figure("Position", [0 0 800 500])
t = tiledlayout(1, 2);

% linear interpolation
tile = nexttile;
data = rmoutliers(plan_lin(:, 2));
% data = plan_lin(:, 2);
h = histfit(data);
pd = fitdist(data, "Normal");
h(1).FaceColor = COLOR.BLUE;
h(2).Color = COLOR.RED;
h(2).LineWidth = 3;
title("Linear interpolation")
xlabel("Time [ms]")
ylabel("Count")
pbaspect([1 0.7 1])
annotation("textbox", "String", ["\mu = " + num2str(pd.mu, 3), "\sigma = " + num2str(pd.sigma, 3)], "Position", tile.Position, "Margin", 5, "LineStyle", "None", "HorizontalAlignment", "Right");

ax = gca;
ax.YTickLabel = ax.YTick * 3;

% parabolic interpolation
tile = nexttile;
data = rmoutliers(plan_par(:, 2));
h = histfit(data);
pd = fitdist(data, "Normal");
h(1).FaceColor = COLOR.BLUE;
h(2).Color = COLOR.RED;
h(2).LineWidth = 3;
title("Parabolic interpolation")
xlabel("Time [ms]")
ylabel("Count")
pbaspect([1 0.7 1])
annotation("textbox", "String", ["\mu = " + num2str(pd.mu, 3), "\sigma = " + num2str(pd.sigma, 3)], "Position", tile.Position, "Margin", 5, "LineStyle", "None", "HorizontalAlignment", "Right");

ax = gca;
ax.YTickLabel = ax.YTick * 3;

export_fig(DIR_IMGS + "/planning-interpolation-plan-time.pdf", "-painters")