close all; clear; clc;
run("../../scripts/rovi_system.m");

% experiments (sorted by pick location)
EXPERIMENTS = ["20211213_001315", "20211213_001542", "20211213_001858"];

% deduce directories
DIR_DATA = get_experiment_data_dir("planning_moveit", EXPERIMENTS(1))
DIR_IMGS = get_img_dir("planning_moveit")

% select method (PRM / SBL)
METHOD = "PRM";

%% trajectory plot

% for transparency
close all;
set(groot, "DefaultFigureRenderer", "opengl");

figure()

% add background image
img = imread(DIR_IMGS + "/traj-bg.png");
ax1 = axes();
imshow(img, "Parent", ax1);
ax2 = axes("Color", "none");
hold on

% trajectories
for i = 0:49
	traj = readmatrix(DIR_DATA + "/" + METHOD + "/traj" + i + ".csv");
	plot3(traj(:, 4), traj(:, 8), traj(:, 12), "LineWidth", 1, "Color", [COLOR.BLUE 0.25])
end

% add start and end waypoints
plot3(traj(1, 4), traj(1, 8), traj(1, 12), "O", "MarkerFaceColor", COLOR.GREEN, "Color", "white")
plot3(traj(end, 4), traj(end, 8), traj(end, 12), "O", "MarkerFaceColor", COLOR.RED, "Color", "white")

ax = gca;
ax.SortMethod = "childorder";

% orient view/zoom
% xl = xlim, yl = ylim, zl = zlim, [az,el] = view
view(105.7295,  0.6099)
xlim([-0.4351 0.8390])
ylim([-0.7504 1.8598])
zlim([-0.3908 1.7031])

f = gcf;
exportgraphics(f, DIR_IMGS + "/" + METHOD + "-traj.png")

%% histogram (planning time)

close all;
set(groot, "DefaultFigureRenderer", "painters");
figure("Position", [0 0 500 600])
t = tiledlayout(3,1);

data_acum = [];

for timestamp = EXPERIMENTS

	tile = nexttile;
	DIR_DATA = get_experiment_data_dir("planning_moveit", timestamp);

	% extract pick_index from info.txt
	text = fileread(DIR_DATA + "/info.txt");
	[mat,tok] = regexp(text,"pick_index: (\d)",'match', 'tokens');
	pick_index = str2double(cell2mat(tok{:}));

	% histogram
	plan = readmatrix(DIR_DATA + "/" + METHOD + "/plan.csv");
%     data = rmoutliers(plan(:,2), "mean"); % remove outliers
	data = plan(:,2);
%     data_acum = [data; data_acum];
	h = histfit(data, 10);
	pd = fitdist(data, "Normal");
	title("Pick location " + (pick_index + 1))
	h(1).FaceColor = COLOR.BLUE;
	h(2).Color = COLOR.RED;
	h(2).LineWidth = 3;
	annotation("textbox", "String", ["\mu = " + num2str(pd.mu, 3), "\sigma = " + num2str(pd.sigma, 3)], "Position", tile.Position, "Margin", 5, "LineStyle", "None", "HorizontalAlignment", "Right");

end

xlabel(t, "Planning time [ms]")
ylabel(t, "Count")

export_fig(DIR_IMGS + "/" + METHOD + "-planning-time.pdf")
%% histogram (trajectory duration)

close all;
set(groot, "DefaultFigureRenderer", "painters");
figure("Position", [0 0 500 600])
t = tiledlayout(3,1);

for timestamp = EXPERIMENTS

	tile = nexttile;
	DIR_DATA = get_experiment_data_dir("planning_moveit", timestamp);

	% extract pick_index from info.txt
	text = fileread(DIR_DATA + "/info.txt");
	[mat,tok] = regexp(text,"pick_index: (\d)",'match', 'tokens');
	pick_index = str2num(cell2mat(tok{:}));

	% histogram
	plan = readmatrix(DIR_DATA + "/" + METHOD + "/plan.csv");
%     data = rmoutliers(plan(:,3), "mean"); % remove outliers
	data = plan(:,3);
	h = histfit(data, 10);
	pd = fitdist(data, "Normal");
	title("Pick location " + (pick_index + 1))
	h(1).FaceColor = COLOR.BLUE;
	h(2).Color = COLOR.RED;
	h(2).LineWidth = 3;
	annotation("textbox", "String", ["\mu = " + num2str(pd.mu, 3), "\sigma = " + num2str(pd.sigma, 3)], "Position", tile.Position, "Margin", 5, "LineStyle", "None", "HorizontalAlignment", "Right");
	xlim([2 9]);

end

xlabel(t, "Trajectory duration [s]")
ylabel(t, "Count")

export_fig(DIR_IMGS + "/" + METHOD + "-traj-dur.pdf")

%% box plots

close all;

plan_time_prm = [];
traj_dur_prm = [];
plan_time_sbl = [];
traj_dur_sbl = [];

% accumulate data
for timestamp = EXPERIMENTS

	DIR_DATA = get_experiment_data_dir("planning_moveit", timestamp);
	plan_prm = readmatrix(DIR_DATA + "/PRM/plan.csv");
	plan_sbl = readmatrix(DIR_DATA + "/SBL/plan.csv");

	plan_time_prm = [plan_time_prm ; plan_prm(:, 2)];
	plan_time_sbl = [plan_time_sbl ; plan_sbl(:, 2)];

	traj_dur_prm = [traj_dur_prm ; plan_prm(:, 3)];
	traj_dur_sbl = [traj_dur_sbl ; plan_sbl(:, 3)];

end

% analyze data
pd_plan_time_prm = fitdist(plan_time_prm, "Normal")
pd_traj_dur_prm = fitdist(traj_dur_prm, "Normal")
pd_plan_time_sbl = fitdist(plan_time_sbl, "Normal")
pd_traj_dur_sbl = fitdist(traj_dur_sbl, "Normal")

% box plots
close all;
SIZE = [0 0 500 300];

% planning time
figure("Position", SIZE)
plan_time = [plan_time_prm plan_time_sbl];
plan_time = rmoutliers(plan_time, "mean");
boxplot(plan_time, ["PRM", "SBL"])
ylabel("Planning time [ms]")
% xlabel("Planner")

export_fig(DIR_IMGS + "/PRM-SBL-boxplot-plan-time.pdf")

% trajectory duration
figure("Position", SIZE)
traj_dur = [traj_dur_prm traj_dur_sbl];
traj_dur = rmoutliers(traj_dur, "mean");
boxplot(traj_dur, ["PRM", "SBL"])
ylabel("Trajectory duration [s]")
% xlabel("Planner")

export_fig(DIR_IMGS + "/PRM-SBL-boxplot-traj-dur.pdf")