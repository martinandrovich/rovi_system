close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
DIR_DATA = get_experiment_data_dir("planning_moveit", "20211202_140349")
DIR_IMGS = get_img_dir("planning_moveit")

% select method (RRT, RRTConnect, SBL, EST)
METHOD = "EST";

% experiments
EXPERIMENTS = ["20211202_140349", "20211202_140703", "20211202_141338"];

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
    plot3(traj(:, 4), traj(:, 8), traj(:, 12), "LineWidth", 1, "Color", [0 207/255 255/255 0.25])
end

% add start and end waypoints
plot3(traj(1, 4), traj(1, 8), traj(1, 12), "O", "MarkerFaceColor", COLOR.ORANGE)
plot3(traj(end, 4), traj(end, 8), traj(end, 12), "O", "MarkerFaceColor", COLOR.PURPLE)

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

for timestamp = EXPERIMENTS
    
    tile = nexttile;
    DIR_DATA = get_experiment_data_dir("planning_moveit", timestamp);
    
    % extract pick_index from info.txt
    text = fileread(DIR_DATA + "/info.txt");
    [mat,tok] = regexp(text,"pick_index: (\d)",'match', 'tokens');
    pick_index = str2double(cell2mat(tok{:}));
    
    % histogram
    plan = readmatrix(DIR_DATA + "/" + METHOD + "/plan.csv");
    data = rmoutliers(plan(:,2)); % remove outliers
    h = histfit(data);
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
    data = rmoutliers(plan(:,3)); % remove outliers
    h = histfit(data);
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
