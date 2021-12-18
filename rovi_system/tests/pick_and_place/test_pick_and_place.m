close all; clear; clc;
run("../../scripts/rovi_system.m");

% experiments (sorted by pick location)
EXPERIMENTS = ["20211216_164833", "20211216_171429", "20211216_173727"];

% deduce directories
DIR_DATA = get_experiment_data_dir("pick_and_place", EXPERIMENTS(1))
DIR_IMGS = get_img_dir("pick_and_place")

%% binary histograms

close all;
set(groot, "DefaultFigureRenderer", "painters");
figure("Position", [0 0 500 300])
t = tiledlayout(1,3);

for i = 1:numel(EXPERIMENTS)

	tile = nexttile;
	timestamp = EXPERIMENTS(i);
	DIR_DATA = get_experiment_data_dir("pick_and_place", timestamp);

	% extract pick_index from info.txt
	text = fileread(DIR_DATA + "/info.txt");
	[mat,tok] = regexp(text,"pick_index: (\d)",'match', 'tokens');
	pick_index = str2double(cell2mat(tok{:}));

	% histogram
	data = readmatrix(DIR_DATA + "/pick_and_place.csv");
	success = data(1:30,2);
	ratio = (sum(success(:))/numel(success))*100;
	histogram(success, "FaceColor", COLOR.MAP(i, :))
	xticks([0 1])
	xticklabels(["False", "True"])
	title("\fontsize{12}\rm Pick location " + pick_index, "\fontsize{10}\rm " + num2str(ratio,3) + "%")

end

ylabel(t, "Count");
xlabel(t, "Pick-and-place success");

export_fig(DIR_IMGS + "/pick-and-place-hist.pdf", "-painters")

%% diff_xy

close all;
set(groot, "DefaultFigureRenderer", "painters");
figure("Position", [0 0 500 300])
t = tiledlayout(1,3);

diff_xy_acum = [];
for i = 1:numel(EXPERIMENTS)

	tile = nexttile;
	timestamp = EXPERIMENTS(i);
	DIR_DATA = get_experiment_data_dir("pick_and_place", timestamp);

	% extract pick_index from info.txt
	text = fileread(DIR_DATA + "/info.txt");
	[mat,tok] = regexp(text,"pick_index: (\d)",'match', 'tokens');
	pick_index = str2double(cell2mat(tok{:}));

	% histogram
	data = readmatrix(DIR_DATA + "/pick_and_place.csv");
	diff_xy = data(1:30, 4);
	diff_xy_acum = [diff_xy_acum ; diff_xy];
	histogram(diff_xy, 10, "FaceColor", COLOR.MAP(i, :))
	
	title("\fontsize{12}\rm Pick location " + pick_index, "\fontsize{10}\rm\mu = " + num2str(mean(diff_xy),3) + " [m]")
end

ylabel(t, "Count");
xlabel(t, "Euclidean error in xy");

% accumulated mean and deviation
% figure
% histogram(diff_xy_acum)
mu = mean(diff_xy_acum)
sigma = std(diff_xy_acum)

export_fig(DIR_IMGS + "/pick-and-place-diffxy.pdf", "-painters")