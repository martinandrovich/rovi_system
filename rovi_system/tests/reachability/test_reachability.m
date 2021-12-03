close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
GRASP_POS = "side"; % "top" or "side"
DIR_DATA = get_experiment_data_dir("reachability", "20211119_162442")
DIR_IMGS = get_img_dir("reachability")

% load data
data = readmatrix(DIR_DATA + "/" + GRASP_POS + ".csv");

increment = 0.1;
x = [min(data(:, 2)) max(data(:, 2))];
y = [min(data(:, 1)) max(data(:, 1))];

C = [];
for i = 1:size(data)
	row = data(i, 1) * (1/increment) + 1;
	col = data(i, 2) * (1/increment) + 1;
	C(row, col) = data(i, 3);
end

% plot
figure()
set(gcf, "Position", [0 0 500 250]);
imagesc(x, y, C)
colorbar
set(gca,"XAxisLocation","top","YAxisLocation","left","ydir","reverse");
axis image;
xlabel("Table width (y)")
ylabel("Table height (x)")
ytickformat("%.1f");
xtickformat("%.1f");

hold on
plot(1, 0.4,'or', "MarkerFaceColor", "Red", "MarkerSize", 10)
text(1, 0.48, ["Object"], "Color", "White", "FontSize", 10, "HorizontalAlignment", "center")

% export
export_fig(DIR_IMGS + "/reachability-" + GRASP_POS + ".pdf", "-painters")