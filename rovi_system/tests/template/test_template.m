close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
% DIR_EXP  = get_experiment_dir("template");
DIR_DATA = get_experiment_data_dir("template", "20211119_124628")
DIR_IMGS = get_img_dir("template")

% load data
data = readmatrix(DIR_DATA + "/data.csv");

% plot
plot(data(:,1), data(:,2), "LineWidth", 3, "Color", COLOR.BLUE)
title("test\_template plot");
xlabel("x [s]");
ylabel("y [m]");

% export
export_fig(DIR_IMGS + "/template.pdf", "-painters")