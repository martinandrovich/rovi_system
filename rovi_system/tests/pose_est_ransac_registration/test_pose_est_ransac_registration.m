close all; clear; clc;
run("../../scripts/rovi_system.m");
format short

DIR_IMGS = get_img_dir("pose_est_ransac_registration")

% load data
load("m2_data.mat");
X = experimentm2;
noise = [0, 8e-3, 12e-3, 16e-3];

%% translational error

set(groot, "DefaultFigureRenderer", "painters");
f = figure("Position", [0 0 500 500]);

for i = 1:size(noise, 2)
    X_it = X(X(:, 2) == noise(i), :);
    time = [X_it(:, 3)];
    pos_est = [X_it(:, 4:6)];
    pos_des = [X_it(:, 11:13)];
    pos_error = pos_des - pos_est;
    subplot(2, 2, i);
    scatter(pos_error(:, 1)*1000, pos_error(:, 2)*1000, ".", "MarkerEdgeColor", COLOR.BLUE, "MarkerEdgeAlpha", 0.4);
    pbaspect([1 1 1]);
    title('\rm\sigma = ' + string(noise(i)*1000) + " [mm]");
    xlim([-50 50]);
    ylim([-50 50]);
    xlabel('x [mm]');
    ylabel('y [mm]');
    circle(0, 0, 30);
    xticks([-40 -20 0 20 40])
    yticks([-40 -20 0 20 40])
    xticklabels({'-40', '-20', '0', '20', '40'})
    yticklabels({'-40', '-20', '0', '20', '40'})
end

exportgraphics(f, DIR_IMGS + "/pose-est-m2-xy-error.pdf")

function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, "LineWidth", 2, "Color", [255 95 126]/255);
end
