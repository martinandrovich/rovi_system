% PLEASE LOAD THIS USING:
% run("path/to/rovi_system.m")

disp("Resolving directories for ROVI system...")

% define directories struct
DIR.SCRIPTS     = erase(strrep(which(mfilename),"\","/"), "/" + mfilename + ".m");
DIR.ROVI_SYSTEM = DIR.SCRIPTS + "/..";
DIR.EXPORT_FIG  = DIR.SCRIPTS + "/export_fig";
DIR.TESTS       = DIR.ROVI_SYSTEM + "/tests";

% configure export_fig
% https://github.com/altmany/export_fig
addpath(DIR.EXPORT_FIG);

% configure figure defaults
set(groot, "DefaultFigureRenderer", "painters");
set(groot, "DefaultFigurePosition", [0 0 500 500]);
set(groot, "DefaultFigureColor", [1 1 1]);
set(groot, "DefaultAxesFontSize", 14); % !!!

% colors struct
% https://colorhunt.co/palette/5441796166b332c1cd17d7a0
% https://colorhunt.co/palette/142f43ffab4cff5f7eb000b9
COLOR.GRAY = [200 200 200]/255;
COLOR.LIGHTGRAY = [220 220 220]/255;
COLOR.NAVY = [84 65 121]/255;
COLOR.BLUE = [46, 203, 255]/255;
% COLOR.CYAN = [0 207 255]/255; % OG
COLOR.TEAL = [89, 255, 244]/255;
COLOR.GREEN = [23 215 160]/255;
COLOR.ORANGE = [255, 171, 76]/255;
COLOR.RED = [255 95 126]/255;
COLOR.PURPLE = [176 0 185]/255;

% default MATLAB colors
% http://math.loyola.edu/~loberbro/matlab/html/colorsInMatlab.html
% MATLAB_COLORS = {
% 	[0.0000, 0.4470, 0.7410] ;
% 	[0.8500, 0.3250, 0.0980] ;
% 	[0.9290, 0.6940, 0.1250] ;
% 	[0.4940, 0.1840, 0.5560] ;
% 	[0.4660, 0.6740, 0.1880] ;
% 	[0.3010, 0.7450, 0.9330] ;
% 	[0.6350, 0.0780, 0.1840] ;
% };

% print resolved directories
disp("DIR.ROVI_SYSTEM: " + what(DIR.ROVI_SYSTEM).path)
disp("DIR.SCRIPTS: " + what(DIR.SCRIPTS).path)
disp("DIR.TESTS: " + what(DIR.TESTS).path)

% export local functions
get_experiment_dir = @get_experiment_dir_;
get_img_dir = @get_img_dir_;
get_experiment_data_dir = @get_experiment_data_dir_;

% local functions

function dir = get_experiment_dir_(experiment_name)
	dir_tests = evalin("base","DIR.TESTS");
	dir = dir_tests + "/" + experiment_name;
	dir = what(dir).path;
end

function dir = get_img_dir_(experiment_name)
	dir_tests = evalin("base","DIR.TESTS");
	dir = dir_tests + "/" + experiment_name + "/img";
	dir = what(dir).path;
end

function dir = get_experiment_data_dir_(experiment_name, timestamp)
	arguments
		experiment_name
		timestamp = ""
	end
	dir_tests = evalin("base","DIR.TESTS");
	dir = dir_tests + "/" + experiment_name + "/data/" + timestamp;
	dir = what(dir).path;
end

