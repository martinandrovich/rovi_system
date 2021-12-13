close all; clear; clc;
run("../../scripts/rovi_system.m");

% deduce directories
GRASP_POS = "side"; % "top" or "side"
DIR_DATA = get_experiment_data_dir("test_pose_estimation_grasp")
DIR_IMGS = get_img_dir("test_pose_estimation_grasp")