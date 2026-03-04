% link all function folders to the same path to allow calls between folders
% must be called before running code
repo_base_dir = fileparts(mfilename('fullpath'));
addpath(genpath(repo_base_dir));
disp('Repository linked successfully.');