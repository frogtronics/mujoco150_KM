% Script to process Kassina walking data (from the 2019 publication).  
% Use it to locate relevant trial xyz points, convert xyz points (12 markers) to
% quaternions, run Kassina_TL_MA.exe (MJ) from matlab, then import the
% resulting moment arms and tendon lengths, then export into text files.
% Also calls KassinaInputs12.m and transformFrogData12.m, in my case these are in
% 'C:\Users\admin\Documents\Mujoco\mjpro150\scripts\', as well as some
% quaternion functions found in \scripts\quaternions.
% Need walking_metadata.csv in the root folder.

% Chris Basu
fixed = false; % is pelvic rotation fixed in the y axis (i.e. no lateral motion)?

addpath 'quaternions';


folder = uigetdir(matlabroot,'Data location');
files = dir(fullfile(folder,'*.dat'));
metadata= readtable('walking_metadata.csv');

numtrials = size(metadata,1);

for i=1:numtrials;
    filename = files(i).name;
    trialname = files(i).name(1:11);
    animal = trialname(1:4);
    trial = trialname(10:11);
    trial_metadata = metadata(metadata.AnimalFileNameForExporting == categorical(cellstr((trialname))),:);
    trial_start = trial_metadata.StartOfLimbCycle___;
    trial_end = trial_metadata.EndOfLimbCycle___;
    if fixed
        saveas = ['C:\Users\admin\Documents\Mujoco\mjpro150\input\kassinawalks\' trialname '_fixed.txt'];
    else
        saveas = ['C:\Users\admin\Documents\Mujoco\mjpro150\input\kassinawalks\' trialname '.txt'];
    end    
    kassinaInputs12(([folder '\' filename]),saveas, trial_start, trial_end, fixed);
    
     %% Call MJ and run simulation
   cd C:\Users\admin\Documents\Mujoco\mjpro150\bin
   string = ['Kassina_TL_MA ..\model\Kassina\Kassina.xml ' saveas]; 
   system(string);
   
    tendon_list = readtable("C:\Users\admin\Documents\Mujoco\mjpro150\output\tendon_list.txt");
    tendon_list = tendon_list.Properties.VariableNames;
    ten_length = readtable('C:\Users\admin\Documents\Mujoco\mjpro150\output\ten_length.txt');
    ten_length.Properties.VariableNames = tendon_list;
    moment_arm = csvread('C:\Users\admin\Documents\Mujoco\mjpro150\output\moment_arm.txt');

    if fixed
         outTenFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Walking data\tendon_length\' trialname '_fixed_ten_length.csv'];
         outMAFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Walking data\moment_arm\' trialname '_fixed_momentarm.csv'];
    else
         outTenFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Walking data\tendon_length\' trialname '_ten_length.csv'];
         outMAFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Walking data\moment_arm\' trialname '_momentarm.csv'];
    end
         writetable(ten_length,outTenFile);
    csvwrite(outMAFile,moment_arm);
    
end