% Script to read Kassina trial metadata, use it to extract trial
% metadata from premade tables, use it to locate relevant trial xyz points,
% cut out the jump (start:end), convert xyz points (8 markers) to
% quaternions, run Kassina_TL_MA.exe (MJ) from matlab, then import the
% resulting moment arms and tendon lengths, then export into text files.

% Need metadata5.xlsx in the same folder. Also calls KassinaInputs8.m and
% transformFrogData8.m, in my case these are in
% 'C:\Users\admin\Documents\Mujoco\mjpro150\scripts\', as well as some
% quaternion functions found in \scripts\quaternions.

% Chris Basu

addpath 'quaternions';
% specifiy options for importing metadata file
opts = spreadsheetImportOptions;

opts.VariableNames = ["frog_index", "animal", "type", "trial_index", "trial", "date", "frate", "vrate", "startframe", "endframe", "cutoff", "correctFstart", "correctFend", "jumpstart", "jumpend", "wstart", "wend", "framecount", "realWeight", "SVlength", "file_name", "modified", "force_file_name", "nudge", "vto", "chunk_start", "TO_angle", "vto_NoNaN", "trial_tags", "kinematics_filter_cutoff"];
opts.VariableTypes = ["double", "categorical", "categorical", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string", "double", "string", "double", "double", "double", "double", "double", "double", "double"];
opts.DataRange = "A2:AD51";
metadata5= readtable('metadata5.xlsx',opts);

numtrials = size(metadata5,1);
for i= 1:numtrials
    %% Get trial metadata
    animal = char(metadata5.animal(i));
    type = char(metadata5.type(i));
    if  (metadata5.trial_index(i))<10
        trial = ['0' num2str(metadata5.trial_index(i))];
    else
    trial =  num2str(metadata5.trial_index(i));
    end
    trialname = [animal type trial];
    trial_metadata = metadata5(metadata5.animal==animal & metadata5.trial_index == str2num(trial),:);
    trial_start = trial_metadata.chunk_start;
    trial_end = trial_metadata.vto_NoNaN;
    
    %% Make quaternions from xyz points
    
   filename = ['C:\Users\admin\Documents\DATA\kassina kinematics\Sept2016\' animal '_HOP_' trial '_xyzPOINTS.dat'];
   saveas = ['C:\Users\admin\Documents\Mujoco\mjpro150\input\kassinawalks\' animal '_HOP_' trial '.txt'];
   kassinaInputs8(filename,saveas,trial_start,trial_end);
    
    %% Call MJ and run simulation
   cd C:\Users\admin\Documents\Mujoco\mjpro150\bin
   string = ['Kassina_TL_MA ..\model\Kassina\Kassina.xml ' saveas]; 
   system(string);
   
    tendon_list = readtable("C:\Users\admin\Documents\Mujoco\mjpro150\output\tendon_list.txt");
    tendon_list = tendon_list.Properties.VariableNames;
    ten_length = readtable('C:\Users\admin\Documents\Mujoco\mjpro150\output\ten_length.txt');
    ten_length.Properties.VariableNames = tendon_list;
    moment_arm = csvread('C:\Users\admin\Documents\Mujoco\mjpro150\output\moment_arm.txt');

    
    outTenFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Sept2016\tendon_length\' trialname '_ten_length.csv'];
    outMAFile = ['C:\Users\admin\Documents\DATA\kassina kinematics\Sept2016\moment_arm\' trialname '_momentarm.csv'];
    writetable(ten_length,outTenFile);
    csvwrite(outMAFile,moment_arm);
    

    
   
end
