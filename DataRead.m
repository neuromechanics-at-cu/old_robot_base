% Basic script that analyzes robot data
% Experiment type: by conditions (ex. speed)
% Updated by Gary: 5-10-2019

% statedata
% 1 viewmenu    5 movingout     9  intertrial     13 exitgame
% 2 startgame   6 attarget      10 warning
% 3 home        7 finishmvt     11 game_message
% 4 wait4mvt    8 movingback    12 rest

clear all
%% Experiment specific details
% The parent folder for the mfiles and dat files 
% If you keep the same folder system as in the git repo this shouldn't need
% to change.
projpath = pwd;
% Make sure to addpath where the mfiles are stored.
addpath(projpath);

% This is important for get_robotinfo
% Add this name into line 6, 16, or 26 depending. Can also just make your
% own.
expname = 'Example';

% Good idea to store the data in this format, where data contains folders
% for each individual subject, and inside the subject folders are the
% specific condition files.
datafolder_names = [projpath filesep 'Data'];
expfolder= [projpath];

% Settig what you want the output mat file to be named.
filename = 'Example';
fprintf('%s \n',filename);

% Pulls the names of the data from the data folder and stores in an array
% This is where you say which subjects you want to analyze. subjtoload will
% be an array of subject numbers that you want to load.
% If all subjects, subjtoload = [1:length(list)-2];
cd(datafolder_names);
subjtoload = 1:4;
nsubj=length(subjtoload);

list = dir;
list = list(2+subjtoload);
subjarray = list.name;

cd(expfolder);

% Initialize the conditions. for this data each data file is labeled as
% 'Subj_c', where c is the condition.
conditions={'fml' '0' '3' '5' '8'};
% If you want unordered need to create conditions for each subject:
% conditions{1}={'fml' '0' '3' '5' '8'};
nc=length(conditions);

ColorSet = parula(nsubj);

% These threshold values are from legacy movement time algorithms. They're
% still here if you want to compare older methods to the current method.
% end threshold for movement end, one for each condition
endthres = 0.015*ones(1,nc);
% Threshold for target distance
tarthres = 0.10*ones(1,nc);
% vthres for movement onsets, one for each condition
vthres = 0.01*ones(1,nc);

% Defining how many trials per block. I would just have totaltrials equal
% all the trials, and deal with data filtering later.
popts.practicetrials = 0; % # familiarization trials, without metabolic data
popts.totaltrials = 400;  % # total trials

% Adjusting robot sampling rate, generally this won't change.
fsR=200; tsR=1/fsR; % robot sampling frequency

% Some robot options, may not need to alter these.
ropts.rotate = 0;
ropts.switchhometar = 0;
ropts.longtrialtime_frames = 5*fsR; %4 seconds

scrsz = get(0,'ScreenSize');
color2use = {'k' 'b' 'r' 'g' 'm' 'c' 'k' 'b' 'r' 'g' 'm' 'c'};

%% Load Robot Data
% Get robot info
[ropts.statenames, ropts.avstatenames, ropts.robotvars] = get_robotinfo(expname);

% Initalize main data structures.
T{nc,nsubj} = []; Ev{nc,nsubj} = []; Data{nc,nsubj} = []; MT{nc,nsubj} = [];
for subj = 1:nsubj
    subjid = subjarray{subjtoload(subj)};
    for c = 1:nc
        condition{c,subj} = conditions{c};
        
        datafolder{c,subj} = [datafolder_names filesep subjid filesep subjid '_' condition{c,subj}];
        
        num_trials{c,subj} = 400;
    end
end

% Begin to analyze the data.
for subj = 1:nsubj
    subjid = subjarray{subjtoload(subj)};
    for c = 1:nc
        % read robot .dat files
        % Here is where you would make which trials to analyze. Put it into
        % the popts data structure.
        T{c,subj}=dataread_robot(subjid,datafolder{c,subj});
        popts.totaltrials(c) = length(T{c,subj}.framedata);
    end
    for c = 1:nc
        
        % Get robot events
        Ev{c,subj} = get_robotevents(T{c,subj}.framedata, ropts.statenames, ropts.avstatenames);

        % Data is structure of matrices concatenated from T.framedata
        % Then remove framedata field from T
        [Data{c,subj}, Ev{c,subj}] = analyze_robot_basic(T{c,subj}, Ev{c,subj}, ropts,c,subj);

        % Get movement position and velocity
        Data{c,subj}.v = (Data{c,subj}.vx.^2+Data{c,subj}.vy.^2).^0.5;
        Data{c,subj}.p = ((Data{c,subj}.x).^2 + (Data{c,subj}.y).^2).^0.5;

        % Get velocity by differentiating position (Data.v is bumpy)
        Data{c,subj} = get_vsign(Data{c,subj},num_trials{c,subj});
                
    end
    for c = 1:nc
        % Get movement times
        [MT{c,subj},Data{c,subj}] = get_mvttimes(Data{c,subj}, Data{c,subj}.v_sign, Data{c,subj}.p, Ev{c,subj}, vthres(c), endthres(c), tarthres(c),c,subj);
        fprintf('Subject %g %s Condition %s Processed\n',subj,subjid,condition{c,subj});
    end
end

