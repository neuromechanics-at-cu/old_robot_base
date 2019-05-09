% Plots have been commented out with marker '%%%###'
function [Data, Ev] = analyze_robot_basic(T, Ev, opts,c,subj)

fsR=200; tsR=1/fsR; % robot sampling frequency

%% Define start and target position in robot space
% initialize/preallocate
Data.startposition_act=zeros(T.config.totaltrials,3);
Data.targetposition_act=zeros(T.config.totaltrials,3);

Data.startposition=zeros(T.config.totaltrials,3);
Data.targetposition=zeros(T.config.totaltrials,3);

for i=1:T.config.totaltrials    
    % frame in wait4mvt
    idx = min(Ev{1,i+1}{1}(strcmp('home',Ev{1,i+1}{3}))+2); 
    
    % actual location in tcl space
    Data.startposition_act(i,1) = T.framedata(i).homex(idx)*0.01; % in m
    Data.startposition_act(i,2) = T.framedata(i).homey(idx)*0.01; % in m
    
    Data.targetposition_act(i,1) = T.framedata(i).targetx(idx)*0.01; % in m
    Data.targetposition_act(i,2) = T.framedata(i).targety(idx)*0.01; % in m
    if Data.targetposition_act(i,1)==0
        1;
    end
end

% If home and target switches, revise T.startposition_act and
% T.targetposition_act
opts.switchhometar = 0;
if opts.switchhometar
    Data.startposition_act = [Data.targetposition_act(2:T.config.totaltrials,:); Data.targetposition_act(T.config.totaltrials-1,:)];
end

% shift initial start to 0,0,0 and adjust target
Data.startposition = Data.startposition_act-repmat(Data.startposition_act(1,:),T.config.totaltrials,1);
Data.targetposition = Data.targetposition_act-repmat(Data.startposition_act(1,:),T.config.totaltrials,1);

% if screen rotated, switch x & y
opts.rotate = 0;
if opts.rotate
    Data.startposition_act(:,1:2) = [Data.startposition_act(:,2) Data.startposition_act(:,1)];
    Data.targetposition_act(:,1:2) = [Data.targetposition_act(:,2) Data.targetposition_act(:,1)];
    
    Data.startposition(:,1:2) = [Data.startposition(:,2) Data.startposition(:,1)];
    Data.targetposition(:,1:2) = [Data.targetposition(:,2) Data.targetposition(:,1)];
end
 
%% Determine the maximum number of samples among all trialdata sets
for i= 1:T.config.totaltrials
    frames(i) = length(T.framedata(i).frame);
end

[sortedframes idx] = sort(frames);
longtrials = idx(sortedframes > opts.longtrialtime_frames);

% There are some really long trials because I start collecting when the
% state changes to home. Ex. trial 21 in the metabolic curve protocol is ~1
% minute.

% if ~isempty(longtrials) 
%     for i = 1:length(longtrials)
%         figure
%         X = T.framedata(longtrials(i)).time;
%         [AX,H1,H2] = plotyy(X, T.framedata(longtrials(i)).y, X, T.framedata(longtrials(i)).statenumber);
%         set(get(AX(1),'Ylabel'),'String','y-pos')
%         set(AX(2),'ytick', 1:1:length(opts.statenames),'yticklabel',opts.statenames);
%         set(get(AX(2),'Ylabel'),'String','States')
%         xlabel('Time (s)');
%     end
% end
maxframe = max(sortedframes(sortedframes <= opts.longtrialtime_frames));

%% Create matrices of robotvars
%  Preallocate 
for v = 1:length(opts.robotvars)
    eval(['Data.' opts.robotvars{v} ' = NaN(maxframe,T.config.totaltrials);']);
end

frames(frames > maxframe) = maxframe;

for i=1:T.config.totaltrials
    if ismember(i,longtrials)
        idx = Ev{1,i+1}{1}(strcmp('home',Ev{1,i+1}{3}));
        nl = length(T.framedata(i).time(idx:end));
        nl = min(nl, maxframe);
        for v = 1:length(opts.robotvars)
            eval(['Data.' opts.robotvars{v} '(1:nl,i)=T.framedata(i).' opts.robotvars{v} '(idx:idx+nl-1);']);
            
            % Re-identify events to match truncated data
            if v == length(opts.robotvars)
                select_framedata.statenumber = T.framedata(i).statenumber(idx:idx+nl-1);
                select_framedata.avstatenumber = T.framedata(i).avstatenumber(idx:idx+nl-1);
                select_framedata.robotstatenumber = T.framedata(i).robotstatenumber(idx:idx+nl-1);
                Ev_temp = get_robotevents(select_framedata, opts.statenames, opts.avstatenames);
                Ev(:,i+1) = Ev_temp(:,2);
            end
        end
    else
        for v = 1:length(opts.robotvars)
            eval(['Data.' opts.robotvars{v} '(1:frames(i),i)=T.framedata(i).' opts.robotvars{v} '(1:frames(i));']);
        end
    end
    Vx = [0;diff(Data.x(:,i))/0.005];
    Vy = [0;diff(Data.y(:,i))/0.005];
    Vx(sum(~isnan(Vx))) = Vx(sum(~isnan(Vx))-1);
    Vy(sum(~isnan(Vy))) = Vy(sum(~isnan(Vy))-1);
    Data.vx(:,i) = Vx;
    Data.vy(:,i) = Vy;
    Data.x(sum(~isnan(Data.x(:,i))),i) = Data.x(sum(~isnan(Data.x(:,i)))-1,i);
    Data.y(sum(~isnan(Data.y(:,i))),i) = Data.y(sum(~isnan(Data.y(:,i)))-1,i);
end


%% Find Force TrialTypes
Data.catchtrials = find(strcmp(T.trialtypename,'clamp')==1);
Data.nulltrials = find(strcmp(T.trialtypename,'null')==1);
Data.divtrials = find(strcmp(T.trialtypename,'div')==1);
Data.curltrials = find(strcmp(T.trialtypename,'curl')==1);

%% PLOTS
%     figure;
%     subplot(231); plot(T.framedata(1).x, T.framedata(1).y); ylabel('robot y'); xlabel('robot x');
%  if ischar(T.config.curltrials)
%      trial2plot1 = 2;
%      trial2plot2 = 3;
%  else
%      trial2plot1 = T.config.curltrials(1);
%      trial2plot2 = T.config.curltrials(2)+1;
%  end
%     subplot(232); plot(T.framedata(trial2plot1).x, T.framedata(trial2plot1).y); ylabel('robot y'); xlabel('robot x');
%     subplot(233); plot(T.framedata(trial2plot2).x, T.framedata(trial2plot2).y); ylabel('robot y'); xlabel('robot x');
%  
%     subplot(234); plot(Data.x(:,1), Data.y(:,1)); ylabel('robot y'); xlabel('robot x');
%     subplot(235); plot(Data.x(:,trial2plot1),Data.y(:,trial2plot1)); ylabel('robot x'); xlabel('robot y');
%     subplot(236); plot(Data.x(:,trial2plot2), Data.y(:,trial2plot2)); ylabel('robot x'); xlabel('robot y');
%% Find and fix discontinuities
 Data = find_discont(Data.x, Data.y, Data.vy, Data, c,subj);
 
 %Data = fix_discont(Data);

