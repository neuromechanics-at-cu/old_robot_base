function [statenames, avstatenames, robotvars] = get_robotinfo(expname)
%% Updated 5-10-2019: Gary Bruening
% This file initializes the states and avstates of the robot. Generally not
% that important to alter this, the biggest one is having wait4mvt as this
% is when the target generally shows up.

% Only change the state and avstates if you altered them in the .tcl files.


%%
% sets robot code states according to the experiment name
% robot states changed as robot code evolved

if ismember(expname, {'Mass','Example'})
    statenames = {'viewmenu' 'startgame' 'home' 'wait4mvt' 'movingout' 'attarget' 'movingback' 'finishmvt' 'intertrial'  'rest' 'warning' ...
        'game_message' 'exitgame'};
    avstatenames = {'blank' 'clear_messagewin' 'show_homeonly' 'show_home' 'show_score' 'show_target' ...
        'show_tooslowtarget' 'show_toofasttarget' 'show_goodjob' 'show_noreward' 'show_aimtarget' 'show_clearhome' ...
        'show_jumptarget' 'show_cleartarget' 'show_cleartargetscore' 'show_score_only' 'rest' ...
        'show_movefaster' 'show_stayincircle' 'show_fail' 'show_messagebox' 'show_gameover'};
    robotvars = {'time' 'time_ms' 'statenumber' 'avstatenumber' 'x' 'y' 'vx' 'vy' 'fx' 'fy' 'ftx' 'fty' 'ftz'};
end
end