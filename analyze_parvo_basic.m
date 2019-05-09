function [PA, P] = analyze_parvo_basic(P, blocks, subjblockorder, subjblocktimes, winsize_min)

%% Get metabolic power using Brockway equation
% Brockway, 1987
% Pmet,gross = 16.58 [W*s/(mL O2)]*VO2dot + 4.51 [W*s/(mL CO2)]*VCO2dot
% P.VO2 & P.VCO2 is in L/min
% P.VO2_kg is in mL/kg/min

P.MP_W = brockway(P.VO2*1000/60, P.VCO2*1000/60);
P.MP_W_kg = P.MP_W/P.info.subjwgt_kg;

%%
varnames = fieldnames(P);
varnames = varnames(~ismember(varnames,{'info' 'blocknames' 'blocktimes' 'writtentimes'}));
nv = length(varnames);

%% take time-weighted average of Parvo data
% This accounts for uneven sampling intervals

for d = 1:length(winsize_min)
    for b = 1:length(blocks)
        bi = find(strcmp(blocks{b},subjblockorder));
        [C, idx1] = min(abs(P.TIME-(subjblocktimes(bi,2)-winsize_min(d))));
        [C, idx2] = min(abs(P.TIME-subjblocktimes(bi,2)));
        weightings = [nan; diff(P.TIME(idx1:idx2))];
        disp([subjblockorder{bi} ',' num2str(P.TIME(idx1)) ',' num2str(P.TIME(idx2)) ', ' num2str(P.TIME(idx2)-P.TIME(idx1)) ', ' num2str(nansum(weightings))])
        for v = 1:nv
            eval(['data2mean = P.' varnames{v} '(idx1:idx2);']);  
            eval(['PA.' varnames{v} '(d,b) = wgtave(data2mean, weightings);']);
        end
    end
end
