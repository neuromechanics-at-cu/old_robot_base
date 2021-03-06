%Load Undorded without FML

projpath = pwd;
datafolder = [projpath filesep 'Data'];

expname = 'Preferred Mass';

expfolder= [projpath expname];
addpath([projpath filesep expname filesep 'mfiles']);
cd(expfolder);

excel_file = 'FullData_SmallTargetnoFML_test.csv';
fileID=fopen(excel_file,'w');

TrialCutoff=popts.totaltrials-nTrials_speed+1;

order(1)=3;
order(2)=2;
order(3)=1;
order(4)=4;
order(5)=5;
order(6)=6;
order(7)=1;
order(8)=2;
order(9)=3;
order(10)=4;
order(11)=5;
order(12)=6;
order(13)=1;
order(14)=2;
order(15)=4;
order(16)=3;
order(17)=5;
order(18)=6;

delcount=0;

A={'subj'...
    ',condition'...
    ',delcond'...
    ',order'...
    ',targetnum'...%5
    ',trial'...
    ',trialtot'...
    ',odd_trial'...
    ',movedur'...
    ',react1_dist'...%10    
    ',react1_vel'...
    ',pathltar'...
    ',pathlall'...
    ',maxex'...
    ',avevel_target'...%15
    ',peakvel_target'...
    ',peakvel_target_vsign'...
    ',avevel_all'...
    ',peakvel_all'...
    ',missangle'...%20
    ',absmissangle'...
    ',reaction_tanv'...
    ',reaction_tanvel'...
    ',miss_dist'...
    ',miss_rad'...%25
    ',endpt_vel'...
    ',endpt_tanvel'...
    ',reacttotar_frames'...
    ',tar_vel'...
    ',miss_dist'...%30
    ',miss_vel'...
    ',wait4mvt'...
    ',idxonset'...
    ',idxonset3'...
    ',idxpeakv'...%35
    ',idxendpt'...
    ',idxmoveback'...
    ',idxcrossing'...
    ',starttopeak'...
    ',peaktoend'...%40 
    ',react3_dist'...   
    ',react3_vel'...
    ',vthres_onset'...
    ',vthres_endpt'...
    ',endx'...%45
    ',endy'...
    ',sumVx'...
    ',sumVy'...
    ',sumVx_pdiff'...
    ',sumVy_pdiff'... %50
    ',sumVx_orig'...
    ',sumVy_orig'...
    };
fprintf(fileID,'%s',A{:});
fprintf(fileID,'\n');

for subj=1:nsubj
    
    subjid = subjarray{subjtoload(subj)};
    tr_count=1;
        
    for c=1:(nc) %Check if using FML or not

        condition_cur=char(conditions{subj}(c));

        if c>=2
            condition_prev=char(conditions{subj}(c));
        end

        if strcmp(condition_cur,'fml')
            condition_cur=0;
            condition_cur1=0;
        elseif strcmp(condition_cur,'0')
            condition_cur=0;
            condition_cur1=0;
        elseif strcmp(condition_cur,'3')
            condition_cur=3;
            condition_cur1=3;
        elseif strcmp(condition_cur,'5')
            condition_cur=5;
            condition_cur1=5;
        elseif strcmp(condition_cur,'8')
            condition_cur=8;
            condition_cur1=8;
        elseif strcmp(condition_cur,'0f')
            condition_cur=0;
            condition_cur1=10;
        end

        if c==1
            condition_prev=0;
        end

        if strcmp(condition_prev,'fml')
            condition_cur=0;
            condition_cur1=0;
        elseif strcmp(condition_prev,'0')
            condition_prev=0;
        elseif strcmp(condition_prev,'3')
            condition_prev=3;
        elseif strcmp(condition_prev,'5')
            condition_prev=5;
        elseif strcmp(condition_prev,'8')
            condition_prev=8;
        elseif strcmp(condition_prev,'0f')
            condition_prev=0;                
        end

        delcount=delcount+1;

        delta_condition=condition_cur-condition_prev;
        delta_condition1(delcount)=condition_cur-condition_prev;

        for i=1:size(Data{c,subj}.x,2)
            if mod(i,2) == 0
                odd_trial = 0;
            else
                odd_trial = 1;
            end
            [~,idxpeakv] = max(Data{c,subj}.v(MT{c,subj}.idxonset(i):end,i)); 
            idxpeakv = idxpeakv + MT{c,subj}.idxonset(i);
            
            Vx = diff(Data{c,subj}.x(:,i))/0.005;
            Vy = diff(Data{c,subj}.y(:,i))/0.005;
            
            A={subj...
                ,condition_cur...
                ,delta_condition1(delcount)...
                ,order(subj)...
                ,Data{c,subj}.targetnumber(i)...%5
                ,i...
                ,tr_count...
                ,odd_trial...
                ,Data{c,subj}.timings.reacttoendpt(i)...
                ,Data{c,subj}.P_abs(MT{c,subj}.idxonset(i),i)...%10
                ,Data{c,subj}.v(MT{c,subj}.idxonset(i),i)...
                ,Data{c,subj}.pathlength.totarget(i)...
                ,Data{c,subj}.pathlength.all(i)...
                ,max(Data{c,subj}.P_abs(:,i))...
                ,Data{c,subj}.avevel.totarget(i)...%15
                ,Data{c,subj}.peakvel.totarget(i)...
                ,MT{c,subj}.peakvy(i)...
                ,Data{c,subj}.avevel.all(i)...
                ,Data{c,subj}.peakvel.all(i)...
                ,Data{c,subj}.miss_angle(i)...%20
                ,abs(Data{c,subj}.miss_angle(i))...
                ,Data{c,subj}.reaction_tanv(i)...
                ,Data{c,subj}.TanV(MT{c,subj}.idxonset(i),i)...
                ,Data{c,subj}.miss_dist(i)...
                ,Data{c,subj}.P_abs(MT{c,subj}.idxendpt(i),i)...%25
                ,Data{c,subj}.v_sign(MT{c,subj}.idxendpt(i),i)...
                ,Data{c,subj}.TanV(MT{c,subj}.idxendpt(i),i)...
                ,MT{c,subj}.idxtarget(i)-MT{c,subj}.idxonset(i)...
                ,Data{c,subj}.v(MT{c,subj}.idxtarget(i),i)...
                ,Data{c,subj}.miss_dist(i)...%30
                ,Data{c,subj}.v(MT{c,subj}.idxendpt(i),i)...
                ,MT{c,subj}.robotstates.wait4mvt(i)...
                ,MT{c,subj}.idxonset(i)...
                ,MT{c,subj}.idxonset3(i)...
                ,idxpeakv...%35
                ,MT{c,subj}.idxendpt(i)...
                ,MT{c,subj}.idxmoveback(i)...
                ,MT{c,subj}.idxcrossing(i)...
                ,idxpeakv-MT{c,subj}.idxonset(i)...
                ,MT{c,subj}.idxendpt(i)-idxpeakv...%40
                ,Data{c,subj}.P_abs(MT{c,subj}.idxonset3(i),i)...
                ,Data{c,subj}.v(MT{c,subj}.idxonset3(i),i)...
                ,MT{c,subj}.idxvthresh_onset(i)...
                ,MT{c,subj}.idxvthresh_endpt(i)...
                ,Data{c,subj}.x(MT{c,subj}.idxmoveback(i),i)...%45
                ,Data{c,subj}.y(MT{c,subj}.idxmoveback(i),i)...
                ,sum(Data{c,subj}.vx(~isnan(Data{c,subj}.vx(:,i)),i))*0.005...
                ,sum(Data{c,subj}.vy(~isnan(Data{c,subj}.vx(:,i)),i))*0.005...
                ,sum(Vx(~isnan(Vx)))*0.005...
                ,sum(Vy(~isnan(Vx)))*0.005... %50
                ,sum(Data{c,subj}.vx_orig(~isnan(Data{c,subj}.vx_orig(:,i)),i))*0.005...
                ,sum(Data{c,subj}.vy_orig(~isnan(Data{c,subj}.vx_orig(:,i)),i))*0.005...
                };
            
                dlmwrite(excel_file,A,'-append')
            tr_count=tr_count+1;                
        end
    end
end
fclose(fileID);