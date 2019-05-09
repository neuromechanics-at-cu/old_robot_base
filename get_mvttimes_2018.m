%again plots commented out '%%%###'
function [MT,Data] = get_mvttimes(Data, V, P, Ev, vthres, endthres, tarthres,condition,subject);

if nargin < 3
    vthres = []; 
    endthres = 0.01;
    tarthres = .015;
end

j9=0;
vsign = sign(nanmean(V));
Data.vsigns = vsign;

psign = sign(nanmean(P));

pthres = sqrt(Data.targetposition(1,1)^2+Data.targetposition(1,2)^2);
targ_dist = ((Data.targetposition_act(:,1)-Data.startposition_act(:,1)).^2 + ...
    (Data.targetposition_act(:,2)-Data.startposition_act(:,2)).^2).^0.5;
targ_loc = (Data.targetposition_act(:,1).^2 + Data.targetposition_act(:,2).^2).^0.5;

low_react = 0;
n_long = 0;
P = (Data.x.^2 + Data.y.^2).^0.5;
time_s = Data.time_ms*0.001;

for i = 1:length(V(1,:))
    Data.p(sum(~isnan(Data.p(:,i))),i)=Data.p(sum(~isnan(Data.p(:,i)))-1,i);
    P(sum(~isnan(Data.p(:,i))),i)=P(sum(~isnan(Data.p(:,i)))-1,i);
%     vsign(i) = 1;
    [MT.peakvy(i) MT.idxpeakvy(i)] = nanmax(V(:,i)*vsign(i),[],1);
    [MT.minvy(i) MT.idxminvy(i)] = nanmin(V(:,i)*vsign(i),[],1);
    if MT.peakvy(i)>0.4
        1;
    end
end

if isempty(vthres)
    for i = 1:length(Y(1,:))
        MT.idxonset(i) = Ev{1,i+1}{1}(strcmp('home',Ev{1,i+1}{3}));
        MT.idxendpt(i) = Ev{1,i+1}{1}(strcmp('movingout',Ev{1,i+1}{3}))+1;
        MT.mvttime(i) = Data.time(MT.idxendpt(i),i)-Data.time(MT.idxonset(i),i);
    end
else
    n_fail = 0;
    n_long = 0;
    for i = 1:length(V(1,:))
        if sum(~isnan(Data.v(:,i))) ~= sum(~isnan(Data.p(:,i)))
            1;
        end
        
        %%
        %Robot state starts
%       if ~isempty(find(Data.statenumber(:,i)==3,1,'first');
        a=find(Data.statenumber(:,i)==3,1,'first');
        if ~isempty(a);
            MT.robotstates.home(i)=find(Data.statenumber(:,i)==3,1,'first');
        end
        a=find(Data.statenumber(:,i)==4,1,'first');
        if ~isempty(a);
            MT.robotstates.wait4mvt(i)=find(Data.statenumber(:,i)==4,1,'first');
        end
        a=find(Data.statenumber(:,i)==5,1,'first');
        if ~isempty(a);
            MT.robotstates.movingout(i)=find(Data.statenumber(:,i)==5,1,'first');
        end
        a=find(Data.statenumber(:,i)==6,1,'first');
        if ~isempty(a);
            MT.robotstates.attarget(i)=find(Data.statenumber(:,i)==6,1,'first');
        else
            [~,MT.robotstates.attarget(i)] = max(P(:,i));
        end
        a=find(Data.statenumber(:,i)==7,1,'first');
        if ~isempty(a);
            MT.robotstates.finishmvt(i)=find(Data.statenumber(:,i)==7,1,'first');
        end  

        if P(MT.robotstates.wait4mvt(i),i) >= .05
%             P(:,i) = (P(:,i)*-1)+.1;
            Data.RadV(:,i) = Data.RadV(:,i)*-1;
            Data.P_abs(:,i) = (P(:,i)*-1)+.1;
            MT.inwards(i) = 1;
        else
            Data.P_abs(:,i) = P(:,i);
            MT.inwards(i) = 0;
        end
        P(:,i) = Data.P_abs(:,i);
        
        %Position threshold for target
        if max(P(MT.robotstates.wait4mvt(i):end,i))>tarthres
            b(i) = find(P(MT.robotstates.wait4mvt(i):end,i)*psign >= tarthres,1,'first');
            if ~isempty(b(i))
                MT.idxtarget(i)=b(i)+MT.robotstates.wait4mvt(i);
            else
                [tarmax(i),MT.idxtarget(i)]=max(P(MT.robotstates.wait4mvt(i):end,i));
                MT.idxtarget(i) = MT.idxtarget(i)+MT.robotstates.wait4mvt(i);
            end
        else
            [MT.targdist(i),MT.idxtarget(i)] = max(P(MT.robotstates.wait4mvt(i):end,i));
            MT.idxtarget(i) = MT.idxtarget(i)+MT.robotstates.wait4mvt(i);
        end
        MT.idxtarget(i) = MT.idxtarget(i)-2;
       

       %%
        %Current Distance to target
        D_to_tar = sqrt((Data.x(:,i)-Data.targetposition(i,1)).^2+(Data.y(:,i)-Data.targetposition(i,2)).^2);
        MT.Dtotardiff(:,i) = diff(D_to_tar);
        
        %Reaction time from tangential velocity
        clear V_diff j
%         V_diff(:,i) = diff(Data.TanV(:,i));
        V_diff(:,i) = Data.TanA(:,i);
        j = MT.robotstates.wait4mvt(i)+...
            find(Data.TanV(MT.robotstates.wait4mvt(i):end,i)>.2*max(Data.TanV(MT.robotstates.wait4mvt(i):end,i)),1,'first');
        react1_found = 0;
        react3_found = 0;
        if j<20
            j=20;
        end
        while  j>=12 %&& j>=MT.robotstates.wait4mvt(i)+1 %
            tanvstd = std(Data.TanV(j-10:j,i));
            vdiffstd = std(V_diff(j-10:j,i));
            j=j-1;
            if (j <= 12 || tanvstd<6e-4 || (vdiffstd<7.5e-3 && tanvstd<2e-3) ||  sum(V_diff(j:j+10,i)>0)<3) && Data.TanV(j,i) < .2*max(Data.TanV(:,i)) && react1_found == 0
                MT.idxonset(i)=j;
                react1_found = 1;
            end
            if (j >= 12 || tanvstd<6e-4 || vdiffstd<7.5e-3 || sum(V_diff(j:j+10,i)>0)<3) && Data.TanV(j,i) < .2*max(Data.TanV(:,i)) && react3_found == 0 && j >= MT.robotstates.wait4mvt(i)
                MT.idxonset3(i)=j;
                react3_found = 1;
                j9 = j9+1;
            elseif j == MT.robotstates.wait4mvt(i) && react3_found == 0
                MT.idxonset3(i) = j;
                react3_found = 1;
                j9=j9+1;
            end
        end
        if react1_found==0
            react1_found=1;
            MT.idxonset(i) = j;
        end
        if react3_found==0
            react3_found=1;
            MT.idxonset3(i) = j;
        end
        if vsign(i) == sign(nanmean(V(MT.idxonset(i):end,i)))
            1;
        else
            vsign(i) =  sign(nanmean(V(MT.idxonset(i):end)));
            fprintf('Flipped Vsign');
        end
        
        clear V_diff
        highV = find(abs(V(MT.robotstates.wait4mvt(i):end,i))>.3*max(abs(V(MT.robotstates.wait4mvt(i):end,i))),1,'first');
        highV = highV+MT.robotstates.wait4mvt(i);
        %         [~,highV] = max(abs(V(:,i)));
        Vthreshold=0.05;
        Athreshold=0.0001;
        Difference=diff(abs(V(:,i)));
        V_diff(:,i) = Difference;
        lastlowAcc=find(Difference(1:highV)<Athreshold,1,'last');
        framebegVelocity(i)=find(V(1:highV,i)<Vthreshold,1,'last');

        if min(Difference(1:framebegVelocity(i)))>Athreshold  				
            lastlowAccVel=find(Difference(1:framebegVelocity(i))==min(Difference(1:framebegVelocity(i))));
        else    
            lastlowAccVel=find(Difference(1:framebegVelocity(i))<Athreshold,1,'last');
        end
        if lastlowAccVel < MT.robotstates.wait4mvt(i)
            lastlowAccVel = MT.robotstates.wait4mvt(i);
        end

        MT.idxonsetErik(i) = lastlowAccVel;
        
        if MT.idxonset(i)-MT.robotstates.wait4mvt(i)<20
            low_react = low_react+1;
            1;
        end
        
        V_diff(:,i) = diff(Data.TanV(:,i));
%         if MT.idxonset(i)-MT.robotstates.wait4mvt(i)>150
%             fprintf('high reaction %g\n',MT.idxonset(i)-MT.robotstates.wait4mvt(i));
%             figure(1);clf(1);
%             subplot(2,1,1);
%             plot(V_diff(:,i));
%             subplot(2,1,2);
%             plot(Data.TanV(:,i));
%             1;
%             fprintf('Subj %g, cond %g, trial %g, reaction is %g \n',subject,condition,i, MT.idxonset(i));
%         end
            
        %% Correction Index
        
%         [~,j]=max(V(1:MT.idxtarget(i),i));
%         j=j+10;
%         while 1 
%             if j > 1000
%                 1;
%             end
%             if V_diff(j,i)<0
%                 j=j+1;
%             elseif V_diff(j,i)>0
%                 MT.idxcorrection(i)=j;
%                 break
%             end
%         end        
        
        %%
        %Position threshold for moving back
        k=MT.idxtarget(i);
        while P(k+1,i) > P(k,i)
            if k+1<length(P(:,i))
                k=k+1;
            else
                break
            end
        end
        MT.idxmoveback(i)=k;
              
        if max(P(:,i))>0.1
            MT.idxcrossing(i) = find(P(:,i)>.1,1,'first');
        else
            [~,MT.idxcrossing(i)] = max(P(:,i));
        end
        
        % End point: first time they turn around (velocity = 0) or last
        % data point in trial
        tempV = Data.TanV(MT.robotstates.wait4mvt(i):end,i);
        tempP = Data.P_abs(MT.robotstates.wait4mvt(i):end,i);
        
        for k=1:length(diff(D_to_tar))-50
            D_to_tar_var(k) = std(diff(D_to_tar(k:k+30)));
        end
        
        [~,MT.idxpeakvelreact(i)] = max(V(MT.idxonset(i):end,i));
        test = find(D_to_tar(MT.idxonset(i):end)<.02,1,'first')+MT.idxonset(i);
        if max(P(MT.robotstates.wait4mvt(i):end,i))>.090
            if ~isempty(test)
                a = find(D_to_tar(MT.idxonset(i):end)<.02,1,'first')+MT.idxonset(i);
            else
                a = find(P(MT.idxonset(i):end,i)>.090,1,'first')+MT.idxonset(i);
            end
%             if sum(~isnan(P(:,i)))-a>300
%                 endpt = a+300;
%             elseif sum(~isnan(P(:,i)))-a>200
%                 endpt = a+200;
%             elseif sum(~isnan(P(:,i)))-a>100
%                 endpt = a+100;
%             else
%                 endpt = sum(~isnan(P(:,i)));
%             end
            endpt = sum(~isnan(P(:,i)));
            for k = a:endpt
%                  if sum(Data.v(k:k+10,i)<.015)>=8 & P(k,i)>.095
                 if k+10>=endpt || (std(Data.TanV(k:k+10,i))<2e-3 & P(k,i)>.090)
                    break
                 end
            end
            if k == endpt
                fprintf('didnt use std\n');
            end
            b = k;
        else
            b = find(Data.TanV(test:end,i) <= 0, 1, 'first')+test;
        end
        
        if ~isempty(b)
            MT.idxendpt(i) = b-1;
        else
            MT.idxendpt(i) = find(~isnan(V(:,i)),1,'last')-1;
        end
%         [~,MT.idxendpt(i)] = max(P(:,i));
%         fprintf('V_end = %f, idxnedpt = %g\n',Data.v_sign(MT.idxendpt(i),i),MT.idxendpt(i))
%         plot(Data.v_sign(:,i))
        1;
        
%         if max(P(:,i))>.08
%             a = find(tempP > .08, 1, 'first');
%             [~,b] = min(Data.v(a+MT.robotstates.wait4mvt(i):end,i));
% %             b = find(Data.TanV(find(P(:,i)>.08,1,'first'):end,i) <= 0, 1, 'first')+find(P(:,i)>.08,1,'first');
%         else
%             b = find(Data.TanV(MT.idxpeakvy(i):end,i) <= 0, 1, 'first');
%         end
%         if ~isempty(b)
% %             MT.idxendpt(i) = MT.idxpeakvy(i)+b-1;
%             MT.idxendpt(i) = MT.robotstates.wait4mvt(i)+a+b-1;
%         else
% %             n_fail = n_fail+1;
% %             fprintf('Didnt find a final point, indexing to last \n');
%             MT.idxendpt(i) = find(~isnan(V(:,i)),1,'last')-1;
%         end
        xerr = Data.x(MT.idxmoveback(i),i) - Data.targetposition_act(i,1);
        yerr = Data.y(MT.idxmoveback(i),i) - Data.targetposition_act(i,2);
        err = sqrt(xerr^2+yerr^2);
        
        MT.timeonset(i) = time_s(MT.idxonset3(i),i);
        MT.timeendpt(i) = time_s(MT.idxendpt(i),i);
        
        %% Bad velocity thresholding idx
%         vsign(i)=1;
        [MT.peakvy(i),b] = nanmax(V(MT.robotstates.wait4mvt(i):MT.idxendpt(i),i)*vsign(i),[],1);
        MT.idxpeakvy(i) = b+MT.robotstates.wait4mvt(i);
        [MT.minvy(i),b] = nanmin(V(MT.robotstates.wait4mvt(i):end,i)*vsign(i),[],1);
        MT.idxminvy(i) = b+MT.robotstates.wait4mvt(i);

        MT.timepeakvy(i) = time_s(MT.idxpeakvy(i),i);
        MT.timeminvy(i) = time_s(MT.idxminvy(i),i);
        
        MT.idxvthresh_onset(i) = find(V(MT.idxonset(i):MT.idxendpt(i),i)*vsign(i)>0.01,1,'first')+MT.idxonset(i);
        if min(Data.TanV(MT.idxpeakvy(i):MT.idxendpt(i),i))<.01
            MT.idxvthresh_endpt(i) = find(Data.TanV(MT.idxpeakvy(i):MT.idxendpt(i),i)<0.01,1,'first')+MT.idxpeakvy(i);
        else
            [~,MT.idxvthresh_endpt(i)] = min(Data.TanV(MT.idxpeakvy(i):MT.idxendpt(i),i));
            MT.idxvthresh_endpt(i) = MT.idxvthresh_endpt(i) + MT.idxpeakvy(i);
        end
        
        %%        
        if isnan(MT.idxendpt(i)) || isnan(Data.v(MT.idxendpt(i),i)) || isnan(Data.p(MT.idxendpt(i),i))
            MT.idxendpt(i) = min([find(~isnan(Data.v(:,i)),1,'last'),...
                find(~isnan(Data.v(:,i)),1,'last')]);
            fprintf('Nan endpt, trial; %g\n',i);
        end
        
        MT.timeendpt(i) = time_s(MT.idxendpt(i),i);
        
        if isnan(Data.v(MT.idxendpt(i),i)) || isnan(Data.p(MT.idxendpt(i),i))
            fprintf('NAN ENDPT\n');
        end

        MT.timetarget(i) = time_s(MT.idxtarget(i),i);

        % wait4mvt
        MT.idxwait4mvt(i) = Ev{1,i+1}{1}(strcmp('home',Ev{1,i+1}{3}))+1;        
        MT.timewait4mvt(i) = time_s(MT.idxwait4mvt(i),i);
        MT.attar_to_endpt(i) = MT.idxendpt(i) - MT.robotstates.attarget(i);
        
        % Movement time and rxn time
        MT.timeendpt(i) = time_s(MT.idxendpt(i),i);
        MT.mvttime(i) = MT.timeendpt(i)-MT.timeonset(i);
        MT.rxntime(i) = MT.timeonset(i) - MT.timewait4mvt(i);
        
        pathl=0;
        for k=MT.idxonset(i)+1:MT.idxendpt(i)
            pathl = pathl + sqrt((Data.x(k,i)-Data.x(k-1,i)).^2+...
                (Data.y(k,i)-Data.y(k-1,i)).^2);
        end
        if Data.targetposition(i,1)>0
            if Data.targetposition(i,2)>0
                t_num = 1;
            else
                t_num = 4;
            end
        elseif Data.targetposition(i,1)<0
            if Data.targetposition(i,2)<0
                t_num = 3;
            else
                t_num = 2; 
            end
        end
        MT.peakvy(i);
        max(Data.v_sign(MT.idxonset(i):MT.idxendpt(i),i)*vsign(i));
        1;
%         figure(3);
%         hold on
%         if mod(i,2) && MT.peakvy(i)<.6
%             switch mod(t_num,2)
%                 case 0
%                     plot(MT.mvttime(i),MT.peakvy(i),'bo')
%                 case 1
%                     plot(MT.mvttime(i),MT.peakvy(i),'ro')
%             end
%         end
        if MT.idxmoveback(i)<MT.idxonset(i)
            1;            
            figure(1);clf(1);subplot(3,2,1);
            hold on
            plot(V(:,i));
            plot(MT.robotstates.wait4mvt(i),V(MT.robotstates.wait4mvt(i),i),'*')
            plot(MT.idxonset(i),V(MT.idxonset(i),i),'x');
            plot(MT.idxonsetErik(i),V(MT.idxonsetErik(i),i),'square');
            plot(MT.idxendpt(i),V(MT.idxendpt(i),i),'o');
            title('Velocity')
            subplot(3,2,2);
            hold on
            plot(P(:,i));
            plot(MT.robotstates.wait4mvt(i),P(MT.robotstates.wait4mvt(i),i),'*')
            plot(MT.idxonset(i),P(MT.idxonset(i),i),'x');
            plot(MT.idxonsetErik(i),P(MT.idxonsetErik(i),i),'square');
            plot(MT.idxendpt(i),P(MT.idxendpt(i),i),'o');
            title('Position')
            subplot(3,2,3);
            hold on
            plot(V_diff(:,i));
            plot(MT.robotstates.wait4mvt(i),V_diff(MT.robotstates.wait4mvt(i),i),'*')
            plot(MT.idxonset(i),V_diff(MT.idxonset(i),i),'x');
            plot(MT.idxonsetErik(i),V_diff(MT.idxonsetErik(i),i),'square');
            plot(MT.idxendpt(i)-1,V_diff(MT.idxendpt(i)-1,i),'o');
            title('V_diff')
            subplot(3,2,4);
            hold on
            plot(Data.x(:,i),Data.y(:,i));
            plot(Data.x(MT.idxonset(i),i),Data.y(MT.idxonset(i),i),'x');
            plot(Data.x(MT.idxonsetErik(i),i),Data.y(MT.idxonsetErik(i),i),'square');
            plot(Data.x(MT.idxendpt(i),i),Data.y(MT.idxendpt(i),i),'o');
            title('X-Y')
            subplot(3,2,5);
            plot(Data.TanV(:,i));
            hold on
            title('Tan V');
            plot(MT.robotstates.wait4mvt(i),Data.TanV(MT.robotstates.wait4mvt(i),i),'*')
            plot(MT.idxonset(i),Data.TanV(MT.idxonset(i),i),'x');
            plot(MT.idxonsetErik(i),Data.TanV(MT.idxonsetErik(i),i),'square');
            plot(MT.idxendpt(i),Data.TanV(MT.idxendpt(i),i),'o');
            drawnow
            subplot(3,2,6);
            plot(Data.v(:,i));
            fprintf('Movement Time: %f \n',MT.mvttime(i));
            fprintf('Pathl: %f \n',pathl);
            1;
        end
        
%         if Data.TanV(MT.idxonset(i),i)>.025 && (MT.idxonset(i)-MT.robotstates.wait4mvt(i))<=1
%             n_0react_fastreact = n_0react_fastreact +1;
%         end
%         if Data.TanV(MT.idxonset(i),i)>.025
%             n_fastreact = n_fastreact+1;
%         end
%         n_trials = n_trials +1;
% 
%         if condition(end) =='S' || sum(condition(end-1:end) =='VS')==2 || condition(end) =='M'
%             if MT.mvttime(i) < .5
%                 1;
%             end
%         
%         end
    end
    fprintf('React<20 frames = %g, react0 = %g\n',low_react,j9);
end



% Intertrial time = between one trial end and next trial beginning
MT.intertrial(1) = 0;  % zero for first trial
for i=2:length(V(1,:))
    MT.intertrial(i) = MT.timewait4mvt(i)-MT.timeendpt(i-1);
end

MT.time_s = time_s;
end