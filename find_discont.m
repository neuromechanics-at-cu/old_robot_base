% plots commented out and marked '%%%###'
function Data = find_discont(Px, Py, V, Data, c,subj)

% assumes each trial is a col
% V can be Vy or Vx

scrsz = get(0,'ScreenSize');

% figure, plot(range(diff(V)),'o'), ylabel('range of diff(V)'), xlabel('trial')
Data.discont.trials = find(range(diff(V)) > mean(range(diff(V)))*4);


for i = 1:length(Data.discont.trials)
    
    Data.discont.nspikes(i)=length(find(diff(V(:,Data.discont.trials(i)))>mean(range(diff(V)))*2));
     
%     clf(1);
%     figure(1)
%     hold on
%     a=sprintf('Condition: %g, Subject: %g, Trial %g',c,subj,Data.discont.trials(i));
%     plot(Data.x(:,Data.discont.trials(i)));
%     plot(Data.y(:,Data.discont.trials(i)));
%     xlabel(a);
    
%     figure(3)
%     plot(diff(V(~isnan(V(:,Data.discont.trials(i))))))
    
%     if Data.discont.nspikes(i)==1
%         [minval Data.discont.frame(i,1)] = min(diff(V(:,Data.discont.trials(i))));
%         [maxval Data.discont.frame(i,2)] = max(diff(V(:,Data.discont.trials(i))));
%         
%         [minval Data.discont.frame(i,1)] = min(diff(Data.x(:,Data.discont.trials(i))));
%         [maxval Data.discont.frame(i,2)] = max(diff(Data.x(:,Data.discont.trials(i))));
% 
%         % Sometimes the spike happens at the beginning/end of trial and it
%         % can't find other side, so it picks a frame at the max of the movement
%         % and results in beginning/end frames that are out of order.
%         if Data.discont.frame(i,1)>Data.discont.frame(i,2)
%             num_frames = sum(~isnan(V(:,Data.discont.trials(i))));
%             if abs(Data.discont.frame(i,1) - num_frames)<=3
%                 Data.discont.frame(i,2) = Data.discont.frame(i,1); % at end of mvt
%             elseif Data.discont.frame(i,2) <=3
%                 Data.discont.frame(i,1) = Data.discont.frame(i,2); % at start of mvt
%             else
%                 a=Data.discont.frame(i,1);
%                 b=Data.discont.frame(i,2);
%                 Data.discont.frame(i,1)=b;
%                 Data.discont.frame(i,2)=a;
%                 if Data.discont.frame(i,1)>Data.discont.frame(i,2)
%                     fprintf('\n Frames are out of order: trial %i \n',i)
%                 end
%             end
%         end
%         
%             dt = Data.discont.trials(i);
%             df = round(mean(Data.discont.frame(i,1:2)));
%             Data.x(df,dt) = mean(Data.x(Data.discont.frame(i,:),dt));
%             Data.y(df,dt) = mean(Data.y(Data.discont.frame(i,:),dt));
% 
%             vlength=sum(~isnan(Data.vy(:,Data.discont.trials(i))));
%             if (Data.discont.frame(i)-vlength) <= -11
%                 xi = Data.discont.frame(i,1)-8:Data.discont.frame(i,2)+8;
%                 Data.x(xi,dt) = interp1([xi(1) xi(end)], Data.x([xi(1) xi(end)],dt), xi);
%                 Data.y(xi,dt) = interp1([xi(1) xi(end)], Data.y([xi(1) xi(end)],dt), xi);
%                 
%                 Data.vx(xi,dt) = interp1([xi(1) xi(end)], Data.vx([xi(1) xi(end)],dt), xi);
%                 Data.vy(xi,dt) = interp1([xi(1) xi(end)], Data.vy([xi(1) xi(end)],dt), xi);
% 
%                 Data.fx(xi,dt) = interp1([xi(1) xi(end)], Data.fx([xi(1) xi(end)],dt), xi);
%                 Data.fy(xi,dt) = interp1([xi(1) xi(end)], Data.fy([xi(1) xi(end)],dt), xi);
%                 
%             elseif (Data.discont.frame(i)-vlength) <= -3
%                 xi = Data.discont.frame(i,1)-3:Data.discont.frame(i,2)+3;
%                 Data.x(xi,dt) = interp1([xi(1) xi(end)], Data.x([xi(1) xi(end)],dt), xi);
%                 Data.y(xi,dt) = interp1([xi(1) xi(end)], Data.y([xi(1) xi(end)],dt), xi);
%                 
%                 Data.vx(xi,dt) = interp1([xi(1) xi(end)], Data.vx([xi(1) xi(end)],dt), xi);
%                 Data.vy(xi,dt) = interp1([xi(1) xi(end)], Data.vy([xi(1) xi(end)],dt), xi);
% 
%                 Data.fx(xi,dt) = interp1([xi(1) xi(end)], Data.fx([xi(1) xi(end)],dt), xi);
%                 Data.fy(xi,dt) = interp1([xi(1) xi(end)], Data.fy([xi(1) xi(end)],dt), xi);
%                 
%             elseif (Data.discont.frame(i)-vlength) <= -1          
%                 xi = Data.discont.frame(i,1)-1:Data.discont.frame(i,2)+1;
%                 Data.x(xi,dt) = interp1([xi(1) xi(end)], Data.x([xi(1) xi(end)],dt), xi);
%                 Data.y(xi,dt) = interp1([xi(1) xi(end)], Data.y([xi(1) xi(end)],dt), xi);
%                 
%                 Data.vx(xi,dt) = interp1([xi(1) xi(end)], Data.vx([xi(1) xi(end)],dt), xi);
%                 Data.vy(xi,dt) = interp1([xi(1) xi(end)], Data.vy([xi(1) xi(end)],dt), xi);
% 
%                 Data.fx(xi,dt) = interp1([xi(1) xi(end)], Data.fx([xi(1) xi(end)],dt), xi);
%                 Data.fy(xi,dt) = interp1([xi(1) xi(end)], Data.fy([xi(1) xi(end)],dt), xi);
%             
%             end
%             
%     else
        for k=1:Data.discont.nspikes(i)
            
%             length(find(diff(Data.vy(:,Data.discont.trials(i)))>mean(range(diff(V)))*2))
%             Data.discont.nspikes(i)
            if length(find(diff(Data.vy(:,Data.discont.trials(i)))>mean(range(diff(V)))*2))==0
                break
            end
            
            Data.discont.frames = find(diff(V(:,Data.discont.trials(i)))>mean(range(diff(V)))*2,k);
            Data.discont.refframe(i,1) = Data.discont.frames(k);
%             
%             [minval Data.discont.frame(i,1)] = min(diff(V(Data.discont.refframe(i,1)-10:Data.discont.refframe(i,1)+10,Data.discont.trials(i))));
%             Data.discont.frame(i,1)=Data.discont.refframe(i,1)-Data.discont.frame(i,1)-2;
%             [maxval Data.discont.frame(i,2)] = max(diff(V(Data.discont.refframe(i,1)-10:Data.discont.refframe(i,1)+10,Data.discont.trials(i))));
%             Data.discont.frame(i,2)=Data.discont.refframe(i,1)-Data.discont.frame(i,2)+2;
%             
            Data.discont.frame(i,1)=Data.discont.refframe(i,1)-1;
            Data.discont.frame(i,2)=Data.discont.refframe(i,1)+1;
            
            % Sometimes the spike happens at the beginning/end of trial and it
            % can't find other side, so it picks a frame at the max of the movement
            % and results in beginning/end frames that are out of order.
            if Data.discont.frame(i,1)>Data.discont.frame(i,2)
                num_frames = sum(~isnan(V(:,Data.discont.trials(i))));
                if abs(Data.discont.frame(i,1) - num_frames)<=3
                    Data.discont.frame(i,2) = Data.discont.frame(i,1); % at end of mvt
                elseif Data.discont.frame(i,2) <=3
                    Data.discont.frame(i,1) = Data.discont.frame(i,2); % at start of mvt
                else
                    fprintf('\n Frames are out of order: trial %i \n',i)
                end
            end
            
                dt = Data.discont.trials(i);
                df = round(mean(Data.discont.frame(i,1:2)));
                Data.x(df,dt) = mean(Data.x(Data.discont.frame(i,:),dt));
                Data.y(df,dt) = mean(Data.y(Data.discont.frame(i,:),dt));
            
            vlength=sum(~isnan(Data.vy(:,Data.discont.trials(i))));    
            
            if (Data.discont.frame(i)-vlength) < -3
                xi = Data.discont.frame(i,1)-2:Data.discont.frame(i,2)+2;
                Data.x(xi,dt)=nan;Data.y(xi,dt)=nan;
                Data.x(xi,dt)=spline(1:length(Data.x(:,dt)),Data.x(:,dt),xi);
                [a,b]=lastwarn;
                warning('off',b);
                Data.y(xi,dt)=spline(1:length(Data.y(:,dt)),Data.y(:,dt),xi);
                
                Data.vx(xi,dt)=nan;Data.vy(xi,dt)=nan;
                Data.vx(xi,dt)=spline(1:length(Data.vx(:,dt)),Data.vx(:,dt),xi);                
                Data.vy(xi,dt)=spline(1:length(Data.vy(:,dt)),Data.vy(:,dt),xi);
                
                Data.fx(xi,dt)=nan;Data.fy(xi,dt)=nan;
                Data.fx(xi,dt)=spline(1:length(Data.fx(:,dt)),Data.fx(:,dt),xi);
                Data.fy(xi,dt)=spline(1:length(Data.fy(:,dt)),Data.fy(:,dt),xi);
                
            elseif (Data.discont.frame(i)-vlength) <= -1
                xi = Data.discont.frame(i,1)-1:Data.discont.frame(i,2)+1;
%                 linear interpolation = bad
                Data.x(xi,dt) = interp1([xi(1) xi(end)], Data.x([xi(1) xi(end)],dt), xi);                
                Data.y(xi,dt) = interp1([xi(1) xi(end)], Data.y([xi(1) xi(end)],dt), xi);
                
                Data.vx(xi,dt) = interp1([xi(1) xi(end)], Data.vx([xi(1) xi(end)],dt), xi);
                Data.vy(xi,dt) = interp1([xi(1) xi(end)], Data.vy([xi(1) xi(end)],dt), xi);

                Data.fx(xi,dt) = interp1([xi(1) xi(end)], Data.fx([xi(1) xi(end)],dt), xi);
                Data.fy(xi,dt) = interp1([xi(1) xi(end)], Data.fy([xi(1) xi(end)],dt), xi);
            
            end            
        end
%     end
%         clf(2);
%         figure(2);
%         hold on
%     plot(Data.x(:,Data.discont.trials(i)));
%     plot(Data.y(:,Data.discont.trials(i)));
%     a=sprintf('Condition: %g, Subject: %g, Trial %g',c,subj,Data.discont.trials(i));
%     xlabel(a);
end

% if ~isempty(Data.discont.trials)
%     disp('Trials with discontinuities: ');
%     disp(Data.discont.trials)
% %      figure('name','Discont', 'Position',[scrsz(3)*0.1 scrsz(4)*0.05 scrsz(3)*0.6 scrsz(4)*0.85])
% %      subplot(131), plot(Px, Py)
% %     xlabel('Px'), ylabel('Py'), title('All trials')
% %     
% %      subplot(132), plot(Px(:,discont.trials), Py(:,discont.trials)), legend(regexp(num2str(discont.trials), '\s*', 'split'))
% %    hold on
% %     for i = 1:length(discont.trials)
% %          plot(Px(discont.frame(i,1),discont.trials(i)),Py(discont.frame(i,1),discont.trials(i)), 'o')
% %          plot(Px(discont.frame(i,2),discont.trials(i)),Py(discont.frame(i,2),discont.trials(i)), 'x')
% %     end
% %     xlabel('Px'), ylabel('Py'), title('Discont trials'), legend(regexp(num2str(discont.trials), '\s*', 'split'))
% %     
% %      subplot(133), plot(V(:,discont.trials))
% %     ylabel('V')
% end
