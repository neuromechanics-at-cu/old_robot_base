function E = get_robotevents(framedata, statenames, avstatenames)

% outputs a cell array E that has the indices of the last frame in a given
% state, the number of that given state, and the names of that given state

E(:,1) = {'state'; 'av'; 'robot'};

for i = 1:length(framedata)
    E{1,i+1}{1} = find(abs(diff(framedata(i).statenumber)) > 0);
    E{2,i+1}{1} = find(abs(diff(framedata(i).avstatenumber)) > 0);
    E{3,i+1}{1} = find(abs(diff(framedata(i).robotstatenumber)) > 0);
    
    E{1,i+1}{2} = framedata(i).statenumber(E{1,i+1}{1});
    E{2,i+1}{2} = framedata(i).avstatenumber(E{2,i+1}{1});
    E{3,i+1}{2} = framedata(i).robotstatenumber(E{3,i+1}{1});
    
    E{1,i+1}{3} = statenames(E{1,i+1}{2});
    E{2,i+1}{3} = E{2,i+1}{2}; %avstatenames(E{2,i+1}{2});
    E{3,i+1}{3} = [];
end
