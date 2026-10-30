function traj = repeat_data(traj, n)

fs = fieldnames(traj);
for k = 1:n
    for i = 1:numel(fs)
        name = fs{i};
        if name == 't'
            traj.t = [traj.t; traj.t(end) + traj.t - traj.t(1) + traj.dt];
        elseif isscalar(traj.(name))
        else
            traj.(name) = [traj.(name); traj.(name)];
        end
    end
end