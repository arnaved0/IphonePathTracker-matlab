
addpath lib


thisDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(thisDir, 'data');

if ~isfolder(dataDir)
    error('Data folder not found: %s. Run record_walk.m first.', dataDir);
end

D = dir(fullfile(dataDir, '*.mat'));
if isempty(D)
    error('No .mat logs in %s. Run record_walk.m to create one.', dataDir);
end

[~,idxNewest] = max([D.datenum]);                 
file = fullfile(D(idxNewest).folder, D(idxNewest).name);
fprintf('Loaded: %s\n', file);

S = load(file);                                
t = S.t; Fs = S.Fs; acc = S.acc_u; gyr = S.gyr_u; mag = S.mag_u;
dt = 1/Fs;


N = numel(t); q = [1;0;0;0]; state = [];
acc_z_world = zeros(N,1); yaw_arr = zeros(N,1);

for k = 2:N
    [q, state] = fusion_complementary(q, gyr(k,:).', acc(k,:).', mag(k,:).', dt, state);
    yaw_arr(k) = quat_utils('yaw', q);
    aw = quat_utils('rotate', q, acc(k,:).');
    acc_z_world(k) = aw(3);
end


steps = step_detect(acc_z_world, Fs);
fprintf('Detected %d steps\n', numel(steps));

pos = [0;0]; trail = pos; step_len = 0.70;   
for s = steps(:).'
    yaw = yaw_arr(s);
    pos = pos + step_len*[cos(yaw); sin(yaw)];
    trail(:,end+1) = pos; %#ok<AGROW>
end


figure;
subplot(2,1,1); plot(t, unwrap(yaw_arr)); ylabel('yaw (rad)'); grid on
subplot(2,1,2); plot(trail(1,:), trail(2,:), '-o'); axis equal; grid on
xlabel('x (m)'); ylabel('y (m)'); title(sprintf('2D path (%s)', D(idxNewest).name));
