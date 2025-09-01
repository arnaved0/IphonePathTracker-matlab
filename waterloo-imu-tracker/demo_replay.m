addpath lib


Fs = 50; dt = 1/Fs; N = 20*Fs;
acc = zeros(N,3); gyr = zeros(N,3); mag = repmat([30 0 0], N,1);
% gravity + small noise
acc(:,3) = 9.81 + 0.1*randn(N,1);
% a gentle yaw to the left between 8s..12s
for k=1:N
    t = (k-1)*dt;
    if t>8 && t<12, gyr(k,3) = 15*pi/180/4; end % 15° over 4 s
end


q = [1;0;0;0]; state = [];
pos = [0;0]; trail = pos;
step_len = 0.7; t_next = 0.7;
acc_z_world = zeros(N,1);

for k=2:N
    [q, state] = fusion_complementary(q, gyr(k,:).', acc(k,:).', mag(k,:).', dt, state);
    R = quat_utils('rotm', q);
    acc_w = R*acc(k,:).';
    acc_z_world(k) = acc_w(3);

    t = (k-1)*dt;
    if t >= t_next
        yaw = quat_utils('yaw', q);
        pos = pos + step_len * [cos(yaw); sin(yaw)];
        trail(:,end+1) = pos; %#ok<AGROW>
        t_next = t_next + 0.7 + 0.1*randn; % variable cadence
    end
end

figure; plot(trail(1,:), trail(2,:), '-o'); axis equal
grid on; xlabel('x (m)'); ylabel('y (m)'); title('Replay path')
