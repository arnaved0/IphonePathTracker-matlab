function [q_next, state] = fusion_complementary(q, gyr, acc, mag, dt, state)
% q: quaternion (4x1), gyr [rad/s], acc [m/s^2], mag [uT] OR [] if unavailable
% returns q_next (unit quaternion). 'state' carries low-pass filters.

if nargin < 6 || isempty(state)
    state.acc_lp = [0;0;9.81];
    state.mag_lp = [1;0;0];
end

% Gains
alpha_g = 0.02;   % gravity correction gain (0.01–0.05)
alpha_m = 0.01;   % magnetometer correction (0.005–0.03)

% If magnetometer missing, disable yaw correction
useMag = ~isempty(mag) && all(isfinite(mag));
if ~useMag, alpha_m = 0; end

beta_acc = 0.1;   % accel LP
beta_mag = 0.1;   % mag LP

% 1) integrate gyro
dq = quat_utils('fromSmallAngle', gyr(:)*dt);
q_pred = quat_utils('mul', q, dq);
q_pred = quat_utils('norm', q_pred);

% 2) low-pass acc (& mag if present)
state.acc_lp = (1-beta_acc)*state.acc_lp + beta_acc*acc(:);
if useMag
    state.mag_lp = (1-beta_mag)*state.mag_lp + beta_mag*mag(:);
end

% 3) rotate measurements to world
g_meas_w = quat_utils('rotate', q_pred, state.acc_lp);
g_ref = [0;0;9.81];
e_g = cross(g_meas_w, g_ref);

e_m = [0;0;0];
if useMag
    m_meas_w = quat_utils('rotate', q_pred, state.mag_lp);
    m_ref = [1;0;0];       % arbitrary horizontal reference
    % keep only horizontal components to avoid tilt coupling
    m_meas_w(3) = 0; m_ref(3) = 0;
    if norm(m_meas_w)>0, m_meas_w = m_meas_w/norm(m_meas_w); end
    e_m = cross(m_meas_w, m_ref);
end

% 4) small corrective rotation
e = alpha_g*e_g + alpha_m*e_m;
dq_corr = quat_utils('fromSmallAngle', -e);
q_next = quat_utils('mul', dq_corr, q_pred);
q_next = quat_utils('norm', q_next);
