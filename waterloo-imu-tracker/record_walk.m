addpath lib
clear; clc

% -- create device handle
m = mobiledev;

% -- set sample rate if available (common: 50 Hz)
if isprop(m,'SampleRate')
    try
        m.SampleRate = 50;   % adjust if your phone struggles
    catch
        % some platforms lock SampleRate; it's fine to ignore
    end
end

% -- enable the sensors your class supports
enableIfPresent(m,'AccelerationSensorEnabled',true);
enableIfPresent(m,'AngularVelocitySensorEnabled',true);
enableIfPresent(m,'MagneticSensorEnabled',true);      % <-- your API
% (optional, not needed for our fusion)
% enableIfPresent(m,'OrientationSensorEnabled',true);
% enableIfPresent(m,'PositionSensorEnabled',true);

fprintf('Connected: %d | Fs=~%s Hz\n', m.Connected, string(tryGet(m,'SampleRate','?')));

% -- start logging
m.Logging = true; pause(0.5);
disp('Walk ~30–60 s with phone flat in hand; return here and press any key...');
pause;  % waits for keypress
m.Logging = false; pause(0.3);

% -- fetch logs
[acc, t_acc] = accellog(m);
[gyr, t_gyr] = angvellog(m);
[mag, t_mag] = magfieldlog(m);      % should work on your API

% quick sanity print
fprintf('acc:%d samples | gyr:%d | mag:%d\n', size(acc,1), size(gyr,1), size(mag,1));

% -- resample to uniform timeline
Fs = 50;   % processing rate
t0 = min([t_acc(1), t_gyr(1), t_mag(1)]);
t1 = max([t_acc(end), t_gyr(end), t_mag(end)]);
t  = (t0:1/Fs:t1).';

acc_u = interp1(t_acc, acc, t, 'linear','extrap');
gyr_u = interp1(t_gyr, gyr, t, 'linear','extrap') * pi/180; % deg/s -> rad/s
mag_u = interp1(t_mag, mag, t, 'linear','extrap');

% -- save to data/
if ~exist('data','dir'), mkdir data; end
fname = fullfile('data', ['walk_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(fname, 't','Fs','acc_u','gyr_u','mag_u');
fprintf('Saved %s\n', fname);

% ===== helpers =====
function enableIfPresent(obj, prop, val)
    if isprop(obj, prop)
        try
            obj.(prop) = val;
        catch ME
            warning('Could not set %s: %s', prop, ME.message);
        end
    end
end

function v = tryGet(obj, prop, fallback)
    if isprop(obj, prop)
        try, v = obj.(prop); return; end
    end
    v = fallback;
end
