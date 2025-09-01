function steps = step_detect(acc_z, Fs)
% TOOLBOX-FREE step detector:
%  - crude 0.5–3 Hz band-pass (first-order sections below)
%  - rectified envelope
%  - local-max finder with threshold + refractory period
%
% acc_z : vertical acceleration (world z), column or row
% Fs    : sampling rate (Hz)
%
% returns:
%   steps : indices (samples) where steps are detected

x = acc_z(:);
x = x - mean(x);

% --- simple band-pass ~0.5–3 Hz (walk cadence) ---
[b1,a1] = butter1(0.5/(Fs/2), 'high');
[b2,a2] = butter1(3.0/(Fs/2), 'low');
x = filter(b1,a1, x);
env = filter(b2,a2, abs(x));     % rectified + smoothed

% --- local maxima with threshold & refractory ---
N = numel(env);
minDist = max(1, round(0.25*Fs));      % min time between steps (~0.25 s)
% threshold: mean + k * std (tune k if needed)
kthr = 0.5;                             % try 0.3–0.8
thr = mean(env) + kthr*std(env);

steps = zeros(0,1);
last = -Inf;

for i = 2:N-1
    if env(i) > thr && env(i) >= env(i-1) && env(i) >= env(i+1)
        if i - last >= minDist
            steps(end+1,1) = i; %#ok<AGROW>
            last = i;
        end
    end
end

% Optional: fall back if threshold too high (no steps found)
if isempty(steps)
    [~,imax] = max(env);
    if ~isempty(imax), steps = imax; end
end

end

% ------- helpers -------
function [b,a] = butter1(normCut, type)
% first-order IIR approximations (very lightweight)
if strcmpi(type,'high')
    % high-pass via mirrored low-pass
    k = tan(pi*(0.5 - normCut)/2);
    a0 = k + 1;
    b0 = [1, -1] / a0;
    a  = [1, (1-k)/a0];
    b  = b0;
else % 'low'
    k = tan(pi*normCut/2);
    a0 = k + 1;
    b  = [k, k] / a0;
    a  = [1, (k-1)/a0];
end
end
