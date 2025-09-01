classdef app1 < matlab.apps.AppBase
 
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        StatusLabel       matlab.ui.control.Label
        StepLenField      matlab.ui.control.NumericEditField
        SteplengthmLabel  matlab.ui.control.Label
        ReplayButton      matlab.ui.control.Button
        StopButton        matlab.ui.control.Button
        StartButton       matlab.ui.control.Button
        ax2d              matlab.ui.control.UIAxes
        ax3d              matlab.ui.control.UIAxes
    end


    properties (Access = private)
        m           
        timerObj    
        Fs = 50     
        q = [1;0;0;0]     
        fState           
        yaw_arr double = [] 
        accZ_arr double = []
        pos double = [0;0]  
        trail double = [0;0]
    end

    
    methods (Access = private)

        
        function startupFcn(app)
            addpath('lib');
            app.pos = [0;0]; app.trail = app.pos;
            app.yaw_arr = []; app.accZ_arr = [];
            cla(app.ax2d); grid(app.ax2d,'on'); axis(app.ax2d,'equal');
            xlabel(app.ax2d,'x (m)'); ylabel(app.ax2d,'y (m)');
            app.StepLenField.Value = 0.70;
            app.StatusLabel.Text = 'Idle';
        end

        
function StartButtonPushed(app, ~)
try
    addpath('lib');
    app.StatusLabel.Text = 'Connecting...';

    
    try
        app.m = mobiledev;
    catch ME
        if contains(ME.message, 'already exists', 'IgnoreCase', true)
       
            try
                app.m = evalin('base','m');   
            catch
     
                evalin('base','clearvars m');
                app.m = mobiledev;
            end
        else
            rethrow(ME);
        end
    end

    
    app.m.AccelerationSensorEnabled = true;
    app.m.AngularVelocitySensorEnabled = true;
    app.m.MagneticFieldSensorEnabled   = true;
    pause(0.3);
    app.m.Logging = true;

   
    app.q = [1;0;0;0]; app.fState = [];
    app.yaw_arr = []; app.accZ_arr = [];
    resetPlots(app);

 
    if ~isempty(app.timerObj) && isvalid(app.timerObj)
        stop(app.timerObj); delete(app.timerObj);
    end
    app.timerObj = timer('ExecutionMode','fixedSpacing','Period',0.05, ...
                         'TimerFcn', @(~,~) liveTick(app));
    start(app.timerObj);

    app.StatusLabel.Text = 'Live (streaming)';
catch ME
    app.StatusLabel.Text = ['Error: ' ME.message];
end
end

        
        function StopButtonPushed(app, ~)
            try
                if ~isempty(app.m), app.m.Logging = false; end
                if ~isempty(app.timerObj) && isvalid(app.timerObj)
                    stop(app.timerObj); delete(app.timerObj);
                end
                app.StatusLabel.Text = 'Stopped';
            catch ME
                app.StatusLabel.Text = ['Error: ' ME.message];
            end
        end

        
        function ReplayButtonPushed(app, ~)
            try
                addpath('lib');
                [f,p] = uigetfile('data/*.mat','Pick a walk log');
                if isequal(f,0), return; end
                S = load(fullfile(p,f));
                t=S.t; Fs=S.Fs; acc=S.acc_u; gyr=S.gyr_u; mag=S.mag_u; dt=1/Fs;

                app.q=[1;0;0;0]; app.fState=[];
                app.pos=[0;0]; app.trail=app.pos;
                resetPlots(app);

                N = numel(t);
                accZ = zeros(N,1); yaw = zeros(N,1);
                for k=2:N
                    [app.q, app.fState] = fusion_complementary(app.q, gyr(k,:).', acc(k,:).', mag(k,:).', dt, app.fState);
                    yaw(k) = quat_utils('yaw', app.q);
                    aw = quat_utils('rotate', app.q, acc(k,:).');
                    accZ(k) = aw(3);
                end
                steps = step_detect(accZ, Fs);
                L = app.StepLenField.Value;
                for idx = steps(:).'
                    app.pos = app.pos + L*[cos(yaw(idx)); sin(yaw(idx))];
                    app.trail(:,end+1) = app.pos; %#ok<AGROW>
                end
                plot(app.ax2d, app.trail(1,:), app.trail(2,:), '-o');
                title(app.ax2d, sprintf('Replay: %s (steps=%d)', f, numel(steps)));
                app.StatusLabel.Text = 'Replay done';
            catch ME
                app.StatusLabel.Text = ['Replay error: ' ME.message];
            end
        end

       
        function resetPlots(app)
            cla(app.ax2d); grid(app.ax2d,'on'); axis(app.ax2d,'equal');
            xlabel(app.ax2d,'x (m)'); ylabel(app.ax2d,'y (m)');
            app.pos=[0;0]; app.trail=app.pos;
            plot(app.ax2d, app.trail(1,:), app.trail(2,:), '-o');
        end

       
        function liveTick(app)
            try
                [acc, t_acc] = accellog(app.m);
                [gyr, t_gyr] = angvellog(app.m);
                [mag, t_mag] = magfieldlog(app.m);
                if isempty(t_acc) || isempty(t_gyr) || isempty(t_mag), return; end

                t0 = max([t_acc(end), t_gyr(end), t_mag(end)]) - 1.0;
                acc = acc(t_acc>=t0,:); t_acc = t_acc(t_acc>=t0);
                gyr = gyr(t_gyr>=t0,:); t_gyr = t_gyr(t_gyr>=t0);
                mag = mag(t_mag>=t0,:); t_mag = t_mag(t_mag>=t0);

                t = (t0:1/app.Fs:max([t_acc; t_gyr; t_mag])).';
                if numel(t) < 2, return; end
                acc_u = interp1(t_acc, acc, t, 'linear','extrap');
                gyr_u = interp1(t_gyr, gyr, t, 'linear','extrap')*pi/180;
                mag_u = interp1(t_mag, mag, t, 'linear','extrap');

                for k=2:numel(t)
                    dt = 1/app.Fs;
                    [app.q, app.fState] = fusion_complementary(app.q, gyr_u(k,:).', acc_u(k,:).', mag_u(k,:).', dt, app.fState);
                    yaw = quat_utils('yaw', app.q);
                    aw = quat_utils('rotate', app.q, acc_u(k,:).');
                    app.accZ_arr(end+1,1) = aw(3); %#ok<AGROW>
                    app.yaw_arr(end+1,1)  = yaw;   %#ok<AGROW>
                end

                Fs = app.Fs; L = app.StepLenField.Value;
                seg = app.accZ_arr(max(1, numel(app.accZ_arr)-3*Fs+1):end);
                steps_local = step_detect(seg, Fs);
                if ~isempty(steps_local)
                    offset = numel(app.accZ_arr) - numel(seg);
                    persistent lastIdx
                    if isempty(lastIdx), lastIdx = 0; end
                    for s = steps_local(:).'
                        idx = offset + s;
                        if idx > lastIdx
                            yaw_s = app.yaw_arr(idx);
                            app.pos = app.pos + L*[cos(yaw_s); sin(yaw_s)];
                            app.trail(:,end+1) = app.pos; %#ok<AGROW>
                            plot(app.ax2d, app.trail(1,end-1:end), app.trail(2,end-1:end), '-o');
                            drawnow limitrate
                            lastIdx = idx;
                        end
                    end
                end

                app.StatusLabel.Text = 'Live (OK)';
            catch ME
                app.StatusLabel.Text = ['Live error: ' ME.message];
            end
        end
    end

 
    methods (Access = public)

     
        function app = app1
            createComponents(app)
            registerApp(app, app.UIFigure)
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

      
        function delete(app)
            try
                if ~isempty(app.m), app.m.Logging = false; end
            end
            try
                if ~isempty(app.timerObj) && isvalid(app.timerObj)
                    stop(app.timerObj); delete(app.timerObj);
                end
            end
            delete(app.UIFigure)
        end
    end

  
    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure('Visible','off');
            app.UIFigure.Position = [100 100 700 500];
            app.UIFigure.Name = 'Phone IMU Tracker';

            app.ax2d = uiaxes(app.UIFigure);
            title(app.ax2d,'2D Path');
            xlabel(app.ax2d,'x (m)'); ylabel(app.ax2d,'y (m)');
            app.ax2d.Position = [350 50 320 420];

            app.ax3d = uiaxes(app.UIFigure);
            title(app.ax3d,'Orientation');
            app.ax3d.Position = [20 280 300 200];

            app.StartButton = uibutton(app.UIFigure,'push','Text','Start Live',...
                'Position',[20 230 100 25],'ButtonPushedFcn',@(src,event)StartButtonPushed(app));
            app.StopButton = uibutton(app.UIFigure,'push','Text','Stop',...
                'Position',[140 230 100 25],'ButtonPushedFcn',@(src,event)StopButtonPushed(app));
            app.ReplayButton = uibutton(app.UIFigure,'push','Text','Replay Log',...
                'Position',[20 180 100 25],'ButtonPushedFcn',@(src,event)ReplayButtonPushed(app));

            app.SteplengthmLabel = uilabel(app.UIFigure,'Text','Step length (m)',...
                'Position',[20 140 100 22]);
            app.StepLenField = uieditfield(app.UIFigure,'numeric','Position',[140 140 80 22],...
                'Value',0.70);

            app.StatusLabel = uilabel(app.UIFigure,'Text','Idle',...
                'Position',[20 470 200 22]);

            app.UIFigure.Visible = 'on';
        end
    end
end
