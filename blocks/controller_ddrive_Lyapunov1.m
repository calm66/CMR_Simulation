% Nonlinear kinematic tracking control using Lyapunov method, without
% trajectory generator (1 point)
function ctrl = controller_ddrive_Lyapunov1(path)
%     if ~isnumeric(path) || ~isreal(path) || length(size(path)) ~= 2 || size(path, 2) ~= 2 || size(path, 1) < 2
%          error('Invalid format: path should be an Nx2 matrix (with N >= 2)');
%     end

    
    ctrl = block_base(1/50, 'platform', @control);
    
    ctrl.graphicElements(end + 1).draw = @drawPath;
    ctrl.graphicElements(end).name = 'Path';
    
    ctrl.figures(end + 1).name = 'controller variables Plot';
%     ctrl.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'covariance_icon.png');
    ctrl.figures(end).init = @createCtrlVarFigure;
    ctrl.figures(end).drawLog = @updateCtrlVarFigure;
    
    ctrl.default_color = [0 0 1];
    ctrl.default_vMax = 1;                  % max. translational velocity [m / s]
    ctrl.default_omegaMax = 90 * pi / 180;  % max. angular velocity [rad / s]
    ctrl.default_accVMax = 2;                  % max. translational acceleration/deceleration [m / s^2]
    ctrl.default_accOmegaMax = 180 * pi / 180; % max. angular acceleration/deceleration [rad / s^2]
    
    ctrl.default_firstTargetPoint = 1;
    ctrl.default_startBackwards = false;
    ctrl.default_K_x = 1;
    ctrl.default_K_phi = 1;
    
    function [f, diag] = createCtrlVarFigure(block, blockName)
        f = figure('Name', ['variables plots for ' blockName], 'NumberTitle', 'off');
        
        diag.axX = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 2/3, 0.5, 1/3]);
        diag.hXd = line('Parent', diag.axX, 'XData', [], 'YData', [], 'Color', 'r', 'LineStyle', '--', 'Marker','o','MarkerSize',2);
        diag.hX = line('Parent', diag.axX, 'XData', [], 'YData', [], 'Color', 'b');
        legend('x_d','x');
        xlabel('t');
        ylabel('x in m');
        
        diag.axY = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 1/3, 0.5, 1/3]);
        diag.hYd = line('Parent', diag.axY, 'XData', [], 'YData', [], 'Color', 'r', 'LineStyle', '--', 'Marker','o','MarkerSize',2);
        diag.hY = line('Parent', diag.axY, 'XData', [], 'YData', [], 'Color', 'b');
        legend('y_d','y');
        xlabel('t');
        ylabel('y in m');
        
        diag.axPhi = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 0, 0.5, 1/3]);
        diag.hPhid = line('Parent', diag.axPhi, 'XData', [], 'YData', [], 'Color', 'r', 'LineStyle', '--', 'Marker','o','MarkerSize',2);
        diag.hPhi = line('Parent', diag.axPhi, 'XData', [], 'YData', [], 'Color', 'b');
        legend('phi_d','phi');
        xlabel('t');
        ylabel('phi in degree');
        
        diag.axV = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0.5, 2/3, 0.5, 1/3]);
        diag.hVd = line('Parent', diag.axV, 'XData', [], 'YData', [], 'Color', 'r', 'LineStyle', '--', 'Marker','o','MarkerSize',2);
        diag.hVc= line('Parent', diag.axV, 'XData', [], 'YData', [], 'Color', 'b');
        legend('v_d','v_c');
        xlabel('t');
        ylabel('v in m/s');
        
        diag.axOmega = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0.5, 1/3, 0.5, 1/3]);
        diag.hOmegad = line('Parent', diag.axOmega, 'XData', [], 'YData', [], 'Color', 'r', 'LineStyle', '--', 'Marker','o','MarkerSize',2);
        diag.hOmegac = line('Parent', diag.axOmega, 'XData', [], 'YData', [], 'Color', 'b');
        legend('omega_d','omega_c');
        xlabel('t');
        ylabel('omega in degree/s');
        
        diag.axErr = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0.5, 0, 0.25, 1/3]);
        diag.hErrX = line('Parent', diag.axErr, 'XData', [], 'YData', [], 'Color', 'r');
        diag.hErrY = line('Parent', diag.axErr, 'XData', [], 'YData', [], 'Color', 'g');
        diag.hErrPhi = line('Parent', diag.axErr, 'XData', [], 'YData', [], 'Color', 'b');
        legend('x_e_r_r','y_e_r_r');
        xlabel('t');   
        ylabel('error in m');
        
        diag.axErr1 = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0.75, 0, 0.25, 1/3]);
        diag.hErrPhi = line('Parent', diag.axErr1, 'XData', [], 'YData', [], 'Color', 'b');
        legend('phi_e_r_r');
        xlabel('t');   
        ylabel('error in degree');
    end
    function diag = updateCtrlVarFigure(block, f, diag, logPos, t, out, debug, state)
        t = t(1:logPos);
        
        xdlog = cellfun(@(s)s.log(2), state(1:logPos));
        ydlog = cellfun(@(s)s.log(3), state(1:logPos));
        phidlog = cellfun(@(s)s.log(4), state(1:logPos));
        vdlog = cellfun(@(s)s.log(5), state(1:logPos));
        omegadlog = cellfun(@(s)s.log(6), state(1:logPos));
        
        xlog = cellfun(@(s)s.log(7), state(1:logPos));
        ylog = cellfun(@(s)s.log(8), state(1:logPos));
        philog = cellfun(@(s)s.log(9), state(1:logPos));
        
        vclog = cellfun(@(s)s.log(10), state(1:logPos));
        omegaclog = cellfun(@(s)s.log(11), state(1:logPos));
        
        xrlog = cellfun(@(s)s.log(12), state(1:logPos));
        yrlog = cellfun(@(s)s.log(13), state(1:logPos));
        phirlog = cellfun(@(s)s.log(14), state(1:logPos));        
             
        set(diag.hXd, 'XData', t, 'YData', xdlog);
        set(diag.hX, 'XData', t, 'YData', xlog);
        
        set(diag.hYd, 'XData', t, 'YData', ydlog);                        
        set(diag.hY, 'XData', t, 'YData', ylog);
        
        set(diag.hPhid, 'XData', t, 'YData', phidlog * 180 / pi);
        set(diag.hPhi, 'XData', t, 'YData', philog * 180 / pi);  
        
        set(diag.hVd, 'XData', t, 'YData', vdlog);
        set(diag.hVc, 'XData', t, 'YData', vclog);
        
        set(diag.hOmegad, 'XData', t, 'YData',omegadlog * 180 / pi);                        
        set(diag.hOmegac, 'XData', t, 'YData', omegaclog * 180 / pi);
        
        set(diag.hErrX, 'XData', t, 'YData', xrlog);
        set(diag.hErrY, 'XData', t, 'YData', yrlog)
        
        set(diag.hErrPhi, 'XData', t, 'YData', phirlog * 180 / pi);  
    end
    
    function handles = drawPath(block, ax, handles, ~, ~, state, varargin)   
        if isempty(handles); 
			handles.path = line('Parent', ax, 'XData', [], 'YData', [], 'Color', block.color, 'LineWidth', 1, 'Marker', '.');            
            handles.nextPoint = line('Parent', ax, 'XData', [], 'YData', [], 'Color', block.color, 'LineStyle', 'none', 'Marker', 's', 'MarkerSize', 8);
        end
        set(handles.path, 'XData', path(:, 1), 'YData', path(:, 2));
        pathPt = [];
        if ~isempty(state);
            if state.targetPointIndex <= size(path, 1)
                pathPt = path(state.targetPointIndex, :);
            end
        end
        if ~isempty(pathPt)
            set(handles.nextPoint, 'XData', pathPt(1), 'YData', pathPt(2));
        else set(handles.nextPoint, 'XData', [], 'YData', []);
        end
    end

    function [state, out, debugOut] = control(block, ~, state, input)
        debugOut = [];
        
        
        if isempty(state)
            if isempty(input)
                out = [0; 0];
                return;
            end
            
            % initialize state on the arrival of the first pose input
            state = struct();
            state.pose = input(end).data(:);
            state.simRun =true;
            state.targetPointIndex = min(size(path, 1), block.firstTargetPoint);
            targetPt = path(state.targetPointIndex, :);
            state.targetPose = [targetPt(1), targetPt(2), mod(atan2(targetPt(2) - state.pose(2), targetPt(1) - state.pose(1)) + pi, 2 * pi) - pi].';
            state.backwards = block.startBackwards;  
            state.firstStep = true;
            
        elseif ~isempty(input)
            state.pose = input(end).data(:);
        end
        if state.simRun == false
             out =[0;0];
             return;
            
        end
        
        pose = state.pose;            
        targetPose = state.targetPose;
        targetVec = [cos(targetPose(3)); sin(targetPose(3))];
        % determine distance of projected position on line-to-target
        % from next target point
        posOnLine = targetVec.' * [pose(1) - targetPose(1); pose(2) - targetPose(2)];

        if posOnLine >= -0.01 
            state.simRun = false;
        end
        
        tRelative =0;
        desiredIn = [path(1,1); path(1,2); path(1,3); 0; 0];        
        
        % lyapunov controller
        [vOmega_c, ctrlErr] = controllerLyn(block, state, desiredIn);
        % convert v/omega to [theta_R, theta_L]'
        if numel(block.wheelRadius) > 1
            R_right = block.wheelRadius(1);
            R_left = block.wheelRadius(2);
        else
            R_right = block.wheelRadius(1);
            R_left = block.wheelRadius(1);
        end
        out = [1 / R_right, 0.5 * block.wheelDistance / R_right; 1 / R_left, -0.5 * block.wheelDistance / R_left] * vOmega_c;
        
        % log data 
            state.log = [tRelative;desiredIn;state.pose;vOmega_c;ctrlErr];            
    end    


    function [outputC, pError] = controllerLyn(block, state, inputC)
        % input: [x_d; y_d; phi_d; v_d; omega_d]
        phi = state.pose(3);
        % control error, in case of different matrix form of state.pose,
        % sub in elementwise
        pError = inputC(1:3) - state.pose;
        % transform error to robot coordinate
        pError_r = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1]*pError;
        pError_r(3) = mod(pError_r(3) + pi, 2 * pi) - pi;
        % controller equtions
        v_c = block.K_x*pError_r(1) + inputC(4)*cos(pError_r(3));
        omega_c = block.K_phi*sin(pError_r(3)) + inputC(4)*pError_r(2) +inputC(5);
       
        outputC = [v_c; omega_c];
    end
end

