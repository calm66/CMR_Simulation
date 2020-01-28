function guidance = guidance_gdwa2d()
    guidance = block_base('sensors/rangefinder', {'platform', 'pathplanner', 'sensors/rangefinder'}, @gdwaStep);
    
    % core DWA related parameters
    guidance.default_vMin = 0;                  % [m / s]
    guidance.default_vMax = 1.5;                % [m / s]
    guidance.default_omegaMin = -80 * pi / 180; % [rad / s]
    guidance.default_omegaMax = 80 * pi / 180;  % [rad / s]
    guidance.default_accV = 4;                  % [m / s^2]
    guidance.default_accOmega = 270 * pi / 180; % [rad / s^2]
    guidance.default_Np = 10;                   % subdivisions of path (from global planner)
    guidance.default_Nt = 30;                   % subdivisions of trajectory candidate
    guidance.default_lambda = 0.5;              % relative weight (lambda * dist_utility) + (1 - lambda) * path_alignment_utility    
    guidance.default_sensorRange = 4.5;         % [m]

    % obstacle line field related parameters
    guidance.default_safetyMargin = 0.05;
    guidance.default_radius = 0.1;    
    
    guidance.graphicElements(end + 1).draw = @drawObstacleLines;
    guidance.graphicElements(end).name = 'Obstacle Lines';
    guidance.graphicElements(end).hideByDefault = true;    
    guidance.graphicElements(end + 1).draw = @drawTrajCandidates;
    guidance.graphicElements(end).name = 'Candidate Trajectories';    
    guidance.graphicElements(end + 1).draw = @drawSelectedTrajectory;
    guidance.graphicElements(end).name = 'Selected Trajectory';    
    guidance.graphicElements(end + 1).draw = @drawEffectivePath;
    guidance.graphicElements(end).name = 'Effective Path';
    
    guidance.figures(end + 1).name = 'GDWA Internals';
    guidance.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'radar.png');
    guidance.figures(end).init = @createFigure;
    guidance.figures(end).draw = @updateFigure;  
    
    % drawing callbacks
    function handles = drawObstacleLines(block, ax, handles, out, debugOut, state, varargin)        
        if isempty(handles) 
            handles = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [1 0 1]);            
        end
        if ~isempty(debugOut)
            lineData = debugOut.obstacleLines;
            N_obst = size(lineData, 1);
            set(handles, 'XData', reshape([lineData(:, [1 3])'; NaN(1, N_obst)], 1, []), ...
                         'YData', reshape([lineData(:, [2 4])'; NaN(1, N_obst)], 1, []));        
        else set(handles, 'XData', [], 'YData', []);
        end
    end          
    
    function handles = drawTrajCandidates(block, ax, handles, out, debugOut, state, varargin)        
        if isempty(handles) 
            handles.curves = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.8 0]);            
            handles.centers = line('Parent', ax, 'XData', [], 'YData', [], 'Marker', '+', 'Color', get(handles.curves, 'Color'), 'LineStyle', 'none');            
            handles.minDist = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineWidth', 2);                    
            handles.trajPoints = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 1 0], 'Marker', 'x', 'LineStyle', 'none');
        end
        if ~isempty(debugOut)             
            N_candidates = numel(debugOut.omegas);
            curvePts = zeros(0, 2);
            centerPts = zeros(0, 2);  
            pose = debugOut.pose;
            for i = 1:N_candidates
                velocity = debugOut.velocities(i);
                omega = debugOut.omegas(i);

                if i > 1; curvePts = [curvePts; NaN, NaN]; end

                if omega ~= 0
                    % curve
                    m = pose(1:2) + velocity / omega * [-sin(pose(3)), cos(pose(3))];
                    r = abs(velocity / omega);                    
                    gamma_robot = pose(3) - sign(velocity / omega) * pi / 2;
                    t = gamma_robot + linspace(0, 2 * pi, min(2000, 200 * pi * r))';

                    curvePts = [curvePts; m(1) + r * cos(t), m(2) + r * sin(t)];
                    centerPts = [centerPts; m(1), m(2)];
                else
                    % straight line
                    curveLength = velocity * 10;
                    curvePts = [curvePts; pose(1) + [0; curveLength * cos(pose(3))], ...
                                          pose(2) + [0; curveLength * sin(pose(3))]];
                end            

            end

            set(handles.curves, 'XData', curvePts(:, 1), 'YData', curvePts(:, 2));
            set(handles.centers, 'XData', centerPts(:, 1), 'YData', centerPts(:, 2));        

            curvePts = zeros(0, 2);
            for i = 1:N_candidates
                if ~debugOut.admissibleCandidates(i)
                    minDist = debugOut.minDists(i);
                    minGamma = debugOut.minGammas(i);
                    velocity = debugOut.velocities(i);
                    omega = debugOut.omegas(i);
                    % collision
                    if size(curvePts, 1) > 0; curvePts = [curvePts; NaN, NaN]; end
                    if isnan(minGamma)
                        lineLength = sign(velocity) * minDist;
                        curvePts = [curvePts; pose(1) + [0; lineLength * cos(pose(3))], pose(2) + [0; lineLength * sin(pose(3))]];                        
                    else
                        gamma_robot = pose(3) - sign(velocity / omega) * pi / 2;
                        angles = gamma_robot + linspace(0, sign(omega) * minGamma, 20)';
                        m = pose(1:2) + velocity / omega * [-sin(pose(3)), cos(pose(3))];
                        r = abs(velocity / omega);
                        curvePts = [curvePts; m(1) + r * cos(angles), m(2) + r * sin(angles)];
                    end
                end                   
            end
            set(handles.minDist, 'XData', curvePts(:, 1), 'YData', curvePts(:, 2));            
            
            % recompute the sample points used for evaluating path
            % similarity measure
            accumulatedTrajPoints = zeros(N_candidates * block.Nt, 2);
            for i = 1:N_candidates
                accumulatedTrajPoints(i * block.Nt + (0:(block.Nt - 1)), :) = ...
                    computeTrajectory(pose, debugOut.velocities(i), debugOut.omegas(i), debugOut.T_max, block.Nt);
            end
            set(handles.trajPoints, 'XData', accumulatedTrajPoints(:, 1), 'YData', accumulatedTrajPoints(:, 2));
            
        else set([handles.curves, handles.centers, handles.minDist, handles.trajPoints], 'XData', [], 'YData', []);
        end
    end

    function handles = drawSelectedTrajectory(block, ax, handles, out, debugOut, state, varargin)
        if isempty(handles)
            handles.traj = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.5 0], 'LineWidth', 2);
            handles.points = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [1 0.5 0], 'Marker', 'x', 'MarkerSize', 7, 'LineStyle', 'none');
        end
        
        if ~isempty(debugOut)
            velocity = out(1);
            omega = out(2);
            pose = debugOut.pose;
            deltaT = state(1) - debugOut.prevState(1);
            if omega == 0
                dist = deltaT * velocity;
                set(handles.traj, 'XData', pose(1) + [0, dist * cos(pose(3))], 'YData', pose(2) + [0, dist * sin(pose(3))]);
            else
                m = pose(1:2) + velocity / omega * [-sin(pose(3)), cos(pose(3))];
                r = abs(velocity / omega);            
                angles = pose(3) - sign(velocity / omega) * pi / 2 + linspace(0, deltaT * omega, 20)';
                set(handles.traj, 'XData', m(1) + r * cos(angles), 'YData', m(2) + r * sin(angles));
            end
            
            selTrajPoints = computeTrajectory(pose, velocity, omega, debugOut.T_max, block.Nt);                            
            set(handles.points, 'XData', selTrajPoints(:, 1), 'YData', selTrajPoints(:, 2));
        else set([handles.traj, handles.points], 'XData', [], 'YData', []);
        end
    end
    function handles = drawEffectivePath(block, ax, handles, out, debugOut, state, varargin)        
        if isempty(handles)
            handles.line = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.5 0], 'Marker', '+');
            handles.minCircle = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.5 0], 'LineStyle', ':');
            handles.maxCircle = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.5 0], 'LineStyle', '--');                    
            handles.trajPoints = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 1 0], 'Marker', 'x', 'LineStyle', 'none');
        end
        if ~isempty(debugOut)
            set(handles.line, 'XData', debugOut.effectivePath(:, 1), 'YData', debugOut.effectivePath(:, 2));
            arcs = linspace(0, 2 *pi, 100);
            set(handles.minCircle, 'XData', debugOut.pose(1) + debugOut.effectiveRange(1) * cos(arcs), 'YData', debugOut.pose(2) + debugOut.effectiveRange(1) * sin(arcs));
            set(handles.maxCircle, 'XData', debugOut.pose(1) + debugOut.effectiveRange(2) * cos(arcs), 'YData', debugOut.pose(2) + debugOut.effectiveRange(2) * sin(arcs));
        else
            set([handles.line, handles.minCircle, handles.maxCircle, handles.trajPoints], 'XData', [], 'YData', []);
        end
    end

    % callbacks for the 'Internals' visualization window
    function [f, diag] = createFigure(block, blockName)
        f = figure('Name', ['GDWA Internals for ' blockName], 'NumberTitle', 'off');
        diag.axVelocity = subplot(2, 3, [1 4]);
        set(diag.axVelocity, 'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', ...
                             'XLim', ([block.omegaMin block.omegaMax] * 180 / pi) + [-10 10], ...
                             'YLim', [block.vMin - 0.1, block.vMax + 0.1]);
        xlabel('omega [deg/s]');
        ylabel('velocity [m/s]');
        title('Dynamic Window');
        rectangle('Parent', diag.axVelocity, 'FaceColor', 0.8 * [1 1 1], 'EdgeColor', 0.2 * [1 1 1], ...
                  'Position', [block.omegaMin * 180 / pi, block.vMin, (block.omegaMax - block.omegaMin) * 180 / pi, block.vMax - block.vMin]);
        
        diag.dynWindowRect = rectangle('Position', [0 0 eps eps], 'FaceColor', [1 1 1], 'EdgeColor', [0 0 0]);
        diag.candidateMarkers = line('XData', [], 'YData', [], 'Marker', '.', 'Color', [0 0.8 0], 'LineStyle', 'none');
        diag.invalidCandidateMarkers = line('XData', [], 'YData', [], 'Marker', '.', 'Color', [1 0 0], 'LineStyle', 'none');
        diag.velocityMarker = line('XData', [], 'YData', [], 'Marker', 'x', 'MarkerSize', 15, 'Color', [0 1 1], 'LineWidth', 2);        
        diag.selectedVelocityMarker = line('XData', [], 'YData', [], 'Marker', 'o', 'MarkerSize', 10, 'Color', 0.5 * [0 1 1], 'LineWidth', 2);
                
        diag.axDistUtility = subplot(2, 3, 2);
        diag.distSurf = surf('XData', [], 'YData', [], 'ZData', [], 'CData', []);
        title('Distance Utility');
        set(diag.axDistUtility , 'ZLim', [0 1], 'ZLimMode', 'manual');
        
        diag.axPathUtility = subplot(2, 3, 3);
        diag.pathSurf = surf('XData', [], 'YData', [], 'ZData', [], 'CData', []);
        title('Path Alignment utility');
        set(diag.axPathUtility, 'ZLim', [0 1], 'ZLimMode', 'manual');
        
        diag.axUtilitySum = subplot(2, 3, [5 6]);
        diag.sumSurf = surf('XData', [], 'YData', [], 'ZData', [], 'CData', []);
        title('Summed Utility');
        set(diag.axUtilitySum, 'ZLim', [0 1], 'ZLimMode', 'manual');
        diag.maxUtilityMarker = line('XData', [], 'YData', [], 'ZData', [], 'Color', [1 0 0], 'LineWidth', 2);
    end
    function diag = updateFigure(block, f, diag, out, debugOut, state, varargin)
        if ~isempty(debugOut)            
            set(diag.distSurf, 'XData', debugOut.omegas * 180 / pi, 'YData', debugOut.velocities, 'ZData', debugOut.utility_dist);                    
            set(diag.pathSurf, 'XData', debugOut.omegas * 180 / pi, 'YData', debugOut.velocities, 'ZData', debugOut.utility_path);            
            set(diag.sumSurf, 'XData', debugOut.omegas * 180 / pi, 'YData', debugOut.velocities, 'ZData', debugOut.sumUtility);            
            
            set(diag.maxUtilityMarker, 'XData', out(2) * 180 / pi * [1 1], 'YData', out(1) * [1 1], 'ZData', [0 1]);            

            minOmega = debugOut.omegas(1, 1);
            maxOmega = debugOut.omegas(1, end);
            minV = debugOut.velocities(1, 1);
            maxV = debugOut.velocities(end, 1);
            set(diag.dynWindowRect, 'Position', [minOmega * 180 / pi, minV, (maxOmega - minOmega) * 180 / pi + eps, maxV - minV + eps]);                                    
            
            set(diag.invalidCandidateMarkers, 'XData', debugOut.omegas(~debugOut.admissibleCandidates) * 180 / pi, ...
                                              'YData', debugOut.velocities(~debugOut.admissibleCandidates));
            set(diag.candidateMarkers, 'XData', debugOut.omegas(debugOut.admissibleCandidates) * 180 / pi, ...
                                       'YData', debugOut.velocities(debugOut.admissibleCandidates));                    
                                   
            set(diag.velocityMarker, 'XData', debugOut.prevState(3) * 180 / pi, 'YData', debugOut.prevState(2));                                   
            set(diag.selectedVelocityMarker, 'XData', out(2) * 180 / pi, 'YData', out(1));                        
        else
            set([diag.distSurf, diag.pathSurf, diag.sumSurf, diag.maxUtilityMarker], 'XData', [], 'YData', [], 'ZData', []);
            set([diag.invalidCandidateMarkers, diag.candidateMarkers, diag.velocityMarker, diag.selectedVelocityMarker], 'XData', [], 'YData', []);
            set(diag.dynWindowRect, 'Position', [0 0 eps eps]);
        end
    end


    % Compute GDWA output
    % debugOut contains most of the drawing data
    function [state, out, debugOut] = gdwaStep(block, t, state, platform, path, rangefinder)
        if isempty(state)
            % state stores [t, v, omega]
            state = [t 0 0];
        end
        
        if ~isempty(platform) % there might be a circular references between this module and the platform, which can cause DWA to be compute before an actual pose has been initialized
            deltaT = t - state(1);
            debugOut.prevState = state;
            
            velocity = state(2);
            omega = state(3);
            minV = max(velocity - deltaT * block.accV, block.vMin);
            maxV = min(velocity + deltaT * block.accV, block.vMax);
            minOmega = max(omega - deltaT * block.accOmega, block.omegaMin);
            maxOmega = min(omega + deltaT * block.accOmega, block.omegaMax);        

            [omegas, velocities] = meshgrid(linspace(minOmega, maxOmega, 7), ...
                                            linspace(minV, maxV, 5));

            omegas(abs(omegas) < 1e-4) = 0;
            v_omegas = [reshape(velocities, [], 1), reshape(omegas, [], 1)];
            debugOut.velocities = velocities;
            debugOut.omegas = omegas;            
            
            % compute obstacle line field
            pose = platform(end).data;
            debugOut.pose = pose;
            safetyRadius = block.radius + block.safetyMargin;

            rayData = rangefinder(end).data;
            rayData.range = rayData.range - safetyRadius;

            N_obst = size(rayData.range, 1);
            rayVect = [rayData.range .* cos(pose(3) + rayData.bearing), ...
                       rayData.range .* sin(pose(3) + rayData.bearing)];
            ptCenter = repmat(pose(1:2), N_obst, 1) + rayVect;
            rayNormal = [-rayVect(:, 2), rayVect(:, 1)] ./ repmat(sqrt(rayVect(:, 1) .* rayVect(:, 1) + rayVect(:, 2) .* rayVect(:, 2)), 1, 2);

            deltaAngles = tan(rayData.bearing(2:end) - rayData.bearing(1:(end - 1)));
            deltaAngles = [deltaAngles(1); deltaAngles; deltaAngles(end)];

            lineData = repmat(ptCenter, 1, 2) + [repmat(safetyRadius + 0.5 * rayData.range .* deltaAngles(1:(end - 1)), 1, 2) .* rayNormal, ...
                                                 -repmat(safetyRadius + 0.5 * rayData.range .* deltaAngles(2:end), 1, 2) .* rayNormal];
            lineData = lineData(rayData.range < inf, :);
            debugOut.obstacleLines = lineData;  

            N_candidates = size(v_omegas, 1);
            minDists = zeros(N_candidates, 1);
            minGammas = zeros(N_candidates, 1);
            admissibleCandidates = false(N_candidates, 1);
            for i = 1:N_candidates
                [minDists(i), minGammas(i)] = lineFieldMinDist(pose, v_omegas(i, :), lineData);                
                % check if this v/omega pair is admissible, i.e. it does not
                % inevitable lead to a collision
                dist = minDists(i) - v_omegas(i, 1) * deltaT;
                if dist < 0; dist = 0; end
                admissibleCandidates(i) = (v_omegas(i, 1) < sqrt(2 * dist * block.accV)) & (v_omegas(i, 2) < sqrt(2 * dist * block.accOmega));
            end
            debugOut.minDists = minDists;
            debugOut.minGammas = minGammas;
            debugOut.admissibleCandidates = admissibleCandidates;            

            % Cost calculation for DWA
            admissibleMatrix = reshape(admissibleCandidates, size(velocities));
            T_max = block.sensorRange / block.vMax;
            debugOut.T_max = T_max;

            % 1st: distance costs
            t_col = reshape(minDists, size(velocities)) ./ velocities;
            T_b = max(cat(3, velocities / block.accV, omegas / block.accOmega), [], 3);
            utility_dist = (t_col - T_b) ./ (T_max - T_b);
            utility_dist(t_col <= T_b) = 0;
            utility_dist(t_col > T_max) = 1;
            utility_dist(~admissibleMatrix) = 0;
            debugOut.utility_dist = utility_dist;
            
            % 2nd: path alignment utility
            utility_path = zeros(size(velocities));
            debugOut.effectiveRange = [0 0];
            if ~isempty(path)
                path = path(end).data;
                if ~isempty(path)
                    % calculate points on "Effective Path"
                    R_max = (velocity + block.accV * deltaT) * T_max;
                    R_min = 2 * 0.5 * block.vMax * block.vMax / block.accV;
                    debugOut.effectiveRange = [R_min, R_max];
                    if R_min > R_max; R_min = R_max; end % this "saturation" is not clearly stated in the paper but they first
                                                         % describe the upper-bounding criterion and only after
                                                         % that the lower bounding
                    for i = 2:size(path, 1)
                        pathPt = path(i, :);
                        distPt = norm(pose(1:2) - pathPt);
                        if distPt >= R_min
                            rLimit = NaN;
                            if distPt > R_max; rLimit = R_max;
                            elseif i > 3; rLimit = R_min;
                            end
                            if ~isnan(rLimit)
                                p_m_r = path(i - 1, :) - pose(1:2);
                                v = path(i, :) - path(i - 1, :);
                                v_square_inv = 1 / (v(1) * v(1) + v(2) * v(2));
                                p = 2 * (p_m_r(1) * v(1) + p_m_r(2) * v(2)) * v_square_inv;
                                q = (p_m_r(1) * p_m_r(1) + p_m_r(2) * p_m_r(2) - rLimit * rLimit) * v_square_inv;
                                Det = p * p / 4 - q;                    
                                if Det >= 0; 
                                    dist = -p / 2 + [-1 1] * sqrt(Det);
                                    if dist(2) >= 0 && dist(2) <= 1; dist = dist(2);
                                    elseif dist(1) >= 0 && dist(1) <= 1; dist = dist(1);
                                    else dist = 0;
                                    end
                                else dist = 0;
                                end
                                pathPt = path(i - 1, :) + dist * v;
                                break;
                            %elseif i >= 2;
                            %   if norm(pathPt - pose(1:2)) > R_min; break; end
                            elseif i >= 3; 
                                break;
                            end
                        end
                    end
                    vEffectivePath = 1.5 * (pathPt - pose(1:2));
                    effectivePathPoints = repmat(pose(1:2), block.Np, 1) + repmat(linspace(0, 1, block.Np)', 1, 2) .* repmat(vEffectivePath, block.Np, 1);
                    debugOut.effectivePath = effectivePathPoints;
                    
                    % calculate alignment measure
                    D_sum = zeros(N_candidates, 1);
                    accumulatedTrajPoints = zeros(N_candidates * block.Nt, 2);
                    for i = 1:N_candidates
                        trajPoints = computeTrajectory(pose, v_omegas(i, 1), v_omegas(i, 2), T_max, block.Nt);
                        accumulatedTrajPoints(i * block.Nt + (0:(block.Nt - 1)), :) = trajPoints;

                        for k = 1:block.Nt
                            d_kj = effectivePathPoints - repmat(trajPoints(k, :), block.Np, 1);
                            D_sum(i) = D_sum(i) + sum((1:block.Np) .* sqrt(d_kj(:, 1) .* d_kj(:, 1) + d_kj(:, 2) .* d_kj(:, 2))');
                        end
                    end
                    debugOut.trajPoints = accumulatedTrajPoints;

                    D_min = min(D_sum);
                    D_max = max(D_sum);
                    utility_path = reshape(1 - (D_sum - D_min) / (D_max - D_min), size(velocities));
                end           
            end
            
            debugOut.utility_path = utility_path;
            
            % create summed utility function
            sumUtility = block.lambda * utility_dist + (1 - block.lambda) * utility_path;
            %sumUtility = imfilter(sumUtility, fspecial('gaussian',[2 2],2), 'replicate', 'same');        
            % clip non-admissible velocities
            sumUtility(~admissibleMatrix) = 0;
            debugOut.sumUtility = sumUtility;
                        
            [maxSumUtilityColumns, maxIndicesSumUtilityColumns] = max(sumUtility);
            [optUtility, optOmegaIndex] = max(maxSumUtilityColumns);
            if optUtility > 0
                optVIndex = maxIndicesSumUtilityColumns(optOmegaIndex);
                velocity = velocities(optVIndex, optOmegaIndex);
                omega = omegas(optVIndex, optOmegaIndex);
            else
                fprintf('No admissible velocity - applying maximum deceleration\n');
                velocity = sign(velocity) * max(0, abs(velocity) - block.accV * deltaT);
                omega = sign(omega) * max(0, abs(omega) - block.accOmega * deltaT);
                debugOut.selectedTrajPoints = zeros(0, 2);
            end
            
            state = [t, velocity, omega];            
        else debugOut = [];
        end
        out = state(2:3);
    end    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function points = computeTrajectory(pose, v, omega, T, N)
    if v ~= 0
        L_t = T * v;
        if omega ~= 0
            r = abs(v / omega);
            gamma_robot = pose(3) - sign(v / omega) * pi / 2;
            m = pose(1:2) + v / omega * [-sin(pose(3)), cos(pose(3))];
            gammas = gamma_robot + linspace(0, sign(omega) * L_t / r, N)';
            points = repmat(m, N, 1) + r * [cos(gammas), sin(gammas)];
        else
            r = sign(v) * linspace(0, L_t, N)';
            points = repmat(pose(1:2), N, 1) + [r * cos(pose(3)), r * sin(pose(3))];
        end
    else points = repmat(pose(1:2), N, 1);
    end   
end

function [minDist, minGamma] = lineFieldMinDist(pose, v_omega, lines)
    minDist = inf;
    minGamma = NaN;

    if abs(v_omega(1)) == 0; return; end
    
    if abs(v_omega(2)) == 0
        % check for collisions between straight trajectory and obstacle lines
        robot_dir = [cos(pose(3)), sin(pose(3))];
        
        for i = 1 : size(lines, 1)
            line_start = lines(i, 1:2);
            line_dir = lines(i, 3:4) - line_start;
            
            dir_cross = robot_dir(1) * line_dir(2) - robot_dir(2) * line_dir(1);
            if dir_cross ~= 0
                inv_dir_cross = 1 / dir_cross;
                
                pos_m_start = pose(1:2) - line_start;
                t = inv_dir_cross * (robot_dir(1) * pos_m_start(2) - robot_dir(2) * pos_m_start(1));
                t_robot = inv_dir_cross * (line_dir(1) * pos_m_start(2) - line_dir(2) * pos_m_start(1));
                if t >= 0 && t <= 1 && sign(t_robot) == sign(v_omega(1))
                    dist = norm(line_start + t * line_dir -pose(1:2));
                    if dist < minDist
                        minDist = dist;
                        minGamma = NaN;
                    end
                end                
            end            
        end
        
    else
        % check for collisions between curve and obstacle lines
        r = v_omega(1) / v_omega(2);
        m = pose(1:2) + r * [-sin(pose(3)), cos(pose(3))];
        gamma_robot = pose(3) - sign(r) * pi / 2;
        r = abs(r);

        for i = 1:size(lines, 1)
            line_start = lines(i, 1:2);
            line_dir = lines(i, 3:4) - line_start;
            inv_line_dir_square = 1 / (line_dir * line_dir');
            p = 2 * ((line_start - m) * line_dir') * inv_line_dir_square;
            q = ((line_start - m) * (line_start - m)' - r * r) * inv_line_dir_square;
            
            Det = p * p / 4 - q;
            if Det > 0
                allT = -p / 2 + [-1 1] * sqrt(Det);
                for t = allT(allT >= 0 & allT <= 1)
                    pISect = line_start + t * line_dir;
                    gamma = sign(v_omega(2)) * (atan2(pISect(2) - m(2), pISect(1) - m(1)) - gamma_robot);
                    if gamma < 0; gamma = gamma + 2 * pi;
                    elseif gamma > 2 * pi; gamma = gamma - 2 * pi;
                    end
                    dist = r * gamma;
                    if dist < minDist                        
                        minDist = dist;
                        minGamma = gamma;
                    end
                end                
            end
        end
    end
end