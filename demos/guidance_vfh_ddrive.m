function guidance = guidance_vfh_ddrive()
    guidance = block_base('sensors/rangefinder', {'relativeGoal', 'sensors/rangefinder', 'sensors/odometer'}, @vfhStep);

    % core VFH related parameters (see paper for their definition)
    guidance.default_mapResolution = 0.1;
    guidance.default_mapRadius = 2;
    guidance.default_sectorCount = 72;
    guidance.default_maxSectors = 9;
    guidance.default_threshold = [0.5, 0.6]; % tau_low, tau_high
    guidance.default_obstacleInfluenceRange = 0.75;%1.5; % obstacles beyond this distance are barely considered (scaling parameter for arg of exp(-arg))
    guidance.default_filterLength = 1; % length of the crude smoothing filter from VFH paper (1 equals no filtering)
    guidance.default_maxVelocity = 0.7; % in m/s
    guidance.default_minVelocity = 0.04; % in m/s
    guidance.default_maxOmega = 120 * pi / 180; % in rad/s
    guidance.default_h_m = 1e4;
    guidance.default_radius = 0.15;
    guidance.default_color = [0 0 1];
    guidance.default_safetyDistance = 0.05;
    guidance.default_sensorUncertainty = 0.7;
    guidance.default_weights = [1 0.5 0.4]; % mu_1...3 from VFH+ paper
    
    %guidance.default_deltaT = 0.1; % propagation duration for one tree level of VFH*
    % guidance.default_maxLevels = 3; % VFH* parameter
    
    % vehicle parameter
    guidance.default_wheelRadius = 0.025;
    guidance.default_wheelDistance = 0.2;
    
    % Visualization of the VFH* tree
%     guidance.graphicElements(end + 1).draw = @drawLocalMapOverlay;
%     guidance.graphicElements(end).name = 'Candidate Tree';
%     guidance.graphicElements(end).hideByDefault = true;
    
    guidance.figures(end + 1).name = 'VFH internals';
    guidance.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'radar.png');
    guidance.figures(end).init = @createFigure;
    guidance.figures(end).draw = @updateFigure;
    
    guidance.log.uniform = true;
    
    % drawing callbacks
    function handles = drawLocalMapOverlay(block, ax, handles, out, debugOut, state, goal, rangefinder, odometer)        
%         if isempty(handles)             
%             handles = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.6 0]);
%             %handles = ...
%         end
%         
%         if ~isempty(poseProvider)
%             pose = poseProvider(end).data;            
%             points = zeros(2, 0);
%             for i = 1:numel(debugOut.candidates)
%                 vOmega = debugOut.candidates(i).command;
%                 if vOmega(2) < 0
%                     r = vOmega(1) / -vOmega(2);
%                     arcs = linspace(0, -vOmega(2) * block.deltaT, 8);
%                     pts = [r * sin(arcs); r * (cos(arcs) - 1)];
%                 elseif vOmega(2) > 0
%                     r = vOmega(1) / vOmega(2);
%                     arcs = linspace(0, vOmega(2) * block.deltaT, 8);
%                     pts = [r * sin(arcs); r * (1 - cos(arcs))];
%                 else
%                     pts = [0 0; block.deltaT * vOmega(1), 0].';
%                 end
%                     
%                 points = [points, NaN(2, 1), pts];
%             end
%             R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
%             transPt = R * points + repmat([pose(1); pose(2)], 1, size(points, 2));
%             set(handles, 'XData', transPt(1, :), 'YData', transPt(2, :));
%         else
%             set(handles, 'XData', [], 'YData', []);
%         end
    end

    % Setup the "local view" window
    function [f, diag] = createFigure(block, blockName)
        f = figure('Name', ['Local view for ' blockName], 'NumberTitle', 'off');

        diag.axHistogram = axes('OuterPosition', [0 0 1 1], 'XGrid', 'on', 'YGrid', 'on', 'Visible', 'off', ...
                                'DataAspectRatio', [1 1 1], 'XLim', block.mapRadius * [-1 1], 'YLim', block.mapRadius * [-1 1]);
		
        % graphic object for the local map
        hold on;
        diag.hMap = surf('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'ZData', [], 'CData', [], 'LineStyle', 'none');
        hold off;
		        
        % setup the polar coordinate system
        % ==== taken from polar.m ====
        % plot spokes
        ls = get(diag.axHistogram, 'GridLineStyle');
        tc = get(diag.axHistogram, 'XColor');
        rmax = block.mapRadius;
        th = (1 : 6) * 2 * pi / 12 + pi/2;
        cst = cos(th);
        snt = sin(th);
        cs = [-cst; cst];
        sn = [-snt; snt];
        line(rmax * cs, rmax * sn, 'LineStyle', ls, 'Color', tc, 'LineWidth', 1, 'HandleVisibility', 'off', 'Parent', diag.axHistogram);
        
        % annotate spokes in degrees
        rt = 1.1 * rmax;
        for i = 1 : length(th)
            text(rt * cst(i), rt * snt(i), int2str(i * 30), 'HorizontalAlignment', 'center', 'HandleVisibility', 'off', 'Parent', diag.axHistogram);
            if i == length(th)
                loc = int2str(0);
            else loc = int2str(i * 30 - 180);
            end
            text(-rt * cst(i), -rt * snt(i), loc, 'HorizontalAlignment', 'center', 'HandleVisibility', 'off', 'Parent', diag.axHistogram);
        end        
        % ==== END taken from polar.m ====
        
        % setup remaining graphic elements to visualize the histogram,
        % candidates, etc.
        diag.hRawHist = line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0 0 1]);
        diag.hFilteredHist = line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [1 0 0]);
        diag.hThresholds = [line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0 0.6 0]), ...
                            line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0 0.6 0])];
        diag.hValleys = patch('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'FaceColor', [0 1 0], 'FaceAlpha', 0.2, 'LineStyle', 'none');
        diag.hSelectedValley = patch('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'FaceColor', [1 1 0], 'FaceAlpha', 0.2, 'LineStyle', 'none');
        diag.hTargetDir = line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0.5 0 0], 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 8);
        diag.hSteeringDir = line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0 0.6 0], 'LineWidth', 3);
        diag.hCandidateDir = line('Parent', diag.axHistogram, 'XData', [], 'YData', [], 'Color', [0 0.7 0], 'LineWidth', 1);
        
        % draw a robot into the center of the map
        rads = linspace(0, 2 * pi, 100); rads(end) = [];        
        XY = [cos(rads.'), sin(rads.')];
        patch('Parent', diag.axHistogram, 'XData', block.radius * XY(:, 1), 'YData', block.radius * XY(:, 2), 'FaceColor', block.color, 'EdgeColor', [0 0 0]);
        line('Parent', diag.axHistogram, 'XData', [0, 0], 'YData', [0, block.radius], 'Color', [1 1 1], 'LineWidth', 2);        
    end

    % Drawing commands to update the local view
    function diag = updateFigure(block, f, diag, out, debugOut, state, varargin)
        if ~isempty(state)
            % Draw the local map
            map = state.map;            
            mapDims = ((0:size(map, 1)) - size(map, 1) / 2) * block.mapResolution;
            [X, Y] = meshgrid(mapDims, mapDims);
            angle = state.mapPose(3);
            XY = cat(3, cos(angle) * X + sin(angle) * Y, -sin(angle) * X + cos(angle) * Y);            
            Z = zeros(size(X));                        
            map(~getMapMask(block, map)) = NaN; % mask cells outside mapRadius
            rotatedMapPose = [cos(angle), sin(angle); -sin(angle), cos(angle)] * state.mapPose(1:2);
            set(diag.hMap, 'XData', XY(:, :, 1) + rotatedMapPose(2), 'YData', XY(:, :, 2) - rotatedMapPose(1), 'ZData', Z, 'CData', repmat(map, [1 1 3]));
            
            
            % Compute conversion constants (--> radius_for_drawing = radiusOffset + original_radius * radiusScaler) 
            radiusRange = [0, 1]; % expected value range for the data displayed in the histogram
            desiredRadiusRange = [block.radius, 0.75 * block.mapRadius]; % radius range used in the visualization            
            radiusOffset = desiredRadiusRange(1) - radiusRange(1) * (desiredRadiusRange(2) - desiredRadiusRange(1)) / (radiusRange(2) - radiusRange(1));
            radiusScaler = (desiredRadiusRange(2) - desiredRadiusRange(1)) / (radiusRange(2) - radiusRange(1));
            
            % draw the thresholds
            angles = linspace(0, 2 * pi, 100);
            lowThresholdRadius = radiusOffset + radiusScaler * block.threshold(1);
            set(diag.hThresholds(1), 'XData', lowThresholdRadius * cos(angles), 'YData', lowThresholdRadius * sin(angles));
            highThresholdRadius = radiusOffset + radiusScaler * block.threshold(end);
            set(diag.hThresholds(2), 'XData', highThresholdRadius * cos(angles), 'YData', highThresholdRadius * sin(angles));
            
            % draw the histograms (unfiltered and filtered)
            alpha = 2 * pi / block.sectorCount;
            histAngles = (0:(block.sectorCount - 1)) * alpha + pi / 2;
            histAngles(end + 1) = histAngles(1);
            radii = radiusOffset + radiusScaler * debugOut.rawHistogram; radii(end + 1) = radii(1);
            set(diag.hRawHist, 'XData', radii .* cos(histAngles), 'YData', radii .* sin(histAngles));            
            radii = radiusOffset + radiusScaler * debugOut.filteredHistogram; radii(end + 1) = radii(1);
            set(diag.hFilteredHist, 'XData', radii .* cos(histAngles), 'YData', radii .* sin(histAngles));
                        
            % draw traversable sectors (valleys)
            valleys = debugOut.valleys;
            if ~isempty(valleys)
                valleyPts = zeros(0, 2);
                selValleyPts = zeros(0, 2);
                for i = 1:size(valleys, 1)
                    startAngle = (valleys(i, 1) - 0.5) * alpha;
                    endAngle = startAngle + valleys(i, 2) * alpha;
                    angles = linspace(startAngle, endAngle, max(2, round((endAngle - startAngle) / (5 * pi / 180)))) + pi/2;
                    thisValleyPts = [cos(angles.'), sin(angles.'); 0, 0];
                    if i == debugOut.valleyIndex
                        selValleyPts = thisValleyPts;
                    end
                    valleyPts = [valleyPts; desiredRadiusRange(2) * thisValleyPts];
                end
                set(diag.hValleys, 'XData', valleyPts(:, 1), 'YData', valleyPts(:, 2));                            
                set(diag.hSelectedValley, 'XData', selValleyPts(:, 1), 'YData', selValleyPts(:, 2));
            else
                set(diag.hValleys, 'XData', [], 'YData', []);
                set(diag.hSelectedValley, 'XData', [], 'YData', []);
            end
            
            % draw the target direction, candidate directions and the
            % selected steering direction
            targetDistance = sqrt(sum(state.goal.^2));
            set(diag.hTargetDir, 'XData', [0, targetDistance * cos(debugOut.targetDirection + pi/2)], 'YData', [0, targetDistance * sin(debugOut.targetDirection + pi/2)]);
            set(diag.hSteeringDir, 'XData', [0, desiredRadiusRange(2) * cos(state.steeringDirection + pi / 2)], 'YData', [0, desiredRadiusRange(2) * sin(state.steeringDirection + pi/2)]);
            
            if ~isempty(debugOut.candidates)
                candidateDirections = [debugOut.candidates.direction] + pi/2;
                set(diag.hCandidateDir, 'XData', reshape([zeros(1, length(candidateDirections)); desiredRadiusRange(2) * cos(candidateDirections(:).'); NaN(1, length(candidateDirections))], 1, []), ...
                                        'YData', reshape([zeros(1, length(candidateDirections)); desiredRadiusRange(2) * sin(candidateDirections(:).'); NaN(1, length(candidateDirections))], 1, []));

            end
        end
    end

    % Compute VFH output
    function [state, out, debugOut] = vfhStep(block, t, state, relativeGoal, rangefinder, odometer)
        if isempty(state)
            state.input = [t, 0, 0]; % [t, v, omega]

            state.mapPose = [0; 0; 0]; % pose of the robot relative to its local map (usually the robot is centered in its local map, keeping track of its position anyway is required to prevent rounding errors
            mapDimension = round((2 * block.mapRadius) / block.mapResolution);
            if mod(mapDimension, 2) == 0
                mapDimension = mapDimension + 1;
            end
            % local map should expresses P(cell is free)
            state.map = ones(mapDimension);
            state.binaryHistogram = true(1, block.sectorCount);
            state.steeringDirection = NaN;
            state.goal = [0 0];
        end
           
        if ~isempty(relativeGoal)
            state.goal = relativeGoal(end).data;
        end
        
        if ~isempty(rangefinder) && ~isempty(odometer)
            % determine egomotion        
            mapPose = state.mapPose;
            prevPhi = mapPose(3);
            tNow = state.input(1);
            tStart = tNow;
            vOmega = state.input(2:3);
            iOdometer = 0;
            while tNow < t
                tPartEnd = t;
                while iOdometer < length(odometer);
                    iOdometer = iOdometer + 1;
                    tNext = min(t, odometer(iOdometer).t);
                    if tNext >= tNow
                        tPartEnd = tNext;
                        break;
                    end
                end

                T = tPartEnd - tNow;
                if T > 0
                    % propagate using current vOmega               
                    phi = mapPose(3);
                    if abs(vOmega(2)) < 1e-12                    
                        mapPose = mapPose + [vOmega(1) * T * cos(phi); ...
                                             vOmega(1) * T * sin(phi); ...
                                                                    0];
                    else
                        mapPose(3) = mapPose(3) + T * vOmega(2);
                        mapPose(1:2) = mapPose(1:2) + vOmega(1) / vOmega(2) * [sin(mapPose(3)) - sin(phi); ...
                                                                               cos(phi) - cos(mapPose(3))];
                    end
                end  

                tNow = tPartEnd;
                if iOdometer <= length(odometer)
                    u = odometer(iOdometer).data;
                    vOmega = [0.5 * (block.wheelRadius(1) * u(1) + block.wheelRadius(end) * u(2)), ...
                              (block.wheelRadius(1) * u(1) - block.wheelRadius(end) * u(2)) / (block.wheelDistance)];
                end            
            end
            deltaPhi = mapPose(3) - prevPhi;
            mapPose(3) = mod(mapPose(3) + pi, 2 * pi) - pi;
            state.mapPose = mapPose;
            state.input = [t, vOmega];

            % move map window centered around robot
            posInMap = round(mapPose(1:2) / block.mapResolution);
            if posInMap(1) > 0
                if posInMap(1) < size(state.map, 1)
                    state.map(1:(end - posInMap(1)), :) = state.map((posInMap(1) + 1):end, :);
                    state.map((end - posInMap(1) + 1):end, :) = 1;
                else state.map(:) = 1;
                end                    
            elseif posInMap(1) < 0
                if -posInMap(1) < size(state.map, 1)
                    state.map((1 - posInMap(1)):end, :) = state.map(1:(end + posInMap(1)), :);
                    state.map(1:-posInMap(1), :) = 1;
                else state.map(:) = 1;
                end
            end
            if posInMap(2) > 0
                if posInMap(2) < size(state.map, 2)
                    state.map(:, (posInMap(2) + 1):end) = state.map(:, 1:(end - posInMap(2)));
                    state.map(:, 1:posInMap(2)) = 1;
                else state.map(:) = 1;
                end                    
            elseif posInMap(2) < 0
                if -posInMap(2) < size(state.map, 2)
                    state.map(:, 1:(end + posInMap(2))) = state.map(:, (1 - posInMap(2)):end);
                    state.map(:, (end + posInMap(2) + 1):end) = 1;
                else state.map(:) = 1;
                end                    
            end
            state.mapPose(1:2) = state.mapPose(1:2) - posInMap * block.mapResolution;        

            % update map from sensor measurements
            if ~isempty(rangefinder)
                validIdx = isfinite(rangefinder(end).data.range);
                range = rangefinder(end).data.range(validIdx);

                bearing = rangefinder(end).data.bearing(validIdx);
                posInMap = [state.mapPose(1) + range .* cos(bearing + state.mapPose(3)), ...
                            state.mapPose(2) + range .* sin(bearing + state.mapPose(3))];
                cellInMap = round([posInMap(:, 1), -posInMap(:, 2)] / block.mapResolution + size(state.map, 1) / 2 + 0.5);
                validCells = (cellInMap(:, 1) >= 1) & (cellInMap(:, 2) >= 1) & (cellInMap(:, 1) <= size(state.map, 1)) & (cellInMap(:, 2) <= size(state.map, 2));
                cellInMap = cellInMap(validCells, :);
                newMap = ones(size(state.map));

                for i = 1:size(cellInMap, 1)
                    newMap(cellInMap(i, 1), cellInMap(i, 2)) = newMap(cellInMap(i, 1), cellInMap(i, 2)) * block.sensorUncertainty;
                end  
                state.map = state.map .* newMap;
            end

            % generate vector field histogram
            [cellCenterU, cellCenterV] = meshgrid(((1:size(state.map, 1)) - size(state.map, 1) / 2 - 0.5) * block.mapResolution);
            validIdx = (cellCenterU.^2 + cellCenterV.^2) <= block.mapRadius^2;
            
            thetaTarget = atan2(state.goal(2), state.goal(1)); % atan2(0, 0) = 0 -> in case we have no valid goal, the current direction is kept
            debugOut.targetDirection = thetaTarget;

            alpha = 2 * pi / block.sectorCount;

            args.cellRadius = sqrt(2) * block.mapResolution / 2 + block.radius + block.safetyDistance;		
            args.a = log(1e-3) / -block.obstacleInfluenceRange;        
            args.cells = reshape(state.map(validIdx), [], 1);
            args.cellCenters = [reshape(cellCenterV(validIdx), [], 1), reshape(-cellCenterU(validIdx), [], 1)];
            args.pose = state.mapPose;
            args.velocity = vOmega(1);
            args.alpha = alpha;
            args.prevBinaryHistogram = state.binaryHistogram;
            args.prevSteeringDirection = state.steeringDirection - deltaPhi;

            args.thetaTarget = thetaTarget;
            args.deltaT = t - tStart;
            if args.deltaT == 0; args.deltaT = 0.1; end

            % VFH core is factored out into a subroutine as a preparation
            % for a future VFH* implementation
            res = getCandidates(args, block);

            debugOut.rawHistogram = res.rawHistogram;        
            debugOut.filteredHistogram = res.filteredHistogram;
            state.binaryHistogram = res.binaryHistogram;

            debugOut.valleys = res.valleys;        
            debugOut.valleyIndex = res.valleyIndex;                
            debugOut.candidates = res.candidates;

            if ~isempty(res.candidates)                        
                % evaluate target directions
                [~, minIdx] = min([res.candidates.cost]);

                state.steeringDirection = res.candidates(minIdx).direction;
                V = res.candidates(minIdx).command(1);
                omega = res.candidates(minIdx).command(2);            
            else            
                state.steeringDirection = NaN;
                V = block.minVelocity;
                omega = block.maxOmega / 4;
            end
        else 
            V = 0;
            omega = 0;

            debugOut.targetDirection = NaN;
            debugOut.rawHistogram = NaN(1, block.sectorCount);
            debugOut.filteredHistogram = NaN(1, block.sectorCount);
            debugOut.valleys = zeros(0, 2);
            debugOut.valleyIndex = [];
            debugOut.candidates = [];            
        end
        
            
        debugOut.vOmega = [V; omega];
        out = [1 / block.wheelRadius(1), 0.5 * block.wheelDistance / block.wheelRadius(1); 1 / block.wheelRadius(end), -0.5 * block.wheelDistance / block.wheelRadius(end)] * [V; omega];
    end
end

function M = getMapMask(block, map)
    [U, V] = meshgrid(((1:size(map, 1)) - size(map, 1) / 2 - 0.5) * block.mapResolution);
    M = (U.^2 + V.^2) <= block.mapRadius^2;
end

function out = getCandidates(args, block)
	betas = mod(atan2(args.cellCenters(:, 2) - args.pose(2), args.cellCenters(:, 1) - args.pose(1)) - args.pose(3), 2 * pi);        
    d = sqrt(sum((args.cellCenters - repmat(args.pose(1:2)', size(args.cellCenters, 1), 1)).^2, 2));

    % scaling different from paper        
    m = 1 - (1 - args.cells) .* exp(-args.a * max(0, d - args.cellRadius));

    v = min(args.velocity, block.maxVelocity);
    R_min = block.maxVelocity / block.maxOmega * (1 / ((block.maxVelocity / v) - 1));

    % hist ~ P(sector is traversable)
    hist = ones(1, block.sectorCount);
    alpha = args.alpha;

    asinArgs = args.cellRadius ./ d;
    asinArgs(asinArgs > 1) = 1;
    gammas = asin(asinArgs);
    gammas(~isreal(gammas)) = pi/2;
    sectorsStart = round((betas - gammas) / alpha);
    sectorsEnd = round((betas + gammas) / alpha);
    backSector = max(1, round(block.sectorCount / 2));
    for i = 1:length(sectorsStart)
        isLeft = (betas(i) < pi);
        if isLeft
            m_circ = [0, R_min];
        else m_circ = [0, -R_min];
        end

        cellPt = d(i) * [cos(betas(i)), sin(betas(i))];
        if sum((cellPt - m_circ).^2) <= (R_min + args.cellRadius)
            % enlarge sector to the back of the robot
            if isLeft                    
                sectorsEnd(i) = max(sectorsEnd(i), backSector);
            else sectorsStart(i) = min(sectorsStart(i), backSector);
            end
        end

        sectors = mod(sectorsStart(i):sectorsEnd(i), block.sectorCount) + 1;
        hist(sectors) = hist(sectors) * m(i);
    end
    
    hist = 1 - hist;
    out.rawHistogram = hist;
    
    % filter histogram
    l = block.filterLength;        
    accuHist = zeros(size(hist));
    for i = (-l + 1):(l - 1)
        accuHist = accuHist + abs(l - i) * hist(mod((0:(length(hist) - 1)) + i, length(hist)) + 1);
    end
    hist = accuHist / (l * l);    
    out.filteredHistogram = hist;
    
    % thresholding with hysteresis according to VFH+ paper
    tau_low = block.threshold(1);
    tau_high = block.threshold(end);

    freeIdx = args.prevBinaryHistogram;
    freeIdx(hist <= tau_low) = true;
    freeIdx(hist >= tau_high) = false;
    out.binaryHistogram = freeIdx;    

    % determine valleys
    valleys = zeros(0, 2); % [start index, length] of valley
    inValley = false;
    for i = 1:length(freeIdx)
        if freeIdx(i)
            if ~inValley
                inValley = true;
                valleys(end + 1, :) = [i - 1, 0];
            end
            valleys(end, 2) = valleys(end, 2) + 1;                    
        else
            if inValley; inValley = false; end
        end
    end

    if ~isempty(valleys)
        if size(valleys, 1) > 1 && valleys(1, 1) == 0 && (valleys(end, 1) + valleys(end, 2) >= length(freeIdx))
            % merge a valley that was split around zero degree
            valleys(1, 2) = valleys(1, 2) + valleys(end, 2);
            valleys(1, 1) = valleys(end, 1);
            valleys(end, :) = [];
        end
    end   
    
    out.valleys = valleys;    
    
    if ~isempty(valleys)                        
        valleyStartAngles = alpha * (valleys(:, 1) - 0.5);
        valleyEndAngles = valleyStartAngles + alpha * valleys(:, 2);
        deltaThetaStarts = mod(valleyStartAngles - args.thetaTarget + pi, 2 * pi) - pi;
        correctIdx = (deltaThetaStarts < 0 & valleyStartAngles - deltaThetaStarts > valleyEndAngles);
        correctIdx2 = (deltaThetaStarts > 0 & valleyStartAngles - deltaThetaStarts + 2 * pi < valleyEndAngles);
        deltaThetaStarts(correctIdx) = deltaThetaStarts(correctIdx) + 2 * pi;
        deltaThetaStarts(correctIdx2) = deltaThetaStarts(correctIdx2) - 2 * pi;

        deltaThetaEnds = mod(args.thetaTarget - valleyEndAngles + pi, 2 * pi) - pi;
        correctIdx = (deltaThetaEnds < 0 & valleyEndAngles + deltaThetaEnds < valleyStartAngles);
        correctIdx2 = (deltaThetaEnds > 0 & valleyEndAngles + deltaThetaEnds - 2 * pi > valleyStartAngles);
        deltaThetaEnds(correctIdx) = deltaThetaEnds(correctIdx) + 2 * pi;
        deltaThetaEnds(correctIdx2) = deltaThetaEnds(correctIdx2) - 2 * pi;


        if 0
            % VFH method
            [~, valleyIndex] = min(min(max(0, deltaThetaStarts), max(0, deltaThetaEnds)));
            out.valleyIndex = valleyIndex;

            if (deltaThetaStarts(valleyIndex) <= 0 && deltaThetaEnds(valleyIndex) <= 0)
                % target direction is inside valley -> use target direction
                % as steering direction (this is not explicitely stated in
                % the paper!)
                candidateDirections = args.thetaTarget;
            else
                valley = valleys(valleyIndex, :);
                if deltaThetaStarts(valleyIndex) < deltaThetaEnds(valleyIndex)
                    % sector start is near border
                    kn = valley(1);                    
                    if valley(2) < block.maxSectors
                        kf = kn + valley(2) - 1;
                    else kf = kn + block.maxSectors;
                    end
                else
                    % sector end is near border
                    kn = valley(1) + valley(2) - 1;
                    if valley(2) < block.maxSectors
                        kf = valley(1);
                    else kf = kn - block.maxSectors;
                    end
                end

                candidateDirections = alpha * (kn + kf) / 2;
            end
        else
            % VFH+ method
            out.valleyIndex = [];

            % collect all candidates
            candidateDirections = [];
            for i = 1:size(valleys, 1)
                if valleys(i, 2) < block.maxSectors
                    candidateDirections(end + 1) = 0.5 * (valleyStartAngles(i) + valleyEndAngles(i));
                else
                    candidateDirections(end + 1) = valleyStartAngles(i) + 0.5 * alpha * block.maxSectors;
                    candidateDirections(end + 1) = valleyEndAngles(i) - 0.5 * alpha * block.maxSectors;
                    if deltaThetaStarts(i) <= 0 && deltaThetaEnds(i) <= 0
                        % target direction inside valley
                        candidateDirections(end + 1) = args.thetaTarget;
                    end
                end
            end
        end

        % fill in candidate directions, costs and the corresponding v/omega
        % suggestions
        candidateDirections = mod(candidateDirections(:) + pi, 2 * pi) - pi;
        
        targetDeltas = abs(mod(candidateDirections - args.thetaTarget + pi, 2 * pi) - pi);
        currentDeltas = abs(mod(candidateDirections + pi, 2 * pi) - pi); % directions are relative to current orientation, therefore the delta is candidateDirection - 0
        if ~isnan(args.prevSteeringDirection)
            prevDeltas = abs(mod(candidateDirections - args.prevSteeringDirection + pi, 2 * pi) - pi);
        else prevDeltas = zeros(size(currentDeltas));
        end                
        candidateCosts = block.weights(1) * targetDeltas + block.weights(2) * currentDeltas + block.weights(3) * prevDeltas;

        % compute control commands (similar to original VFH paper, there is definitely room for improvements...)                
        hcprime = hist(mod(round(candidateDirections / alpha), length(hist)) + 1).';
        Vprime = (block.maxVelocity - block.minVelocity) * (1 - min(block.h_m, hcprime) / block.h_m);
        T = args.deltaT;        
        omega = candidateDirections / T;        
        omega = sign(omega) .* min(abs(omega), block.maxOmega);
        V = Vprime .* (1 - abs(omega) / block.maxOmega) + block.minVelocity;

        out.candidates = repmat(struct('direction', [], 'cost', [], 'command', []), size(candidateDirections));
        for i = 1:numel(out.candidates)
            out.candidates(i).direction = candidateDirections(i);
            out.candidates(i).cost = candidateCosts(i);
            out.candidates(i).command = [V(i), omega(i)];
        end        
    else
        out.candidates = [];
        out.valleyIndex = [];
    end
end
