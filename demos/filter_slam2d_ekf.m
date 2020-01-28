function filter = filter_slam2d_ekf()
    filter = filter_slam2d(@filterStep); % reuse drawing function from generic localization2d block
            
    filter.figures(end + 1).name = 'Covariance Plot';
    filter.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'covariance_icon.png');
    filter.figures(end).init = @createCovFigure;
    filter.figures(end).draw = @updateCovFigure;

    
    function [f, diag] = createCovFigure(block, blockName)       
        f = figure('Name', ['Covariance for ' blockName], 'NumberTitle', 'off');
        diag.ax = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'DataAspectRatio', [1 1 1], 'YDir', 'reverse');
        colormap(diag.ax, 'gray');
        colorbar('peer', diag.ax);
        diag.hCovImg = image('Parent', diag.ax, 'CData', [], 'CDataMapping', 'scaled');
    end
    function diag = updateCovFigure(block, f, diag, out, debug, state, varargin)
        set(diag.hCovImg, 'CData', state.P);
    end
    
    
    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, varargin)
        debugOut = [];
    
        if isempty(state)
            % initially, the state is comprised of only the vehicle pose
            % (no a-priory feature information)
            if isempty(block.initialPose)
                state.x = platform(1).data.pose' + block.initialPoseError' .* randn(3, 1);
            else state.x = block.initialPose';
            end
            state.x(3) = mod(state.x(3) + pi, 2 * pi) - pi;
            state.P = diag(block.initialPoseError.^2);
            state.Q = diag(block.odometryError.^2);
            state.features = []; % array of landmark indices for each feature in x
        end
        
        if t > 0
            % prediction step
            state.x(1:3) = state.x(1:3) + platform.data.odometry';
            state.x(3) = mod(state.x(3) + pi, 2 * pi) - pi;
            % jacobi matrix of state transition function w.r.t. state
            F = eye(length(state.x));
            state.P = F * state.P * F' + state.Q;
        end
        
        % process measurements

        % determine, which measurements belong to features already part
        % of the state vector and which we see for the first time (map
        % management)
        [~, fidx_old, midx_old] = intersect(state.features, sensor.data.lmIds);
        [~, midx_new] = setdiff(1:length(sensor.data.lmIds), midx_old);

        % interpretation of the index arrays
        % midx_old -> indices of measurements of known landmarks...
        % fidx_old -> ...and their associated indices in the state vector
        % midx_new -> indices of measurments of landmarks we see for the first time

        if ~isempty(midx_new)
            % feature initialization for first-time measurements. 
            % Regardless of the block.useBearing/block.useRange
            % settings, we always have to use both measurements here to
            % uniquely determine the initial feature estimate!

            % length of state vector before adding new features
            len = length(state.x);

            % range & bearing measurements as row vectors
            bearings = sensor.data.bearing(midx_new)';
            ranges = sensor.data.range(midx_new)';

            pose = state.x(1:3);
            c = cos(pose(3) + bearings);
            s = sin(pose(3) + bearings);
            % extend state vector with initial x/y estimates of new features
            state.x = [state.x; reshape([pose(1) + ranges .* c; pose(2) + ranges .* s], [], 1)];

            % intermediate covariance matrix for old state combined with new measurements
            bearingErrors = repmat(block.bearingError^2, 1, length(midx_new));
            rangeErrors = (block.rangeError * ranges).^2;
            P_helper = blkdiag(state.P, diag(reshape([bearingErrors; rangeErrors], 1, [])));

            % matrix to transform P_helper into covariance for extended state
            J = zeros(length(state.x), len + 2 * length(midx_new));			
            J(sub2ind(size(J), 1:len, 1:len)) = 1; % keep old state variables -> top-left block is identity matrix
            x_rows = len + (1:2:(2 * length(midx_new)));
            y_rows = len + (2:2:(2 * length(midx_new)));
            J(x_rows, 1) = 1;
            J(y_rows, 2) = 1;
            J(x_rows, 3) = -ranges .* s;
            J(y_rows, 3) = ranges .* c;
            bearing_cols = len + (1:2:(2 * length(midx_new)));
            range_cols = len + (2:2:(2 * length(midx_new)));
            J(sub2ind(size(J), x_rows, bearing_cols)) = J(x_rows, 3);
            J(sub2ind(size(J), y_rows, bearing_cols)) = J(y_rows, 3);
            J(sub2ind(size(J), x_rows, range_cols)) = c;
            J(sub2ind(size(J), y_rows, range_cols)) = s;	

            % extend state covariance (using the transformation J), process noise Q and landmark association data
            state.P = J * P_helper * J';
            state.Q = blkdiag(state.Q, zeros(2 * length(midx_new))); % zero process noise since features are assumed static
            state.features = [state.features; sensor.data.lmIds(midx_new)];
        end

        if ~isempty(midx_old)
            % process measurements of tracked landmarks

            % prepare innovation vector, output jacobi matrix and measurement noise covariance
            y = zeros(0, 1);
            H = zeros(0, length(state.x));
            R = zeros(0, 0);

            % indices of the feature coordinates in the state vector
            x_idx = 2 + 2 * fidx_old';
            y_idx = 3 + 2 * fidx_old';

            deltas = [state.x(x_idx) - state.x(1), state.x(y_idx) - state.x(2)];

            if block.useBearing
                % predict bearing measurements
                z = atan2(deltas(:, 2), deltas(:, 1)) - state.x(3);
                % determine innovation from bearing measurements
                y = [y; mod(sensor.data.bearing(midx_old) - z + pi, 2 * pi) - pi];

                % fill in corresponding entries in the jacobi matrix
                denoms = sum(deltas.^2, 2);
                H_b = zeros(length(midx_old), length(state.x));
                H_b(:, [1 2]) = [deltas(:, 2) ./ denoms, -deltas(:, 1) ./ denoms];
                H_b(:, 3) = -1;
                H_b(sub2ind(size(H_b), 1:length(midx_old), x_idx)) = -deltas(:, 2) ./ denoms;
                H_b(sub2ind(size(H_b), 1:length(midx_old), y_idx)) = deltas(:, 1) ./ denoms;
                H = [H; H_b];

                % covariance of measurement noise -> for bearing measurements independent of sensor output
                R = blkdiag(R, block.bearingError^2 * eye(length(midx_old)));
            end

            if block.useRange

                % predict range measurements
                z = sqrt(sum(deltas(:, 1).^2 + deltas(:, 2).^2, 2));
                % 'real' sensor output
                ranges = sensor.data.range(midx_old);
                % compute difference = innovation 
                y = [y; ranges - z];

                % fill in corresponding entries in the jacobi matrix
                H_r = zeros(length(midx_old), length(state.x));
                H_r(:, [1 2]) = [-deltas(:, 1) ./ z, -deltas(:, 2) ./ z];
                H_r(sub2ind(size(H_r), 1:length(midx_old), x_idx)) = deltas(:, 1) ./ z;
                H_r(sub2ind(size(H_r), 1:length(midx_old), y_idx)) = deltas(:, 2) ./ z;
                H = [H; H_r];

                % covariance of measurement noise (noise scales with distance)
                R = blkdiag(R, diag((block.rangeError * ranges).^2));				
            end

            % innovation covariance
            S = H * state.P * H' + R;
            % compute Kalman gain matrix
            K = state.P * H' / S;

            % EKF update
            state.x = state.x + K * y;
            state.P = (eye(size(state.P)) - K * H) * state.P;
        end

        out.pose = [state.x(1:2)', mod(state.x(3) + pi, 2 * pi) - pi];
        out.cov = state.P(1:3, 1:3);
        out.featurePositions = reshape(state.x(4:end), 2, [])';
        out.landmarkIds = state.features;
        autoCorrs = diag(state.P(4:end, 4:end));
        crossCorrs = diag(state.P(4:end, 4:end), 1);
        out.featureCovariances = [autoCorrs(1:2:end), crossCorrs(1:2:end), autoCorrs(2:2:end)];
    end
end
