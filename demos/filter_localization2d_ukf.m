function filter = filter_localization2d_ukf()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block
    
    % Parameters for Unscented Transform
    filter.alpha = 1e-3;
    filter.kappa = 0;
    filter.beta = 2; % optimal value for normal distributions (minimizes curtosis (a.k.a. 4th order moment) error)
    
    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, landmarks)
        debugOut = [];
        
        if isempty(state)            
            if isempty(block.initialPose)
                state.pose = platform(1).data.pose + block.initialPoseError .* randn(1, 3);
            else state.pose = block.initialPose;
            end
            state.pose(3) = mod(state.pose(3) + pi, 2 * pi) - pi;
            state.cov = diag(block.initialPoseError.^2);
        end
        
        if t > 0
            % quantities entering the filter (all vectors are column vectors)
            x = state.pose';
            P = state.cov;
            u = platform.data.odometry';


            % This implementation assumes additive, zero-mean gaussian noise,
            % therefore we do not need to add noise terms to the state vector

            L = length(x);
            lambda = block.alpha^2 * (L + filter.kappa) - L;

            % compute matrix root: sqrt((L + lambda) * P)
            mRoot = sqrt(L + lambda) * chol(P, 'lower');        

            % generate Sigma Points...
            X = [x, repmat(x, 1, L) + mRoot, repmat(x, 1, L) - mRoot];
            % ...and weight factors
            Ws = [lambda / (L + lambda), repmat(1 / (2 * (L + lambda)), 1, 2 * L)]; % for state
            Wc = [Ws(1) + (1 - filter.alpha^2 + filter.beta), Ws(2:end)]; % for covariance matrix (same except for first element)

            %%% 1. prediction %%%

            % propagation of sigma points with usually nonlinear system model
            X = X + repmat(u, 1, length(Ws));
            % reconstruct mean and covariance from sigma points
            x = sum(repmat(Ws, 3, 1) .* X, 2);
            X_delta = X - repmat(x, 1, length(Ws));
            P = X_delta * diag(Wc) * X_delta' + diag(block.odometryError.^2);

            %%% 2. update %%%
            visIdx = sensor.data.lmIds; % assume we can associate each measurement with a known landmark
            if ~isempty(visIdx)
                % prepare variables
                Z = zeros(0, length(Ws));	% sigma points transformed via nonlinear output function, i.e. predicted measurement for each sigma point
                Z_delta = Z;				% Z w.r.t. to its mean
                z = zeros(0, 1);			% mean of outputs, computed from Z and weight factors Ws
                y = z;						% innovation = measurement - prediction (= z)
                R = [];						% covariance of additive measurement noise

                % use bearing information, if enabled
                if block.useBearing
                    % transform sigma points using nonlinear output function
                    Z_b = zeros(length(visIdx), length(Ws));
                    for i = 1:length(Ws)
                        deltas = landmarks.data(visIdx, :) - repmat(X(1:2, i)', size(visIdx));
                        Z_b(:, i) = mod(atan2(deltas(:, 2), deltas(:, 1)) - X(3, i) + pi, 2 * pi) - pi;
                    end
                    Z = [Z; Z_b];
                    % mean of transformed sigma points
                    z_b = sum(repmat(Ws, length(visIdx), 1) .* Z_b, 2);
                    z = [z; z_b];
                    % covariance of measurement noise
                    R = blkdiag(R, diag(repmat(block.bearingError^2, size(visIdx))));
                    % innovation from bearing information
                    y = [y; (mod(sensor.data.bearing - z_b + pi, 2 * pi) - pi)];
                    Z_delta = [Z_delta; (mod(Z_b - repmat(z_b, 1, length(Ws)) + pi, 2 * pi) - pi)];				
                end
                % use range information, if enabled
                if block.useRange
                    % transform sigma points using nonlinear output function
                    Z_r = zeros(length(visIdx), length(Ws));
                    for i = 1:length(Ws)
                        deltas = landmarks.data(visIdx, :) - repmat(X(1:2, i)', size(visIdx));
                        Z_r(:, i) = sqrt(sum(deltas.^2, 2));
                    end
                    Z = [Z; Z_r];
                    % mean from transformed sigma points
                    z_r = sum(repmat(Ws, length(visIdx), 1) .* Z_r, 2);
                    z = [z; z_r];
                    % covariance of measurement noise
                    ranges = sensor.data.range;
                    R = blkdiag(R, diag((block.rangeError * ranges).^2));
                    % innovation from range measurements
                    y = [y; ranges - z_r];
                    Z_delta = [Z_delta; Z_r - repmat(z_r, 1, length(Ws))];
                end

                % covariance of the complete measurement prediction (from transformed sigma points w.r.t. their mean z)
                Pzz = Z_delta * diag(Wc) * Z_delta' + R;

                % cross terms of state and measurement prediction, computed from sigma points
                Pxz = X_delta * diag(Wc) * Z_delta';
                % correction matrix
                U = chol(Pzz);
                K = (Pxz / U) / U';
                % apply correction
                x = x + K * y;
                P = P - K * Pxz';
            end

            % store new state distribution
            state.pose = [x(1:2)', mod(x(3) + pi, 2 * pi) - pi];
            state.cov = P;
        end
        
        out = state;
    end
end