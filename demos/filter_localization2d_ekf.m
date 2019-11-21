function filter = filter_localization2d_ekf()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block     

    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, landmarks)
        debugOut = [];
        
        if isempty(state)            
            if isempty(block.initialPose)
                state.pose = platform(1).data + block.initialPoseError .* randn(1, 3);;
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

            %%% 1. prediction %%%

            % state prediction: x_k|k-1 = f(x_k-1, u_k), usually nonlinear
            x = x + u;

            % covariance prediction: P_k|k-1 = F * P_k-1 * F' + Q		
            F = eye(3);								% jacobi matrix of transfer function w.r.t. the state (df/dx)
            J_fu = eye(3);							% jacobi matrix of transfer function w.r.t. the input (df/du)
            Q = J_fu * diag(block.odometryError.^2) * J_fu'; % transform input noise into process noise
            P = F * P * F' + Q;						% covariance prediction

            %%% 2. update %%%
            visIdx = sensor.data.lmIds; % assume we can associate each measurement with a known landmark
            if ~isempty(visIdx)
                H = zeros(0, 3);
                R = zeros(0, 0);
                y = zeros(0, 1);

                deltas = landmarks.data(visIdx, :) - repmat(x(1:2)', size(visIdx));
                if block.useBearing
                    % compute output vector z = h(x_k|k-1) (= model-based prediction of measurement)				
                    z = atan2(deltas(:, 2), deltas(:, 1)) - x(3);

                    % innovation vector (measurement - output prediction)
                    y = [y; mod(sensor.data.bearing - z + pi, 2 * pi) - pi]; % force into interval +-pi

                    % H = jacobi matrix of output function w.r.t. state (dh/dx)
                    denoms = deltas(:, 1).^2 + deltas(:, 2).^2;
                    H = [H; [deltas(:, 2) ./ denoms, -deltas(:, 1) ./ denoms, repmat(-1, size(visIdx))]];

                    % R = covariance matrix of measurement noise
                    R = blkdiag(R, block.bearingError^2 * eye(length(visIdx)));
                end
                if block.useRange
                    z = sqrt(deltas(:, 1).^2 + deltas(:, 2).^2);

                    ranges = sensor.data.range;			
                    y = [y; ranges - z];			
                    H = [H; [-deltas(:, 1) ./ z, -deltas(:, 2) ./ z, zeros(size(visIdx))]];

                    R = blkdiag(R, diag((block.rangeError * ranges).^2));			
                end

                % covariance S of the innovation vector
                S = H * P * H' + R;
                % compute Kalman gain K
                K = P * H' / S;			
                % correct state x and covariance P
                x = x + K * y;
                P = (eye(3) - K * H) * P;
            end

            % store new state distribution
            state.pose = [x(1:2)', mod(x(3) + pi, 2 * pi) - pi];
            state.cov = P;
        end
        
        out = state;
    end
end