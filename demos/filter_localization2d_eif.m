function filter = filter_localization2d_eif()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block      

    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, landmarks)
        debugOut = [];
        
        if isempty(state)            
            if isempty(block.initialPose)
                state.pose = platform(1).data.pose + block.initialPoseError .* randn(1, 3);;
            else state.pose = block.initialPose;
            end
            state.pose(3) = mod(state.pose(3) + pi, 2 * pi) - pi;
            state.cov = diag(block.initialPoseError.^2);
            state.H = inv(state.cov);
			state.b = state.H * state.pose';            
        end
        
        if t > 0
            % quantities entering the filter (all vectors are column vectors)
            b = state.b;					% information vector
            H = state.H;					% information matrix

            u = platform.data.odometry';
            x = state.pose';                % state recovered from b

            %%% 1. prediction %%%
            J_fx = eye(3);							% jacobi matrix of transfer function w.r.t. the state (df/dx)
            J_fu = eye(3);							% jacobi matrix of transfer function w.r.t. the input (df/du)
            Q = J_fu * diag(block.odometryError.^2) * J_fu'; % transform input noise into process noise
            H = inv(J_fx / H * J_fx' + Q);
            x = x + u;								% state prediction: x_k|k-1 = f(x_k-1, u_k), usually nonlinear
            b = H * x;

            %%% 2. update %%%
            visIdx = sensor.data.lmIds; % assume we can associate each measurement with a known landmark
            if ~isempty(visIdx)
                % prepare innovation vector, output jacobi matrix and measurement noise covariance
                y = zeros(0, 1);
                J_hx = zeros(0, 3);
                R_inv = zeros(0, 0);

                deltas = landmarks.data(visIdx, :) - repmat(x(1:2)', size(visIdx));
                if block.useBearing
                    % compute output vector z = h(x_k|k-1) (= model-based prediction of measurement)				
                    z = atan2(deltas(:, 2), deltas(:, 1)) - x(3);

                    % innovation vector (measurement - output prediction)
                    y = [y; mod(sensor.data.bearing - z + pi, 2 * pi) - pi]; % force into interval +-pi

                    % J_hx = jacobi matrix of output function w.r.t. state (dh/dx)
                    denoms = deltas(:, 1).^2 + deltas(:, 2).^2;
                    J_hx = [J_hx; [deltas(:, 2) ./ denoms, -deltas(:, 1) ./ denoms, repmat(-1, size(visIdx))]];

                    % R = covariance matrix of measurement noise
                    R_inv = blkdiag(R_inv, (1 / block.bearingError^2) * eye(length(visIdx)));                
                end			
                if block.useRange
                    z = sqrt(deltas(:, 1).^2 + deltas(:, 2).^2);

                    ranges = sensor.data.range;
                    y = [y; ranges - z];			
                    J_hx = [J_hx; [-deltas(:, 1) ./ z, -deltas(:, 2) ./ z, zeros(size(visIdx))]];

                    R_inv = blkdiag(R_inv, diag(1 ./ (block.rangeError * ranges).^2));					
                end

                % compute information vector and matrix 
                Scaler = J_hx' * R_inv;

                H = H + Scaler * J_hx;
                b = b + Scaler * (y + J_hx * x);			
            end
            state.H = H;
            state.b = b;

            % state recovery (for visualization and next step (x only))
            P = inv(H);
            x = P * b;
            state.pose = [x(1:2)', mod(x(3) + pi, 2 * pi) - pi];
            state.cov = P;
        end
        
        out = state;
    end
end