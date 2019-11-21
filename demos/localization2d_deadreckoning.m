function filter = localization2d_deadreckoning()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block     

    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, landmarks)
        debugOut = [];
        
        if isempty(state)            
            if isempty(block.initialPose)
                %state.pose = platform(1).data.pose + block.initialPoseError .* randn(1, 3);
                state.pose = platform(1).data + block.initialPoseError .* randn(1, 3);
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
            P(3,3) = 0; 

            % store new state distribution
            state.pose = [x(1:2)', mod(x(3) + pi, 2 * pi) - pi];
            state.cov = P;
        end
        
        out = state;
    end
end