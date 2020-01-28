function model = model_platform2d_vomega_with_inertial_odometry(initialPose)
    if ~isnumeric(initialPose) || ~isreal(initialPose) || length(initialPose) ~= 3
        error('Invalid format: initialPose should be a 3-element vector');
    end
    
    model = model_platform2d(@move, 'controller');

    model.odometryError = [0.005, 0.005, (0.5 * pi / 180)];	% stddev for odometry [m, m, rad]

    function [state, out, debugOut] = move(block, tNow, state, control)
        debugOut = [];
        if isempty(state)           
            state = [0 initialPose(:)']; 
        end
        
        if isempty(control) 
            inPart = [0 0]; 
            iIn = 1;
        else
            inPart = control(1).data;
            iIn = 2;
        end
        
        t = state(1);
        X = state(2:end);
        while t < tNow
            if iIn <= length(control); 
                tPartEnd = control(iIn).t; 
                iIn = iIn + 1;                    
            else tPartEnd = tNow; 
            end
            if (tPartEnd - t) > 1e-6
                % ode45 seems to have problems with small time intervals
                [~, X] = ode45(@(t, x)dynModel(t, x, inPart), [t, tPartEnd], X);
                X = X(end, :);
            end
            t = tPartEnd;
        end
        
        deltaPose = X - state(2:end) + block.odometryError .* randn(1, 3);
        state = [t X];
        out.pose = X;        
        out.odometry = [deltaPose(1:2), mod(deltaPose(3) + pi, 2 * pi) - pi];

        function dx = dynModel(~, x, u)
            dx = [u(1) * cos(x(3)); ...
                  u(1) * sin(x(3)); ...
                  u(2)];
        end                    
    end
end
