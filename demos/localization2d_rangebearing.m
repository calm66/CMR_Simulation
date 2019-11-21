function filter = localization2d_rangebearing()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block     
    filter.depends = {'sensors/landmark_detector', 'environment/landmarks'};
    
    filter.default_initialPose = [0 0 0]';
    
    function [state, out, debugOut] = filterStep(block, t, state, sensor, landmarks)
        debugOut = [];
        
        if ~isempty(sensor)
            visIdx = sensor(end).data.lmIds; % assume we can associate each measurement with a known landmark
        else visIdx = [];
        end
        
        % quantities entering the filter (all vectors are column vectors)            
        if numel(visIdx) >= 2
            H = zeros(0, 3);
            R = zeros(0, 0);
            y = zeros(0, 1);

            lm  = landmarks.data(visIdx, :);
            rb = [sensor.data.range sensor.data.bearing];

            M = [];
            b = [];
            for i = 1:length(visIdx)
                M(end+1:end+2,1:4) = [1 0 -rb(i,1)*sin(rb(i,2))  rb(i,1)*cos(rb(i,2));...
                                      0 1  rb(i,1)*cos(rb(i,2))  rb(i,1)*sin(rb(i,2))];
                b(end+1:end+2,1) = [lm(i,1); lm(i,2)];
            end

            z = inv(M'*M)*M'*b;

            x = [z(1); z(2); atan2(z(3),z(4))];


            P      = zeros(3, 3);
            P(1,1) = 0.02;
            P(2,2) = 0.02;
        else
            x = NaN(3, 1);
            P = zeros(3, 3);
        end

        % store state for visualization
        state.pose = x;
        state.cov = P;
        % store estimated pose
        out = state;
    end
end