% Perform Localization & SLAM experiments with a single or multiple robots
% Inputs:
%  - filter algorithm: available localization algorithms: 'EKF', 'EIF', 'UKF', 'PF'|'MCL'
%                      available SLAM algorithms: 'EKFSLAM'|'SLAM'
%                      default: 'EKF'
%  - number of robots: default = 1

function exp = exp_navigation2d(varargin) 
    exp = experiment_base('navigation2d');
        
    exp.environment = grp_obstacles_and_landmarks_from_image('../maps/office.png', 'scale', 0.01);    
    pathPoints = [1.00,   0.50;...
                  1.00, 4.30;...
                  2.60, 4.30;...
                  2.60, 2.70;...
                  6.00, 2.70;... 
                  6.00, 5.10;...
                  7.00, 5.10;...
                  6.00, 5.10;...
                  6.00, 3.30;...
                  7.50, 2.70;...
                  7.50, 0.75;...
                  5.00, 1.50;...
                  2.00, 1.50;...
                  2.00, 0.50];
              
    if nargin < 1; spec = 'EKF';
    else spec = varargin{1};
    end
    
    if nargin < 2; N_robots = 1;
    else N_robots = varargin{2};
    end
        
    switch spec
        case 'EKF'
            generator = @()filter_localization2d_ekf();
            exp.display.title = 'Navigation with Extended Kalman Filter';    
        case 'EIF'
            generator = filter_localization2d_eif();
            exp.display.title = 'Navigation with Extended Information Filter';    
        case 'UKF'
            generator = filter_localization2d_ukf();
            exp.display.title = 'Navigation with Unscented Kalman Filter';    
        case {'PF', 'MCL'}
            generator = filter_localization2d_pf();
            exp.display.title = 'Navigation with Particle Filter (Monte-Carlo Localization)';    
        case {'SLAM', 'EKFSLAM'}
            generator = filter_ddrive_ekfslam();
            exp.display.title = 'SLAM with Extended Kalman Filter';    
        otherwise
            error('Unknown navigation filter');
    end

    robotColors = [0 0 1; 1 0 0; 0 1 0; 0.9 0.8 0; 1 0 1; 0 1 1; 1 0.5 0; 0.25 0.5 0.5];    
    
    for i = 1:N_robots
        exp.robots{i}.navigation = generator();
        exp.robots{i}.platform = model_platform2d_on_path_with_inertial_odometry(pathPoints);
        exp.robots{i}.sensors.landmark_detector = sensor_landmarks2d();
        exp.robots{i}.radius = 0.14;
        exp.robots{i}.color = robotColors(mod((i - 1), size(robotColors, 1)) + 1, :);
    end

    % distribute robots on path

    % compute path length and number of subdivisions
    deltas = pathPoints(2:end, :) - pathPoints(1:(end - 1), :);
    lengths = sqrt(sum(deltas.^2, 2));
    accLengths = [0; cumsum(lengths)];

    divisions = N_robots;
    if any(pathPoints(1, :) ~= pathPoints(end, :)), 
        % path not closed -> decrement number of subdivisions, since first 
        % bot should be placed on the start point and the last one at the
        % end point of the path
        divisions = divisions - 1; 
    end

    delta = sum(lengths) / divisions;
    distance = 0;

    for i = 1:N_robots
        % determine the path point, on or after which the robot has to be placed
        pt_idx = find(accLengths <= distance, 1, 'last');

        if pt_idx < size(pathPoints, 1)
            % place before last point
            vec = pathPoints(pt_idx + 1, :) - pathPoints(pt_idx, :);
            angle = atan2(vec(2), vec(1));
            exp.robots{i}.platform.initialPose = [pathPoints(pt_idx, :) + (distance - accLengths(pt_idx)) / norm(vec) * vec, angle];
            exp.robots{i}.platform.firstTargetPoint = pt_idx + 1;
        else
            % on or after (should not happen) the last path point 
            angle = atan2(pathPoints(end, 2) - pathPoints(end - 1, 2), pathPoints(end, 1) - pathPoints(end - 1, 1));			
            exp.robots{i}.platform.initialPose = [pathPoints(end, :), angle];						
            exp.robots{i}.platform.firstTargetPoint = pt_idx;
        end

        distance = distance + delta;		
    end			    
    
	exp.depends = {'*navigation'};
    
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};    
end