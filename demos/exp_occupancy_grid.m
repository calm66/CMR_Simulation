% Experiment setup for Exercise 2: 
% Extende Kalman Filter based localization of a Differential Drive Mobile
% Robot
function exp = exp_occupancy_grid() 
    exp = experiment_base('occupancy_grid');
    
    % Some configuration options for the experiment   
    poseProvider = 'exact'; % use 'exact', 'ekf' or 'slam'
    
    % Prepare map and path for office environment experiment setup
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
    
              
    % The first path point is used as initial pose
    exp.robot.initialPose = [pathPoints(1, :), 90 * pi / 180];
    
    % instantiate the platform (differential drive) and set its parameters
    exp.robot.platform = model_platform2d_ddrive();
    exp.robot.radius = 0.14;
    exp.robot.color = [0 0 1];
    exp.robot.wheelRadius = [0.03 0.03];
    exp.robot.wheelDistance = 0.25;    
        
    % add the controller (which uses the 'platform' as input by default)
    exp.robot.controller = controller_ddrive_follow_path(pathPoints);
    
    % add the sensors
    exp.robot.sensors.rangefinder = sensor_rangefinder2d();
    exp.robot.maxRange = 3;
    
    exp.robot.sensors.odometer = sensor_odometer_wheelspeed();
    exp.robot.odometryError = 10 * pi / 180;        

    exp.robot.sensors.landmark_detector = sensor_landmarks2d();    
    exp.robot.sensors.landmark_detector.fieldOfView = 70 * pi / 180 * [-1 1];
    exp.robot.bearingError = 1 * pi / 180;
    exp.robot.rangeError = 1 / 100;

    % Instantiate the Occupancy Grid mapper
    exp.robot.occupancyGrid = map_occupancy_grid([8 6]);    
    
    
    switch poseProvider
    case 'slam'    
        % localization with EKF-SLAM
        exp.robot.slam = filter_ddrive_ekfslam();
        % By default, the SLAM filter is triggered by the landmark
        % detector, which is not synchronized to the rangefinder. Therefore
        % the pose information used during mapping may be out-of-date and
        % rather large mapping errors may arise (especially during turns).
        % Synchronizing the timings of the SLAM filter to the mapper will
        % reduce these errors
        exp.robot.slam.timing = exp.robot.occupancyGrid.timing;
        exp.robot.slamPose = block_extract('slam', 'pose');
        exp.robot.occupancyGrid.depends{1} = 'slamPose';
    case 'ekf'
        % localization from landmarks with EKF
        exp.robot.localization = filter_ddrive_ekf();
        exp.robot.localization.timing = exp.robot.occupancyGrid.timing;
		%exp.robot.localization.useRange = false;
		%exp.robot.localization.useBearing = false;
        exp.robot.localizationPose = block_extract('localization', 'pose');
        exp.robot.occupancyGrid.depends{1} = 'localizationPose';
    case 'exact'
        exp.robot.occupanceGrid.depends{1} = 'platform';

    otherwise
        error('Invalid pose provider');            
    end
	exp.depends = {'*occupancyGrid'};
    
    exp.display.title = 'Occupancy Grid mapping with laser rangefinder';       
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};
end