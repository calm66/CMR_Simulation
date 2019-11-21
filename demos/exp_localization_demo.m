% Perform Localization & SLAM experiments with a single or multiple robots
% Inputs:
%  - filter algorithm: available localization algorithms: 'EKF', 'EIF', 'UKF', 'PF'|'MCL'
%                      available SLAM algorithms: 'EKFSLAM'|'SLAM'
%                      default: 'EKF'
%  - number of robots: default = 1

function exp = exp_localization_demo(varargin) 
    exp = experiment_base('LocalizationDemo');
        
    exp.environment = grp_obstacles_and_landmarks_from_image('../maps/emptyroom.png', 'scale', 0.01);    
    pathPoints = [2.00, 1.00;...
                  2.00, 4.20;...
                  6.00, 4.20;...
                  6.00, 1.00;...
                  2.00, 1.00];  
    
    exp.robot.navigation = localization2d_deadreckoning();
    exp.robot.navigation.color = [1 0 0];
    exp.robot.navigation2 = localization2d_rangebearing();
    exp.robot.navigation2.color = [0 1 0];
    exp.robot.platform = model_platform2d_on_path_with_inertial_odometry(pathPoints);
    exp.robot.platform.odometryError = [0.01, 0.01, (0.5 * pi / 180)];
    exp.robot.sensors.landmark_detector = sensor_landmarks2d();
    exp.robot.sensors.landmark_detector.bearingError = 5 * pi / 180;        
    exp.robot.sensors.landmark_detector.rangeError = 1 / 100;			   
    exp.robot.sensors.landmark_detector.range = 15;
    exp.robot.sensors.landmark_detector.fieldOfView = [-pi, pi];
    
    exp.robot.radius = 0.14;    
    
	exp.depends = {'*navigation*'};
                  
    exp.display.title = 'Navigation with Dead Reckoning Localization';   
    
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};    
end