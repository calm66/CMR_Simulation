function exp = exp_gdwa2d() 
    exp = experiment_base('dwa2d');
    
    exp.display.title = 'Global Dynamic Window Approach (DWA) With Focussed D* (FD*)';    
    
    
    [exp.environment, gridMapData] = grp_obstacles_and_landmarks_from_image('../maps/office.png', ...
                                                                            'scale', 0.01);        
    
    exp.robot.color = [0.2 0.2 1];
    exp.robot.radius = 0.14;

    exp.environment.inflatedMap = env_gridmap(gridMapData);
    exp.environment.inflatedMap.color = [1 0 0];
    exp.environment.inflatedMap.alpha = 0.5;
    exp.environment.inflatedMap.inflateRadius = exp.robot.radius + 0.05;
                                                             
    exp.robot.platform = model_platform2d_vomega([1, 0.5, 90 * pi / 180]);
    exp.robot.sensors.rangefinder = sensor_rangefinder2d();
    exp.robot.sensors.rangefinder.color = [0.7 0 0];
    
    exp.robot.goal = const_points([4.5, 5]);
    exp.robot.goal.format = {'Marker', 'x', 'MarkerSize', 15, 'LineWidth', 2};
    
    exp.robot.pathplanner = planner_fdstar2d('environment/inflatedMap');
    exp.robot.controller = guidance_gdwa2d();
    
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};    
end