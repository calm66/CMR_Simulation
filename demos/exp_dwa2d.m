function exp = exp_dwa2d() 
    exp = experiment_base('dwa2d');
    
    exp.display.title = 'Dynamic Window Approach (DWA)';    
    
    [exp.environment] = grp_obstacles_and_landmarks_from_image('../maps/office.png', ...
                                                               'scale', 0.01);        
    exp.robot.color = [0.2 0.2 1];
    exp.robot.radius = 0.14;
    
    startPos = [1, 4]; % DWA performs pretty well
    %startPos = [1, 2.5]; % DWA gets stuck
    exp.robot.platform = model_platform2d_vomega([startPos, 90 * pi / 180]);
    exp.robot.sensors.rangefinder = sensor_rangefinder2d();
    exp.robot.sensors.rangefinder.color = [0.7 0 0];
    exp.robot.sensors.rangefinder.timing.deltaT = 1 / 15; % better performance at higher sample rates
    
    exp.robot.goal = const_points([3, 5]);
    exp.robot.goal.format = {'Marker', 'x', 'MarkerSize', 15, 'LineWidth', 2};
    
    exp.robot.controller = guidance_dwa2d();
    

    
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};
    
end