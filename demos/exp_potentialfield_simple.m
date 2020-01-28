function exp = exp_potentialfield_simple() 
    exp = experiment_base('potentialfield_simple');
    
    exp.goal = const_points([6, 5]);
    exp.goal.format = {'Color', [0 0 1], 'Marker', 'p', 'MarkerSize', 15};
    
    exp.robot = robot_potentialfield([1, 1]);
	
	uShapedScenario = false;		
	if uShapedScenario
		exp.robot.obstacles = repmat(struct(), 8, 1);
		[exp.robot.obstacles.pos] = deal([4.5, 1.5], [5, 2], [5.5, 2.5], [5, 3], [4.5, 3.5], [4, 4], [3.5, 3.5], [3, 3]);
		[exp.robot.obstacles.radius] = deal(0.15);
	else
		% use default initialization
	end
	
	exp.robot.randomObstacles = 10;				
	exp.robot.maxObstacleVelocity = 1;
	exp.robot.kAttractive = 1;
	%exp.robot.kAttractive = 2000; % works with the default obstacle set
    
    exp.depends = {'robot'};
    
    exp.display.title = 'Simple Potential Field method test';       
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};
end