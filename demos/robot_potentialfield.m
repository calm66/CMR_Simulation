function rob = robot_potentialfield(varargin)
	rob = block_base(1/50, {'goal'}, @pfStep);
    
    rob.figures(end + 1).name = 'Potential Field internals';
    rob.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'radar.png');
    rob.figures(end).init = @createFigure;
    rob.figures(end).draw = @updateFigure;                    
    
	rob.graphicElements(end + 1).name = 'Potential';
	rob.graphicElements(end).draw = @drawPotential;
	rob.graphicElements(end + 1).name = 'Objects';
	rob.graphicElements(end).draw = @draw;
	
	% Parameters for the robot
	rob.default_initialPos = [0; 0];
    if nargin >= 1
        rob.initialPos = varargin{1}(:)';
	end	
    rob.default_radius = 0.15;
    rob.default_maxVelocity = 0.7;
	
	rob.default_kAttractive = 1;
	rob.default_kRepulsive = 1;
    rob.default_d0 = 0.5; % maximum influence distance for obstacles

    rob.default_workspaceSize = [8, 6];	
		
	% default set of obstacles
	obstacles(1).pos = [3 3];
	obstacles(1).radius = 0.5;
	obstacles(2).pos = [4 2];
	obstacles(2).radius = 0.25;
	obstacles(3).pos = [4.5 3.5];
	obstacles(3).radius = 0.35;
	obstacles(4).pos = [5.5 3.5];
	obstacles(4).radius = 0.25;            	
	rob.default_obstacles = obstacles;
	
	% do not create additional random obstacles by default
	rob.default_randomObstacles = 0;
	
	% motion parameters for the random obstacles
	rob.default_maxObstacleVelocity = 0.8;
	rob.default_maxObstacleOmega = 360 * pi / 180;
	
    
    % drawing callbacks
    function handles = draw(block, ax, handles, out, debugOut, state, goal)        
        if isempty(handles)             
            handles.bot = patch('Parent', ax, 'XData', [], 'YData', [], 'EdgeColor', [0, 0, 0], 'FaceColor', [0 0 1], 'FaceAlpha', 0.5);
            
            handles.obstacles = zeros(size(state.obstacles));
			for i = 1:numel(state.obstacles)
				handles.obstacles(i) = patch('Parent', ax, 'XData', [], 'YData', [], 'EdgeColor', 0.5 * [1 0 0], 'FaceColor', [1 0 0]);
			end
			handles.direction = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0 0.7 0], 'LineWidth', 2);
        end

        arcs = linspace(0, 2 * pi, 36);
        set(handles.bot, 'XData', state.pos(1) + block.radius * cos(arcs), 'YData', state.pos(2) + block.radius * sin(arcs));                 		

		for i = 1:length(state.obstacles)
			r = state.obstacles(i).radius;
			set(handles.obstacles(i), 'XData', state.obstacles(i).pos(1) + r * cos(arcs).', ...
									  'YData', state.obstacles(i).pos(2) + r * sin(arcs).');
		end
			
		forceDir = atan2(state.force(2), state.force(1));
		dirLength = 0.25;
		set(handles.direction, 'XData', state.pos(1) + [0, dirLength * cos(forceDir)], 'YData', state.pos(2) + [0, dirLength * sin(forceDir)]);
	end
	function handles = drawPotential(block, ax, handles, out, debugOut, state, goal)
		if isempty(handles)
			hold on;
			handles.hPotential = surf('Parent', ax, 'XData', [] , 'YData', [], 'ZData', [], 'CData', [], 'LineStyle', 'none');
			hold off;			
			colormap(ax, 'gray');
			%colormap(ax, 'lines');
		end
		[X, Y, U] = computePotential(block, state, goal(end).data);
		U = log(1 + U);
		maxU = max(max(U));
		minU = min(min(U));
		U = (U - minU) / (maxU - minU);
		set(handles.hPotential, 'XData', X, 'YData', Y, 'ZData', zeros(size(X)), 'CData', 1 - U);%repmat(1 - U, [1 1 3]));		
	end

    % callbacks for the 'Internals' visualization window
    function [f, diag] = createFigure(block, blockName)
		f = figure('Name', ['VFH internals for ' blockName], 'NumberTitle', 'off');
		diag.ax = axes('XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on');
		 
		hold on;
		diag.hPotential = surf('Parent', diag.ax, 'XData', [], 'YData', [], 'ZData', [], 'LineStyle', 'none');
		colormap(jet)
        hold off;
		view([-6 46]);
        colorbar;
		rotate3d();
    end
    function diag = updateFigure(block, f, diag, out, debugOut, state, goal)
		if ~isempty(state) && ~isempty(goal)
			[X, Y, U] = computePotential(block, state, goal(end).data);
			set(diag.hPotential, 'XData', X, 'YData', Y, 'ZData', log(U + 1));			
		end
    end


    function [state, out, debugOut] = pfStep(block, t, state, goal)
        debugOut = [];
        out = 0;
        if isempty(state)
            state.t = 0;
			state.pos = block.initialPos(:);
			state.obstacles = block.obstacles;			
			for i = 1:block.randomObstacles
				state.obstacles(end + 1).pos = [block.workspaceSize .* rand(1, 2)];
				state.obstacles(end).radius = 0.1 + 0.3 * rand();
			end
			for i = 1:numel(state.obstacles)
				state.obstacles(i).dir = 2 * pi * rand;
			end			
        end
           
        if ~isempty(goal)
            goal = goal(end).data;
            
            deltaGoal = [state.pos(1) - goal(1); state.pos(2) - goal(2)];
            attractivePotential = block.kAttractive * deltaGoal;                                    
            
            force = -attractivePotential;
            for i = 1:length(state.obstacles)
                deltaObst = [state.pos(1) - state.obstacles(i).pos(1); state.pos(2) - state.obstacles(i).pos(2)];
                dObst = sqrt(sum(deltaObst.^2));
                deltaObstDir = deltaObst / dObst;
                dObst = dObst - state.obstacles(i).radius - block.radius;
                d0 = block.d0;
                if dObst <= d0
                	force = force + block.kRepulsive * (1 / dObst - 1 / d0) * 1 / dObst^2 * deltaObstDir;
                end
			end
            
			T = t - state.t;
			
            state.force = force;
            forceDir = atan2(force(2), force(1));            
			forceStrength = min(block.maxVelocity * T, sqrt(sum(force.^2)));            
            state.pos = state.pos + forceStrength * [cos(forceDir); sin(forceDir)];
			
			
			for i = 1:numel(state.obstacles)
				omega = 2 * block.maxObstacleOmega * (rand() - 0.5);
				dir = state.obstacles(i).dir + omega * T;
				state.obstacles(i).dir = dir;
				dist = block.maxObstacleVelocity * T * rand();
				pos = state.obstacles(i).pos + dist * [cos(dir), sin(dir)];
				pos(1) = max(0, min(pos(1), block.workspaceSize(1)));
				pos(2) = max(0, min(pos(2), block.workspaceSize(2)));
				state.obstacles(i).pos = pos;
			end
			
			state.t = t;
		end
    end
end

function [X, Y, U] = computePotential(block, state, goal)
	resolution = 0.05;
	[X, Y] = meshgrid(0:resolution:block.workspaceSize(1), 0:resolution:block.workspaceSize(2));

	deltaX = X - goal(1) + 0.5 * resolution;
	deltaY = Y - goal(2) + 0.5 * resolution;

	U_attractive = 0.5 * block.kAttractive * (deltaX.^2 + deltaY.^2);
	U = U_attractive;

 	for i = 1:length(state.obstacles)
		dX = X - state.obstacles(i).pos(1) + 0.5 * resolution;
		dY = Y - state.obstacles(i).pos(2) + 0.5 * resolution;
		d = sqrt(dX.^2 + dY.^2);

		d = d - block.radius - state.obstacles(i).radius;

		U_rep = zeros(size(X));
		insideIdx = d <= 0;
		inRangeIdx = d <= block.d0;
		affectedIdx = inRangeIdx & ~insideIdx;
		U_rep(affectedIdx) = 0.5 * block.kRepulsive * (1 ./ d(affectedIdx) - 1 / block.d0).^2;
		U_rep(insideIdx) = inf;				
		U = U + U_rep;
	end
	% heuristic limit
	maxU = max(max(U_attractive));
	U(U > maxU) = maxU;
end