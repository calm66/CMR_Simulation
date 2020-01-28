% RRT & RRT* example
%
% This function demonstrates a very basic application of the RRT and RRT*
% randomized global [path] planner.
% Planning is done in a 2D search space with rectangular obstacles. The
% tree nodes are connected by linear segments, i. e. no vehicle dynamic is
% considered here
% From the GUI, you can manually run a fixed number of iterations and watch
% how the tree evolves
%
function rrt_2d()
    % ======= Parameters (free to modify) =================================
    
    % map dimensions
    sizeX = [0 100];
    sizeY = [0 100];
    
    % start position
    start = [50; 90];    
    % goal definition
    goal = [60; 10];
    goalDistance = 0.5; % radius of the goal region
    goalBias = 0.1;     % probability of extending the tree to the goal (instead of a randomly picked sample)
    
    maxExtensionDistance = 4; % max. length of a new branch during tree extension
    neighbourhoodRadius = 2.5 * maxExtensionDistance; % max. neighbour distance for rewiring
    doRewiring = true;  % activate the RRT* rewiring extension
    
    % Initialization of random number generators (The generated sequence
    % can be fixed using a number instead of 'shuffle', which may help to
    % track down problems)
    rngInit = 0;%'shuffle'; 
    obstacleRngInit = 0;%'shuffle';
    
    % number of iterations that are executed automatically after this file has been started
    initialIterations = 100;

    % Parameters for obstacle generation
    nRandomObstacles = 10;
    randObstacleRelSize = [0.01, 0.2]; % size of random obstacles (relative to map dimensions)    
    obstacles = zeros(0, 4);
    % fixed obstacles
    obstacles(end + 1, :) = [25, 20, 50, 20];
    % You may add more fixed obstacles here!
    
    
    % ======= END: Parameters =============================================            
    
    
    % prepare figure 
    f = figure('Renderer', 'OpenGL', 'NumberTitle', 'off', 'Name', 'Simple RRT in 2D', 'Toolbar', 'figure');
    ax = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'DataAspectRatio', [1 1 1], 'XLim', sizeX, 'YLim', sizeY, 'Layer', 'top');
    
    % draw goal region
    rectangle('Parent', ax, 'Position', [goal(1) - goalDistance, goal(2) - goalDistance, 2 * goalDistance, 2 * goalDistance], 'Curvature', [1 1], 'FaceColor', [1 0.5 0.5], 'EdgeColor', 'none');
    patch('Parent', ax, 'XData', goal(1), 'YData', goal(2), 'EdgeColor', 'none', 'Marker', 'x', 'MarkerSize', 10, 'MarkerEdgeColor', [1 0 0], 'LineWidth', 2);
    
    % add random obstacles and draw the map
    nCreatedObstacles = 0;
    rng(obstacleRngInit);
    while nCreatedObstacles < nRandomObstacles
        wRange = randObstacleRelSize * (sizeX(2) - sizeX(1));
        w = rand() * (wRange(2) - wRange(1)) + wRange(1);
        hRange = randObstacleRelSize * (sizeY(2) - sizeY(1));
        h = rand() * (hRange(2) - hRange(1)) + hRange(1);
         
        left = sizeX(1) + rand() * ((sizeX(2) - sizeX(1)) - w);
        bot = sizeY(1) + rand() * ((sizeY(2) - sizeY(1)) - h);
        if start(1) <= left || start(1) >= left + w || start(2) <= bot || start(2) >= bot + h
            obstacles(end + 1, :) = [left, bot, w, h];
            nCreatedObstacles = nCreatedObstacles + 1;
        end
    end    
    for i = 1:size(obstacles, 1)
        rectangle('Parent', ax, 'Position', obstacles(i, :), 'FaceColor', 0.5 * [1 1 1], 'EdgeColor', 'none');
    end
    
    % add ui controls for manual iterations
    pad = 4;
    x = pad;
    uicontrol('style', 'text', 'String', 'More Iterations:', 'Units', 'pixels', 'Position', [x, 8, 120, 16]); x = x + 120 + pad;
    for i = 0:3
        iterations = 10^i;
        uicontrol('style', 'pushbutton', 'String', sprintf('%d', iterations), 'Units', 'pixels', 'Position', [x + i * (48 + pad), 4, 48, 24], 'Callback', @(varargin)iterate(iterations));
    end    

    % initialize the tree with the start position as root node
    root.pos = start;
    root.parent = [];
    root.children = [];
    root.nodeHandle = patch('Parent', ax, 'XData', start(1) - 0.5, 'YData', start(2) - 0.5, 'EdgeColor', 'none', 'Marker', 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.6 0], 'MarkerEdgeColor', [0 0 0]);
    root.trajHandle = [];
    root.cost = 0;
    
    % The tree is stored as a struct array of nodes. Children and parent
    % relations are stored as indices in this array. Since nodes are only
    % added and never removed or reordered, the indices are stable
    % throughout the planning process.
    % The incoming edge is considered part of a node. Every node, except 
    % for the root, stores exactly one edge
    nodes = root;

    % Further helper variables
    goalIdx = [];       % index (into the nodes array) of the currently selected goal node
    goalNodes = [];     % indices (into the nodes array) of all nodes within the goal region
    
    % Line styles used to display branches (edges) that are|are not part of 
    % the solution path
    goalTrajStyle = {'Color', 0.8 * [1 0 0], 'LineWidth', 2};
    normalTrajStyle = {'Color', [0 0 0], 'LineWidth', 1};    
    
    % random number generator for iterations
    rng(rngInit);
    
    % If requested, do some initial iterations
    iterate(initialIterations);
    
    % ======= End of initialization =======================================
    
    % Perform the requested number of RRT(*) iterations
    function iterate(iterations)        
        for iter = 1:iterations
            % draw a sample
            if rand() < goalBias && isempty(goalNodes)
                % goal bias: use the goal position as sample
                sample = goal;
            else
                % pick a random position within the map
                sample = [sizeX(1) + rand() * (sizeX(2) - sizeX(1)); ...
                          sizeY(1) + rand() * (sizeY(2) - sizeY(1))];
            end
            
            % Determine the (euclidian) distance from the sample to all 
            % nodes in the tree...
            allNodePositions = [nodes.pos];
            distSq = sum((allNodePositions - repmat(sample, 1, size(nodes, 2))).^2);
            % ...and pick the closest node
            [minDistSq, minIdx] = min(distSq);                
            nearPos = nodes(minIdx).pos;
            
            % determine the position of the potential new node
            segmentLength = min(sqrt(minDistSq), maxExtensionDistance);                        
            delta = sample - nearPos;        
            newPos = nearPos + segmentLength * delta / norm(delta);
            
            if ~collisionCheck(nearPos, newPos, obstacles)            
                % If the connection to the new node is not blocked by
                % obstacles, add it to the tree
                newIdx = length(nodes) + 1;
                nodes(newIdx).pos = newPos;                
                nodes(newIdx).parent = minIdx;
                nodes(newIdx).cost = nodes(minIdx).cost + norm(newPos - nearPos);
                nodes(minIdx).children(end + 1) = newIdx;
                % Check, if the new node is in the goal region
                if norm(newPos - goal) < goalDistance
                    goalNodes(end + 1) = newIdx;
                end
                
                % Add the new branch to the figure
                nodes(newIdx).trajHandle = line('Parent', ax, 'XData', [nearPos(1), newPos(1)], 'YData', [nearPos(2), newPos(2)], 'Color', [0 0 0]);                                
                
                % RRT* part: (omit until we got an initial solution)
                if ~isempty(goalNodes) && doRewiring
                    % check nodes in neighbourhood for better connections
                    distSq = sum((allNodePositions - repmat(newPos, 1, size(nodes, 2) - 1)).^2);
                    idxNeighbours = find(distSq < neighbourhoodRadius * neighbourhoodRadius);                    
                    % sort neighbours according to their (euclidian) distance
                    neighbourDistsSq = distSq(idxNeighbours);
                    [~, sortedIdx] = sort(neighbourDistsSq);                    
                    idxNeighbours = idxNeighbours(sortedIdx);
                    % check each neighbour for better (a.k.a. shorter) connections via the newly added node
                    
                    ancestors = getNodeSequence(newIdx);
                    for idxN = idxNeighbours
                        % omit any ancestor of the new node (which would result in an unconnected graph)
                        if any(idxN == ancestors); continue; end
                        
                        % compute path length via the new node
                        nPos = nodes(idxN).pos;
                        newCost = nodes(newIdx).cost + norm(newPos - nPos);

                        if nodes(newIdx).cost + norm(newPos - nPos) < nodes(idxN).cost
                            if ~collisionCheck(newPos, nPos, obstacles)
                                % If path length can be reduced and the new connection is not blocked by obstacles, rewire the neighbour.
                                nodes(nodes(idxN).parent).children = setdiff(nodes(nodes(idxN).parent).children, idxN);
                                nodes(newIdx).children(end + 1) = idxN;
                                nodes(idxN).parent = newIdx;                                
                                % update the neighbour's incoming edge in the figure
                                set(nodes(idxN).trajHandle, 'XData', [newPos(1), nPos(1)], 'YData', [newPos(2), nPos(2)]);
                                % propagate cost reduction recursively to the leafs
                                reduceCostRecursive(idxN, nodes(idxN).cost - newCost);
                            end
                        end
                    end
                end
                
                % If any node is located within the goal region, select the
                % best one and mark it as goal in the figure.
                % This will also also highlight the branches of the goal
                % path
                if ~isempty(goalNodes)
                    goalCosts = [nodes(goalNodes).cost];
                    [~, bestGoalCostIdx] = min(goalCosts);
                    setGoal(goalNodes(bestGoalCostIdx));
                end
            end
        end
        
        % Show some information in the title
        titleText = sprintf('Number of nodes: %d', length(nodes));
        if ~isempty(goalIdx)
            titleText = [titleText sprintf(', best path cost (length) = %f\n', nodes(goalIdx).cost)];
        end
        title(titleText);
    end
    
    % function for recursive update of the node(xyz).cost member after a 
    % cost change
    function reduceCostRecursive(idx, delta)
        nodes(idx).cost = nodes(idx).cost - delta;
        for iChild = nodes(idx).children
            reduceCostRecursive(iChild, delta);
        end
    end

    % Mark a node as goal and highlight the corresponding sequence of
    % branches.
    % If another node was previously marked as goal, its appearance will be
    % reset to normal.
    function setGoal(idx)
        if ~isempty(goalIdx)
            set([nodes.trajHandle], normalTrajStyle{:});
        end
        goalIdx = idx;
        if ~isempty(goalIdx)
            set([nodes(getNodeSequence(idx)).trajHandle], goalTrajStyle{:});    
        end
    end
    function path = getNodeSequence(idx)
        path = [];
        while ~isempty(idx)
            path(end + 1) = idx;
            idx = nodes(idx).parent;
        end
    end
end

% Very simple collision checker that can detect intersections between a
% list of rectangles and a straight line. Obviously, a real application
% would use a highly optimized collision checker for maximum performance.
%
% p, q - start and end point of the line segment
% obstacles - Nx4 array, each row = [x, y, w, h] of one rectangular obstacle
%
% return value: true in case of collision, false if not unblocked
%
function b = collisionCheck(p, q, obstacles)    
    for i = 1:size(obstacles, 1)
        left = obstacles(i, 1);
        bot = obstacles(i, 2);
        w = obstacles(i, 3);
        h = obstacles(i, 4);
        right = obstacles(i, 1) + w;        
        top = obstacles(i, 2) + h;
        % First, check if bounding boxes intersect. 
        % If not, continue with next obstacle.
        if p(1) <= left && q(1) <= left; continue; end
        if p(1) >= right && q(1) >= right; continue; end
        if p(2) <= bot && q(2) <= bot; continue; end
        if p(2) >= top && q(2) >= top; continue; end
        
        % Check if either endpoint is inside the rectangle. 
        if p(1) > left && p(1) < right && p(2) > bot && p(2) < top
            b = true;
            return;
        end
        if q(1) > left && q(1) < right && q(2) > bot && q(2) < top
            b  = true;
            return;
        end
        
        % Check for edge intersections
        D = q - p;
        if D(1) ~= 0
            t = ([left, right] - p(1)) / D(1);
            r = (t * D(2) + p(2) - bot) / h;
            if any(t > 0 & t < 1 & r > 0 & r < 1)
                b = true; % intersection with left or right edge
                return;
            end
        end
        if D(2) ~= 0
            t = ([bot, top] - p(2)) / D(2);
            r = (t * D(1) + p(1) - left) / w;
            if any(t > 0 & t < 1 & r > 0 & r < 1)
                b = true; % intersection with top or bottom edge
                return;
            end
        end
    end
    b = false;
end
    