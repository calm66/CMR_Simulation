function p = planner_fdstar2d(varargin)
    if nargin == 0
        mapInput = 'environment/map';
    else mapInput = varargin{1};
    end
    p = block_base(0, {mapInput, 'platform', 'goal'}, @plan);
    
    p.mexFiles{end + 1}.file = fullfile(fileparts(mfilename('fullpath')), 'mex_fdstarplanner.cpp');
    p.mexFiles{end}.dependencies = {'fdstarplanner.h', 'fdstarplanner.cpp'};
    
    p.graphicElements(end + 1).draw = @drawPath;
    p.graphicElements(end).name = 'Planned path';
    
    p.default_color = [0 0 1];
    p.default_simplifyPath = true;
    mexPlanner = [];
    
    function handles = drawPath(block, ax, handles, out, debug, state, varargin)        
        if isempty(handles); 
            handles = line('Parent', ax, 'XData', [], 'YData', [], 'Color', block.color, 'Marker', '.');
        end        
        set(handles, 'XData', out(:, 1), 'YData', out(:, 2));
    end

    % output format: path as Mx2 array, one point per row
    function [state, out, debugOut] = plan(block, t, state, map, platform, goal)
        debugOut = [];
        if isempty(mexPlanner); mexPlanner = mex_object_handle(@mex_fdstarplanner); end
        
        args = struct();
        gridMap = map(end).data;
        
        if isempty(state); state = [-inf -inf]; end
        
        if state(1) ~= map(end).t            
            state(1) = map(end).t;
            args.map = gridMap.obstacles;
            args.obstacleValue = true;
        end
        
        if ~isempty(platform)
            args.start = (getPos(platform(end).data) - gridMap.offset) / gridMap.scale;
        end
        if ~isempty(goal)
            if state(2) ~= goal(end).t
                state(2) = goal(end).t;
                args.goal = (getPos(goal(end).data) - gridMap.offset) / gridMap.scale;
            end
        end
        
        [path, errMsg] = mexPlanner.invoke('update', args);
        
        if isempty(path)
            warning('Sim:planner_fdstar2d', 'Path planning failed: %s', errMsg);
            out = zeros(0, 2);
        else
            if block.simplifyPath
                lastPoint = path(1, :);
                simplifiedMask = false(size(path, 1), 1);
                lastDir = 0;
                for i = 2:size(path, 1)
                    pt = path(i, :);
                    dir = (pt(1) < lastPoint(1)) + 2 * (pt(1) > lastPoint(1)) + 4 * (pt(2) < lastPoint(2)) + 8 * (pt(2) > lastPoint(2));
                    if dir ~= lastDir
                        simplifiedMask(i - 1) = true;
                        lastDir = dir;
                    end
                    lastPoint = pt;
                end
                simplifiedMask(end) = true;
                path = path(simplifiedMask, :);
            end
            out = ((path + 0.5) * gridMap.scale) + repmat(gridMap.offset, size(path, 1), 1);
        end
    end

    function pos = getPos(arg)
        if isstruct(arg)
            if isfield(arg, 'pose')
                pos = arg.pose(1:2);
            elseif isfield(arg, 'pos')
                pos = arg.pos(1:2);
            else error('Could not determine 2D point coordinates from input');
            end
        else
            pos = arg(1:2);
        end
    end
end

