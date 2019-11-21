function filter = filter_localization2d_pf()
    filter = filter_localization2d(@filterStep); % reuse drawing function from generic localization2d block
    
    filter.default_particleCount = 250; % number of particles          
    filter.default_resampleRatio = 0.9; % ratio of particles to resample from old sample set. The remaining particles are initialized randomly
    filter.default_randomParticleSigma = [0.5, 0.5, 10 * pi / 180]; % sigma for sampling the (1 - resampleRatio) * particleCount randomly initialized particles after each resampling step
    
    filter.graphicElements(end + 1).draw = @drawParticles;
    filter.graphicElements(end).name = 'particles';
    filter.default_initialPoseError = [0.5 0.5 45 * pi / 180];
    
    function handles = drawParticles(block, ax, handles, out, debugOut, state, platform, varargin)        
        if isempty(handles) 
            handles = quiver(ax, [], [], [], [], 'AutoScale', 'on', 'ShowArrowHead', 'off', 'Marker', '.', 'Color', block.color / 2);
        end

        set(handles, 'XData', state(:, 1), 'YData', state(:, 2), ...
                     'UData', 10 * cos(state(:, 3)), 'VData', 10 * sin(state(:, 3)));        
    end

    
    function [state, out, debugOut] = filterStep(block, t, state, platform, sensor, landmarks)
        debugOut = [];
        
        M = block.particleCount;
        
        % the particle set is stored in the state as a <particle_count>x3        
        if isempty(state)                        
            if isempty(block.initialPose)
                pose = platform(1).data.pose;
            else pose = block.initialPose;
            end
            state = repmat(pose, M, 1) + repmat(block.initialPoseError, M, 1) .* randn(M, 3);
            state(:, 3) = mod(state(:, 3) + pi, 2 * pi) - pi; % normalize angles
        end
		
        if t > 0
            % Particle prediction with nonlinear system model
            % The process noise was inflated by a factor of 3 to speed up the 
            % reduction of the filter bias without relying too much on a 
            % particle sampled at the correct location.
            % However, the noise of the extracted estimated pose is increased
            % as well.
            state = state + repmat(platform.data.odometry, M, 1) + ...
                    repmat(3 * block.odometryError, M, 1) .* randn(M, 3);
            state(:, 3) = mod(state(:, 3) + pi, 2 * pi) - pi; % normalize angles        

            visIdx = sensor.data.lmIds; % assume we can associate each measurement with a known landmark
            if ~isempty(visIdx)
                % compute particle weights from measurements

                % to avoid numerical problems, we first compute logarithmic weights
                % weights = prod_over_all_measurements(exp(-0.5 * predicted_measurement - measurement)^2 / sigma^2))
                % -> logWeights = ln(weight) = -0.5 * sum_over_all_measurements((predicted_measurement - measurement)^2 / sigma^2)            
                logWeights = zeros(M, 1);

                if block.useBearing
                    % process bearing measurements, if enabled
                    angles = atan2(repmat(landmarks.data(visIdx, 2)', M, 1) - repmat(state(:, 2), 1, length(visIdx)), ...
                                   repmat(landmarks.data(visIdx, 1)', M, 1) - repmat(state(:, 1), 1, length(visIdx))) - ...
                                   repmat(state(:, 3), 1, length(visIdx));
                    angleDiff = mod(repmat(sensor.data.bearing', M, 1) - angles + pi, 2 * pi) - pi;				
                    logWeights = logWeights - 0.5 / block.bearingError^2 * sum(angleDiff.^2, 2);				
                end
                if block.useRange
                    % process range measurements, if enabled
                    ranges = sqrt((repmat(state(:, 1), 1, length(visIdx)) - repmat(landmarks.data(visIdx, 1)', M, 1)).^2 + ...
                                  (repmat(state(:, 2), 1, length(visIdx)) - repmat(landmarks.data(visIdx, 2)', M, 1)).^2);

                    rangeDiff = repmat(sensor.data.range', M, 1) - ranges;
                    variances = (block.rangeError * sensor.data.range').^2;						
                    logWeights = logWeights - 0.5  * sum(rangeDiff.^2 ./ repmat(variances, M, 1), 2);
                end

                % normalize weights to improve numeric stability (this is not the normalization sum(exp(logWeights)) = 1 !)
                logWeights = logWeights - max(logWeights);

                % convertion of logWeights to "ordinary" weights
                weights = exp(logWeights);
                % now normalized for sum(weights) = 1
                weights = weights / sum(weights);

                % For sake of simplicity we resample in every update step.
                % In a more advanced implementations, one would compute an
                % "effective number of particles" Neff and use it to determine 
                % if resampling is neccessary)
                M_old = block.resampleRatio * M; % number of particles to draw from old sample set (the remaining particles are initilaized randomly)

                % resampling according to Thrun, et. al: Probabilisic Robotics, pp. 110
                r = rand() / M_old;
                take = zeros(1, M_old);
                takeIdx = 1;
                c = weights(1);
                for m = 1:M_old
                    while r > c
                        takeIdx = takeIdx + 1;
                        c = c + weights(takeIdx);
                    end
                    r = r + 1 / M_old;
                    take(m) = takeIdx;
                end
                state = state(take, :);

                % reconstruct gaussian distribution for display purposes
                [out.pose, out.cov] = extractNormalDistribution(state);

                % sample some more random particles Erzeuge noch ein paar zuf�llige Partikel um den Mittelwert herum um Particle Deprivation vorzubeugen
                % und Konvergenz im Kidnapping-Fall zu beschleunigen
                M_new = M - M_old;
                state = [state; ...
                         repmat(out.pose, M_new, 1) + repmat(block.randomParticleSigma, M_new, 1) .* randn(M_new, 3)];
                state((M_old + 1):end, 3) = mod(state((M_old + 1):end, 3) + pi, 2 * pi) - pi;			            

                return;
            end
        end
        
        [out.pose, out.cov] = extractNormalDistribution(state);
    end
end

% Extract parameters of a gaussian distribution from a particle set
% Assume an equally weighted particle set.
function [mu, P] = extractNormalDistribution(particles)
	xy_mean = [mean(cos(particles(:, 3))), mean(sin(particles(:, 3)))];			
	mu = [mean(particles(:, [1 2])), atan2(xy_mean(2), xy_mean(1))];
		
	P = cov([particles(:, [1 2]), mod(particles(:, 3) - mu(3) + pi, 2 * pi) - pi]);
end