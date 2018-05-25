% Copyright 2016 Bassam Alrifaee
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as 
% published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

% Run a closed-loop simulation with a controller.
function result=runSimulation(scenarioConstructor, controller, controllerName, doOnlinePlot)
    
    % Plot controls: SPACE to pause, ESC to abort.
    paused = false;
    abort = false;
    function keyPressCallback(~,eventdata)
        if strcmp(eventdata.Key, 'escape')
            abort = true;
        elseif strcmp(eventdata.Key, 'space')
            paused = ~paused;
        end
    end
    if doOnlinePlot
        % Open figure for online plot.
        figure('units','normalized','outerposition',[.05 .05 .9 .9]);
        set(gcf,'WindowKeyPressFcn',@keyPressCallback);
    end
        
    rng default; % reset RNG
    
    % Container struct
    result = struct;
    
    % Create the result.scenario.
    result.scenario = scenarioConstructor();
    result.scenario.controllerName = controllerName;    
    result.scenario = completeScenario(result.scenario);
    
        
    % Variables and Parameters
    Hp   = result.scenario.Hp;
    Nsim = result.scenario.Nsim;
    dt   = result.scenario.dt;
    nVeh = result.scenario.nVeh;
    nObst = result.scenario.nObst;
    nx   = result.scenario.model.nx;
    nu   = result.scenario.model.nu;
    assert(nu == 1); % if that ever changes, the code needs to be fixed in many places as this assumption was often implicitly made.
    ny   = result.scenario.model.ny;
    result.cfg = config;
    idx = indices;
    
    
    % Result variables
    result.controllerOutputs = cell(Nsim,1);
    result.iterationStructs = cell(Nsim,1);
    result.obstaclePathFullRes = nan(nObst,2,result.scenario.ticks_total+1);
    result.vehiclePathFullRes = nan(nx,nVeh,result.scenario.ticks_total+1);
    result.controlPathFullRes = nan(nVeh,result.scenario.ticks_total+1);
    result.controlPredictions = zeros(Hp,nVeh,Nsim);
    result.trajectoryPredictions = zeros(Hp,ny,nVeh,Nsim);
    result.controllerRuntime = zeros(Nsim,1);
    result.stepTime = zeros(Nsim,1);
    result.evaluations = cell(Nsim,1);
    result.steeringLimitsExceeded = false;
    
    % Calculate obstacle trajectories
    for tick=0:result.scenario.ticks_total
        result.obstaclePathFullRes(:,idx.x,tick+1) = (tick*result.scenario.tick_length)*result.scenario.obstacles(:,idx.speed).*cos( result.scenario.obstacles(:,idx.heading) ) + result.scenario.obstacles(:,idx.x);
        result.obstaclePathFullRes(:,idx.y,tick+1) = (tick*result.scenario.tick_length)*result.scenario.obstacles(:,idx.speed).*sin( result.scenario.obstacles(:,idx.heading) ) + result.scenario.obstacles(:,idx.y);
    end
    
    % Initialize the first step.
    for v=1:nVeh
        result.vehiclePathFullRes(:,v,1) = result.scenario.x0(v,:);
        result.controlPathFullRes(v,1:result.scenario.ticks_delay_u+result.scenario.ticks_per_sim+1) = result.scenario.u0(v,:);
    end
    
    % create output directory
    directoryName = strrep([result.scenario.Name, '_', result.scenario.controllerName], ' ', '_');
    outputPath = ['output\', directoryName ,'\'];
    result.outputPath = outputPath;
    if ~isdir('output')
        mkdir('output');
    end
    if ~isdir(outputPath)
        mkdir(outputPath);
    end
    
    % log output
    diary off
    diary([outputPath 'log.txt'])
    
    % Simulation loop    
    disp('Beginning simulation...');
    for i = 1:Nsim        
        result.stepTimer = tic;
        tick_now = (i-1)*result.scenario.ticks_per_sim+1; % tick index of the current moment.
        tick_of_measurement = max(1,tick_now - result.scenario.ticks_delay_x); % tick index from a past moment, where the state x was measured.
        tick_of_actucator = min(result.scenario.ticks_total+1,tick_now +result.scenario.ticks_delay_u+result.scenario.ticks_per_sim); % tick index in the future when the result of a MPC calculation that starts now is applied.
        
        
        % determine controller inputs
        uMax = zeros(1,nVeh);
        for v=1:nVeh
            % Steering constraints: At high speeds lateral acceleration
            % needs to be limited.
            dynamicSteeringLimit = atan((result.scenario.lateralAccelerationLimit*(result.scenario.Lf(v)+result.scenario.Lr(v)))/(result.vehiclePathFullRes(idx.speed,v,tick_now))^2);
            uMax(v) = min(result.scenario.mechanicalSteeringLimit,dynamicSteeringLimit);
        end
        
        x_measured = result.vehiclePathFullRes(:,:,tick_of_measurement)';        
        % Extract the control output values for the timeslice that starts
        % at (T_now - T_x) and ends at (T_now + T_MPC + T_u).
        % The index slicing is a little confusing because of the truncation
        % to the simulation timespan:
        % In the beginning of the simulation there are no control outputs,
        % so they are assumed to be zero.
        u_path = zeros(nVeh,result.scenario.ticks_delay_x + result.scenario.ticks_per_sim + result.scenario.ticks_delay_u);        
        u_path(:,max(result.scenario.ticks_delay_x-tick_now,1)+(0:tick_of_actucator-1-tick_of_measurement)) = result.controlPathFullRes(:,tick_of_measurement+1:tick_of_actucator);
        
        % call controller
        controllerTimer = tic;        
            % MPC_init is part of the controller and thus inside 'controllerTimer'.
            % since it is a preprocessing step necessary for every MPC
            % controller, it is called here.
            [iter, result.MPC_delay_compensation_trajectory(:,:,:,i)] = MPC_init(result.scenario, x_measured, u_path, result.obstaclePathFullRes(:,:,tick_of_measurement), uMax); 
            iter.reset = i == 1;
            [U,result.trajectoryPredictions(:,:,:,i),result.controllerOutputs{i}] = controller( result.scenario,iter,result.controllerOutputs{max(1,i-1)} );        
        result.controllerRuntime(i)=toc(controllerTimer);
        
        % check steering constraints
        for v=1:nVeh
            if(abs(U(1,v)) > iter.uMax(v)+1e-3)
                fprintf('Steering limit exceeded for vehicle %i: |%d|>%d\n',v,U(1,v),iter.uMax(v));
                result.steeringLimitsExceeded = true;
            end
            if(abs(U(1,v)-iter.u0(v)) > result.scenario.duLim+1e-3)
                fprintf('Steering rate exceeded for vehicle %i: |%d|>%d\n',v,U(1,v)-iter.u0(v),result.scenario.duLim);
                result.steeringLimitsExceeded = true;
            end
            for j=2:Hp
                if(abs(U(j,v)) > iter.uMax(v)+1e-3)
                    fprintf('Steering limit exceeded for vehicle %i: |%d|>%d\n',v,U(j,v),iter.uMax(v));
                 result.steeringLimitsExceeded = true;
                end
                if(abs(U(j,v)-U(j-1,v)) > result.scenario.duLim+1e-3)
                    fprintf('Steering rate exceeded for vehicle %i: |%d|>%d\n',v,U(j,v)-U(j-1,v),result.scenario.duLim);
                    result.steeringLimitsExceeded = true;
                end
            end
        end
        
        % enforce steering constraints
        for v=1:nVeh
            U(1,v) = min(U(1,v), iter.uMax(v));
            U(1,v) = max(U(1,v), -iter.uMax(v));
            U(1,v) = min(U(1,v), iter.u0(v,1)+result.scenario.duLim);
            U(1,v) = max(U(1,v), iter.u0(v,1)-result.scenario.duLim);            
            for j=2:Hp
                U(j,v) = min(U(j,v), iter.uMax(v));
                U(j,v) = max(U(j,v), -iter.uMax(v));
                U(j,v) = min(U(j,v), U(j-1,v)+result.scenario.duLim);
                U(j,v) = max(U(j,v), U(j-1,v)-result.scenario.duLim);                
            end
        end
        
        for v=1:nVeh
            % Save the controller's signal. The controller's signal is
            % saved with a shift of 'ticks_per_sim+ticks_delay_u' into the future. This
            % is done to simulate controller and actuator delay: It takes
            % 'ticks_per_sim' to execute the MPC controller and 'ticks_delay_u'
            % for the signal to propagate to the actual steering angle.
            
            result.controlPathFullRes(v,min( size(result.controlPathFullRes,2) ,  blk(i,result.scenario.ticks_per_sim)+1  +result.scenario.ticks_delay_u+result.scenario.ticks_per_sim) ) = U(1,v);
                        
            % simulate with the controller's signal
            [~,model_step] = ode45(@(t,x) result.scenario.model.ode(x, result.controlPathFullRes(v,min(size(result.controlPathFullRes,2),ceil(t/result.scenario.tick_length)+1)),result.scenario.Lf(v),result.scenario.Lr(v)),...
                linspace((i-1)*dt,i*dt,result.scenario.ticks_per_sim+1), result.vehiclePathFullRes(:,v,tick_now), odeset('RelTol',1e-8,'AbsTol',1e-8));
            result.vehiclePathFullRes(:,v,blk(i,result.scenario.ticks_per_sim)+1) = model_step(2:end,:)';
        end
        result.controlPredictions(:,:,i) = U;
                
        % Check if result.scenario is initially feasible
        if i==1 && ~QCQP_evaluate( result.scenario, convert_to_QCQP( result.scenario,iter,generate_mpc_matrices( result.scenario,iter ) ), zeros(result.scenario.nVeh*result.scenario.Hu,1));
            error('scenario initially infeasible!')
        end
        
        result.evaluations{i} = evaluateInOriginalProblem( result.scenario, iter, U, result.trajectoryPredictions(:,:,:,i), struct('ignoreQCQPcheck', true) );
             
        result.iterationStructs{i} = iter;
        
        if doOnlinePlot
            plotOnline(result,i);
        end
        
        % idle while paused, and check if we should stop early
        while paused
            pause(0.1);
            if abort
                close all;
                disp('Aborted.');
                diary off
                return;
            end
        end
        if abort
            close all;
            disp('Aborted.');
            diary off
            return;
        end
        
        result.stepTime(i) = toc(result.stepTimer);
    end
    % End of simulation loop
    close all;
    
    %% save results
    save([outputPath 'data.mat'],'result');
    
    disp('Done.');
    diary off
end