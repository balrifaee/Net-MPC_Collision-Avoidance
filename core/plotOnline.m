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

function plotOnline(result,step_idx,tick_now)
    iter = result.iterationStructs{step_idx};
    if nargin < 3
        tick_now = (step_idx-1)*result.scenario.ticks_per_sim+1;
    end

    scenario = result.scenario;
    vehiclePositions = result.vehiclePathFullRes(:,:,tick_now);
    obstaclePositions = result.obstaclePathFullRes(:,:,tick_now);
    controlPrediction = result.controlPredictions(:,:,step_idx);
    evaluation = result.evaluations{step_idx};
    
    % add initial position to trajectoryPrediction (not part of the
    % controller output).
    trajectoryPrediction_with_x0 = cat(1,reshape(iter.x0(:,1:2)',1,2,result.scenario.nVeh),result.trajectoryPredictions(:,:,:,step_idx));
    trajectoryPrediction = result.trajectoryPredictions(:,:,:,step_idx);
    delayPrediction = result.MPC_delay_compensation_trajectory(:,:,:,step_idx);

    idx = indices;
    nVeh = scenario.nVeh;
    nObst = size( scenario.obstacles,1);
    Hp = scenario.Hp;

    %% Colors
    colorVeh = vehicleColors();
    
    set(0,'DefaultTextFontname', 'Verdana');
    set(0,'DefaultAxesFontName', 'Verdana');

    
    %% Controller steering inputs
    for v=1:nVeh
        subplot(nVeh,3,(v-1)*3+1);    
        cla
        stairs(0:scenario.Hp-1, 180/pi*controlPrediction(:,v),'Color',colorVeh(v,:) );
        hold on
        plot([0 scenario.Hp-1],180/pi*iter.uMax(v)*[1 1],'k--');
        plot([0 scenario.Hp-1],-180/pi*iter.uMax(v)*[1 1],'k--');
        ylim(1.5*180/pi*scenario.mechanicalSteeringLimit*[-1 1]);
        xlim([0 scenario.Hp-1])
        ylabel(['\fontsize{14}{0}$u_{' num2str(v) '} [^\circ]$'],'Interpreter','LaTex');
        if v == 1
            title('Predicted steering angles','Interpreter','LaTex');
        elseif v == nVeh
            xlabel('Prediction steps','Interpreter','LaTex');
        end
        set(gca,'YTick',180/pi*scenario.mechanicalSteeringLimit*[-1 0 1]);
    end
    
    
    %% Simulation state / scenario plot
    subplot(nVeh,3,[2 3*nVeh]);
    cla
    hold on
    box on
    axis equal
    
    xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
    ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');

    xlim(scenario.plotLimits(1,:));
    ylim(scenario.plotLimits(2,:));
    
    % Sampled trajectory points
    for v=1:nVeh
        plot( iter.ReferenceTrajectoryPoints(:,idx.x,v),iter.ReferenceTrajectoryPoints(:,idx.y,v),'o','MarkerFaceColor',colorVeh(v,:),'MarkerEdgeColor',colorVeh(v,:),'MarkerSize',3 )
    end

    % predicted trajectory
    for v=1:nVeh
        plot( trajectoryPrediction_with_x0(:,idx.x,v),trajectoryPrediction_with_x0(:,idx.y,v),'Color',colorVeh(v,:) );
    end
    
    % vehicle trajectory delay prediction
    for v=1:nVeh
        plot(  delayPrediction(:,idx.x,v),  delayPrediction(:,idx.y,v),'Color',colorVeh(v,:),'LineWidth',2 );
    end

    % Vehicle rectangles
    for v=1:nVeh
        x = vehiclePositions(:,v);
        vehiclePolygon = transformedRectangle(x(idx.x),x(idx.y),x(idx.heading), scenario.Length(v),scenario.Width(v));
        fill(vehiclePolygon(1,:),vehiclePolygon(2,:),colorVeh(v,:));
    end
    
    % Obstacle rectangles
    for i = 1:nObst        
        obstaclePolygon = transformedRectangle(...
            obstaclePositions(i,idx.x),...
            obstaclePositions(i,idx.y),...
            scenario.obstacles(i,idx.heading),...
            scenario.obstacles(i,idx.length),...
            scenario.obstacles(i,idx.width));
        fill(obstaclePolygon(1,:),obstaclePolygon(2,:),[0 0 0]);
    end
    
    % Constraint Violation Markers
    cfg = result.cfg;
    tol = cfg.QCQP.constraintTolerance;
    maxConstraints = rm_dim(max(evaluation.constraintValuesVehicle,[],2),2);
    if nObst > 0
        maxConstraints = max(maxConstraints, rm_dim(max(evaluation.constraintValuesObstacle,[],2),2));
    end    
    violations = maxConstraints>tol;
    for v=1:nVeh
        for k=1:Hp
            if violations(v,k)
                x = trajectoryPrediction(k,1,v);
                y = trajectoryPrediction(k,2,v);
                scatter(x,y,'r*');
            end
        end
    end
    
    [scenarioName, optimizer, strategy]=rename_scenario_optimizer_strategy(result.scenario.Name, result.scenario.controllerName);
    
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (tick_now-1) * result.scenario.tick_length),'Interpreter','LaTex');

    set(t,'HorizontalAlignment', 'center');
    
    drawnow
end

function [scenarioName, optimizer, strategy]=rename_scenario_optimizer_strategy(scenarioName, controllerName)

controllerParts = strsplit(controllerName,' ');
assert(length(controllerParts)==2);
optimizer=controllerParts{1};

if strcmp(controllerParts{2},'PB')
    strategy = 'PB-Non-Coop. DMPC';
elseif strcmp(controllerParts{2},'Centralized')
    strategy = 'Centralized MPC';
else
    strategy = controllerParts{2};
end

[n, count] = sscanf(scenarioName, 'Parallel %i');
if count
    scenarioName = sprintf('%i-Parallel',n);
    return
end

[n, count] = sscanf(scenarioName, 'Circle %i');
if count
    scenarioName = sprintf('%i-Circle',n);
    return
end

if strcmp(scenarioName, '2-way collision (NCD Oscillation)')
    scenarioName = '2-Circle';    
elseif strcmp(scenarioName, 'Crossing PB Example')
    scenarioName = 'Crossing';    
end

end


