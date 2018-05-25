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

% Evalute the performance of any controller in the original problem formulation at a single point in time.
function [ evaluation ] = evaluateInOriginalProblem( scenario, iter, controlPrediction, trajectoryPrediction,options)

    evaluation = struct;
    evaluation.predictionObjectiveValueX = 0;
    evaluation.predictionObjectiveValueDU = 0;
    
    % trajectory Prediction  error term
    sqRefErr = (iter.ReferenceTrajectoryPoints-trajectoryPrediction).^2;    
    for v=1:scenario.nVeh
        evaluation.predictionObjectiveValueX = evaluation.predictionObjectiveValueX + ...
            scenario.Q(v) * sum(sum(sum(sqRefErr(1:end-1,:,v)))) + ... 
            scenario.Q_final(v) * sum(sum(sum(sqRefErr(end,:,v))));
    end
      
    % steering Prediction term
    du = diff([iter.u0';controlPrediction]);
    du = du(1:scenario.Hu,:);
    sqDeltaU = du.^2;
    for v=1:scenario.nVeh
        evaluation.predictionObjectiveValueDU = evaluation.predictionObjectiveValueDU + ...
            scenario.R(v) * sum(sqDeltaU(:,v));
    end    
    
    evaluation.predictionObjectiveValue = ...
        evaluation.predictionObjectiveValueX + evaluation.predictionObjectiveValueDU;    
    
    % crash prediction check based on QCQP
    mpc = generate_mpc_matrices( scenario,iter );
    qcqp = convert_to_QCQP( scenario,iter,mpc );
    du = reshape(du,numel(du),1);    
    [ evaluation.predictionFeasibleQCQP,~, ~, ~, ~, ~, evaluation.constraintValuesVehicleQCQP, evaluation.constraintValuesObstacleQCQP ] = QCQP_evaluate( scenario, qcqp, du);
    
    evaluation.constraintValuesVehicle_trajPred = zeros(scenario.nVeh,scenario.nVeh,scenario.Hp);
    evaluation.constraintValuesObstacle_trajPred = zeros(scenario.nVeh,scenario.nObst,scenario.Hp);
    
    % crash prediction check based on the predicted trajectory
    cfg = config;
    evaluation.predictionFeasible_trajPred = true;
    for k=1:scenario.Hp
        for v=1:scenario.nVeh
            for v2=(v+1):scenario.nVeh
                veh_dist_sq = sum((trajectoryPrediction(k,:,v)-trajectoryPrediction(k,:,v2)).^2);
                ci = scenario.dsafeVehicles(v,v2)^2 - veh_dist_sq;
                evaluation.constraintValuesVehicle_trajPred(v,v2,k) = ci;
                evaluation.constraintValuesVehicle_trajPred(v2,v,k) = ci;
                if(ci > cfg.QCQP.constraintTolerance)
                    evaluation.predictionFeasible_trajPred = false;
                end
            end
            for o=1:scenario.nObst
                obst_dist_sq = sum((trajectoryPrediction(k,:,v)-iter.obstacleFutureTrajectories(o,:,k)).^2);
                ci = scenario.dsafeObstacles(v,o)^2 - obst_dist_sq;
                evaluation.constraintValuesObstacle_trajPred(v,o,k) = ci;
                if(ci > cfg.QCQP.constraintTolerance)
                    evaluation.predictionFeasible_trajPred = false;
                end
            end
        end
    end
    
    if ~isfield(options, 'ignoreQCQPcheck')
        if(evaluation.predictionFeasibleQCQP~=evaluation.predictionFeasible_trajPred)
            fprintf('feasibility criteria disagree\n');
        end
    end
    
    evaluation.predictionFeasible = evaluation.predictionFeasible_trajPred;
    evaluation.constraintValuesVehicle = evaluation.constraintValuesVehicle_trajPred;
    evaluation.constraintValuesObstacle = evaluation.constraintValuesObstacle_trajPred;
end