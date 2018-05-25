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

function [U,trajectoryPrediction,controllerOutput] = SCP_controller( scenario,iter,prevOutput ) 
    controllerOutput = struct;

    mpc = generate_mpc_matrices( scenario,iter );
    qcqp = convert_to_QCQP( scenario,iter,mpc );

    du = zeros(scenario.nVeh*scenario.Hu,1);

    init_method = 'previous';
    if isfield(scenario, 'SCP_init_method')
        init_method = scenario.SCP_init_method;
    end

    if strcmp(init_method, 'previous') && isstruct(prevOutput) && isfield(prevOutput, 'du')
        du = prevOutput.du;
        du = reshape(du,scenario.Hu,scenario.nVeh);
        du(1:end-1,:)=du(2:end,:);
        du(end,:) = 0;
        du = reshape(du,scenario.Hu*scenario.nVeh,1);
    end
    
    controllerOutput.resultInvalid = false;
    optimizerTimer = tic;
    
    % try last MPC sol.
    [du , feasible , ~, controllerOutput.optimization_log] = SCP_optimizer( scenario,iter,qcqp, mpc, du );
    % heuristic: try left and right if no. of veh. is 1
    if scenario.nVeh == 1
        if ~feasible
            % try left
            du = ones(size(du))*scenario.duLim;
            [du_pos , feasible_pos, ~] = SCP_optimizer( scenario,iter,qcqp, mpc, du );
            if feasible_pos
                du = du_pos;
            else
                % try right
                du = -ones(size(du))*scenario.duLim;
                [du_neg , feasible_neg, ~] = SCP_optimizer( scenario,iter,qcqp, mpc, du );
                if feasible_neg
                    du = du_neg;
                else
                    disp('INFEASIBLE PROBLEM')
                    controllerOutput.resultInvalid = true;
                end
            end
        end
    end
    
    
    controllerOutput.du=du;
    [trajectoryPrediction,U] = decode_deltaU(scenario,iter,mpc,du);
    U=squeeze(U(:,1,:));
    controllerOutput.optimizerTime = toc(optimizerTimer);
end