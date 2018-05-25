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

function [U,trajectoryPrediction,controllerOutput] = MILP_CPLEX_controller( scenario,iter, ~ )

    controllerOutput = struct;
    mip = convert_to_MIP( scenario,iter );
    
    %% Solving the Optimization Problem
    global CPLEXoptions;
    if iter.reset
        warning off MATLAB:lang:badlyScopedReturnValue
        CPLEXoptions = cplexoptimset('output.clonelog',-1);
        CPLEXoptions.read.scale = 1;
    end
    
    optimizerTimer = tic;
    [x,fval,exitflag,output] = cplexmilp (mip.f_MILP, mip.Aineq, mip.Bineq, mip.Aeq, mip.Beq,[ ], [ ], [ ], mip.lb, mip.ub, mip.ctype, [ ], CPLEXoptions);
    controllerOutput.optimizerTime = toc(optimizerTimer);
    controllerOutput.CPLEX_output = output;
    controllerOutput.CPLEX_exitflag = exitflag;
    controllerOutput.CPLEX_options = CPLEXoptions;
    controllerOutput.CPLEX_fval = fval;
    controllerOutput.resultInvalid = exitflag < 1 || exitflag == 6;

    %% Extracting the data that will be returned from the optimized variables array
    ny = scenario.model.ny;
    Hp = scenario.Hp;
    Hu = scenario.Hu;
    nVeh = scenario.nVeh;
    
    min_avoid_constr = [];
    if ~isempty(x)
        min_avoid_constr = min(mip.Bineq(mip.avoidanceContraintsStart:end)-mip.Aineq(mip.avoidanceContraintsStart:end,:)*x);
    end

    fprintf('cplexmilp status: flag %i | %s | %s | fval %8f | min_avoid_constr %8e\n', exitflag, output.cplexstatusstring, output.message, fval, min_avoid_constr);
    
    % when exitflag > 0, then x is a solution
    if exitflag > 0
        % extract results
        U = zeros(Hp,nVeh);
        trajectoryPrediction = zeros(Hp,ny,nVeh);
        for v = 1:nVeh
            for k = 1:Hp
                trajectoryPrediction(k,:,v) = x(mip.varIdx.y(v,k));
                U(k,v) = x(mip.varIdx.u(v,min(k,Hu)));
            end
        end        
    else
        [trajectoryPrediction,U] = decode_deltaU(scenario,iter,generate_mpc_matrices( scenario,iter ),zeros(scenario.Hu*scenario.nVeh,1));
        U=squeeze(U(:,1,:));
    end

end
