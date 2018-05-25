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

function [ x_0 , feasible , objValue, optimization_log] = SCP_optimizer( scenario,iter,qcqp, mpc, x_0 )

if abs(x_0(1)) < eps % avoid numerical issues
    x_0(1) = eps;
end

nVeh = scenario.nVeh;
Hp = scenario.Hp;
Hu = scenario.Hu;
nObst = scenario.nObst;
nu = scenario.model.nu;

n_du = length(x_0);
nVars = length(x_0)+1;
nCons = nVeh * ( Hp*(nVeh-1)/2 + Hp*nObst + 2*Hu );

[ ~, objValue_0, ~, ~, max_violation_0, ~ ] = QCQP_evaluate( scenario, qcqp, x_0);
cfg = config;
delta_tol = 1e-4;
slack_weight = 1e5;
slack_ub = 1e30;
slack_lb = 0;
max_SCP_iter = 20;

optimization_log = struct.empty;

for i = 1:max_SCP_iter
    Aineq = zeros(nCons,nVars);
    bineq = zeros(nCons,1);
    row = 0;
    % VEHICLE AVOIDANCE
    for v=1:nVeh-1
        for v2=(v+1):nVeh
            for k=1:Hp
                row = row(end) + 1;
                Aineq(row,1:n_du) = (qcqp.q{v,v2,k}'+2*x_0'*qcqp.p{v,v2,k});
                bineq(row) = -(qcqp.r{v,v2,k}-x_0'*qcqp.p{v,v2,k}*x_0);
            end
        end
    end
    % OBSTACLE AVOIDANCE
    for v=1:nVeh
        for o=1:nObst
            for k=1:Hp
                row = row(end) + 1;
                Aineq(row,1:n_du) = (qcqp.q_o{v,o,k}'+2*x_0'*qcqp.p_o{v,o,k});
                bineq(row) = -(qcqp.r_o{v,o,k}-x_0'*qcqp.p_o{v,o,k}*x_0);
            end
        end
    end
    
    % U limits
    for v = 1:nVeh
        row = row(end) + (1:Hu);
        Aineq(row,(v-1)*(nu*Hu)+1:v*(nu*Hu)) = tril(ones(Hu,Hu));
        bineq(row) = -iter.u0(v) + iter.uMax(v);
        
        row = row(end) + (1:Hu);
        Aineq(row,(v-1)*(nu*Hu)+1:v*(nu*Hu)) =  -(tril(ones(Hu,Hu)));
        bineq(row) = -(-iter.u0(v) - iter.uMax(v));
    end
    
    lb = -ones(n_du,1)*scenario.duLim;
    ub = ones(n_du,1)*scenario.duLim;
    P = 2*qcqp.p0;
    
    % slack var
    q = [qcqp.q0; slack_weight];
    P(end+1,end+1) = 0;
    Aineq(1:end-2*Hu*nVeh,end) = -1; % <--  enable slack var just for collision avoidance constraints
    lb(end+1) = slack_lb;
    ub(end+1) = slack_ub;
    
    [x, fval, exitflag, output] = cplexqp(P, q, Aineq, bineq, [], [], lb, ub);
    
    if exitflag == 1 || exitflag == 5 || exitflag == 6
        slack = x(end);
        prev_x = x_0;
        x_0 = x(1:end-1);
        % Eval. progress
        [ feasible, objValue, ~, ~, max_violation, sum_violations, constraintValuesVehicle, constraintValuesObstacle  ] = QCQP_evaluate( scenario, qcqp, x_0);
        
        max_constraint = max( max(max(max(constraintValuesVehicle))), max(max(max(constraintValuesObstacle))) );
        
        fval = fval + qcqp.r0;
        delta_hat = (objValue_0 + slack_weight*max_violation_0) - fval; % predicted decrease of obj
        delta = (objValue_0 + slack_weight*max_violation_0) - (objValue + slack_weight*max_violation); % real decrease of obj
        fprintf('slack %8f max_violation %8f sum_violations %8f feas %d objVal %8f fval %8f max_constraint %e\n',slack, max_violation, sum_violations, feasible, objValue, fval, max_constraint);
        
        objValue_0 = objValue;
        max_violation_0 = max_violation;
                
        % Log context
        optimization_log(i).P = P;
        optimization_log(i).q = q;
        optimization_log(i).Aineq = Aineq;
        optimization_log(i).bineq = bineq;
        optimization_log(i).lb = lb;
        optimization_log(i).ub = ub;
        optimization_log(i).x = x;
        optimization_log(i).slack = slack;
        optimization_log(i).SCP_ObjVal = fval;
        optimization_log(i).QCQP_ObjVal = objValue;
        optimization_log(i).delta_hat = delta_hat;
        optimization_log(i).delta = delta;
        optimization_log(i).du = x_0;
        optimization_log(i).feasible = feasible;
        optimization_log(i).prev_du = prev_x;
        [optimization_log(i).Traj,optimization_log(i).U] = decode_deltaU(scenario,iter,mpc,x_0);
        [optimization_log(i).prevTraj,optimization_log(i).prevU] = decode_deltaU(scenario,iter,mpc,prev_x);
        
        
        if nVeh == 1
            if  abs(delta) < delta_tol && max_violation > cfg.QCQP.constraintTolerance
                break
            end
        end
        if  abs(delta) < delta_tol && max_violation <= cfg.QCQP.constraintTolerance % max_violation is constraintTolerance in QCQP_evaluate.m.
            break
        end
    else
        disp(['exitflag: ' num2str(exitflag) ', ' output.cplexstatusstring ', ' output.message]);
        feasible = false;
        return;
    end
end
disp(['iterations: ' num2str(i)])
end

