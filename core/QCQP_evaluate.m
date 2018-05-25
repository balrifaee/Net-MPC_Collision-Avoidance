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

% Check QCQP constraints for a particular steering prediction
function [ ...
    feasible,...
    objValue, ...
    feasibilityScore, ...
    feasibilityScoreGradient, ...
    max_violation, ...
    sum_violations, ...
    constraintValuesVehicle,...
    constraintValuesObstacle ] = QCQP_evaluate( scenario, qcqp, deltaU )

    c_linear = 0;
    c_quad = 1e9;
    objectiveTradeoffCoefficient = 1;
    cfg = config;
    sum_violations = 0;
    max_violation = 0;

    feasible = true;
    Hp = scenario.Hp;
    nVeh = scenario.nVeh;
    nObst = size(scenario.obstacles,1);
    
    constraintValuesVehicle = -inf(nVeh,nVeh,Hp);
    constraintValuesObstacle = -inf(nVeh,nObst,Hp);
    
    objValue = deltaU'*qcqp.p0*deltaU + qcqp.q0'*deltaU + qcqp.r0;
    feasibilityScore = objectiveTradeoffCoefficient*objValue;
    feasibilityScoreGradient = objectiveTradeoffCoefficient*((qcqp.p0+qcqp.p0')*deltaU + qcqp.q0);

    for v=1:nVeh
        for k=1:Hp
            % VEHICLES
            for v2=(v+1):nVeh
                ci = deltaU'*qcqp.p{v,v2,k}*deltaU + qcqp.q{v,v2,k}'*deltaU + qcqp.r{v,v2,k};
                constraintValuesVehicle(v,v2,k) = ci;
                constraintValuesVehicle(v2,v,k) = ci;
                feasibilityScore = feasibilityScore + c_quad*max(ci,0)^2 + c_linear*max(ci,0);
                if (ci > 0)
                    feasibilityScoreGradient = feasibilityScoreGradient + ...
                        (c_quad*2*ci+c_linear)*((qcqp.p{v,v2,k}+qcqp.p{v,v2,k}')*deltaU+qcqp.q{v,v2,k});
                end
                if (ci > cfg.QCQP.constraintTolerance)
                    sum_violations = sum_violations + ci;
                    max_violation = max(max_violation,ci);
                end
                if (ci > cfg.QCQP.constraintTolerance)
                    feasible = false;
                end
            end
            % OBSTACLES
            for ob =1:nObst
                ci = deltaU'*qcqp.p_o{v,ob,k}*deltaU + qcqp.q_o{v,ob,k}'*deltaU + qcqp.r_o{v,ob,k};
                constraintValuesObstacle(v,ob,k) = ci;
                feasibilityScore = feasibilityScore + c_quad*max(ci,0)^2 + c_linear*max(ci,0);
                if (ci > 0)
                    feasibilityScoreGradient = feasibilityScoreGradient + ...
                        (c_quad*2*ci+c_linear)*((qcqp.p_o{v,ob,k}+qcqp.p_o{v,ob,k}')*deltaU+qcqp.q_o{v,ob,k});
                end
                if (ci > cfg.QCQP.constraintTolerance)
                    sum_violations = sum_violations + ci;
                    max_violation = max(max_violation,ci);
                end
                if (ci > cfg.QCQP.constraintTolerance)
                    feasible = false;
                end
            end
        end
    end
end