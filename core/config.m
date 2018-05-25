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

% Central configuration file. Holds solver, controller, etc. settings. Anything that's not scenario-specific.
function [ cfg ] = config

    cfg = struct;
    
    cfg.MIP_CPLEX = struct;
    cfg.MIP_CPLEX.bigM = 1000;
    cfg.MIP_CPLEX.R_Gain = 0.1;
    cfg.MIP_CPLEX.polygonalNormApproximationDegree = 6;
    cfg.MIP_CPLEX.timelimit = 300; % in seconds
    cfg.MIP_CPLEX.obstAsQCQP = 1;
    
    cfg.Lagrange_Mosek = struct;
    cfg.Lagrange_Mosek.R_Gain = 10;
        
    cfg.QCQP.default_dsafeExtra = 0;

    % conversion between distance tolerance and 
    % constraint tolerance: cons_tol = 2 * d_safe * d_tol
    cfg.QCQP.constraintTolerance = 2 * 2.1 * 1e-3; % ~1mm is sufficient
end

