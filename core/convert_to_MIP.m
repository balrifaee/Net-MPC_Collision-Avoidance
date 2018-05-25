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

function mip = convert_to_MIP( scenario,iter )

mip = struct;
cfg = config;
idx = indices;

bigM = cfg.MIP_CPLEX.bigM;
polyDegree = cfg.MIP_CPLEX.polygonalNormApproximationDegree;

%% The problem parameters
state0   = iter.x0;
ctrl0    = iter.u0;
refPoint = iter.ReferenceTrajectoryPoints;
nx = scenario.model.nx;
nu = scenario.model.nu;
ny = scenario.model.ny;
dt = scenario.dt;
Hp = scenario.Hp;
Hu = scenario.Hu;
nVeh = scenario.nVeh;
RVeh = scenario.RVeh;
nObst = scenario.nObst;

W = [sin((1:polyDegree)*2*pi/polyDegree); cos((1:polyDegree)*2*pi/polyDegree)]';

%% Linear model
mpc = generate_mpc_matrices(scenario,iter);

%% Variable Index Mapping
statesStart = 0;
NOStates = Hp*nx*nVeh;
ctrlStart = NOStates;
NOCtrl = Hu*nu*nVeh;
deltaCtrlStart =  NOStates + NOCtrl;
NODeltaCtrl = Hu*nu*nVeh;
refDistStart =  NOStates + NOCtrl + NODeltaCtrl;
NORefDist = Hp*nVeh;
bObstAvoidStart = NOStates + NOCtrl + NODeltaCtrl + NORefDist;
bObstAvoid = Hp*ny*nObst*nVeh;
bVehAvoidStart = NOStates + NOCtrl + NODeltaCtrl + NORefDist + bObstAvoid;
bVehAvoid = 0;
if nVeh > 1
    bVehAvoid = Hp*ny*nVeh*nVeh;
end
NOV = NOStates + NOCtrl + NODeltaCtrl + NORefDist+ bObstAvoid + bVehAvoid;

% Index maps to calculate the position of each variable in the optimization
% problem.
varIdx = struct;
varIdx.x = @(v,k) statesStart + (nx*(v-1)*Hp) + nx*(k-1) + (1:nx);
varIdx.y = @(v,k) statesStart + (nx*(v-1)*Hp) + nx*(k-1) + (1:2);
varIdx.u = @(v,k) ctrlStart + Hu*(v-1) + k;
varIdx.refDist = @(v,k) refDistStart + ((v-1)*Hp) + k;
varIdx.deltaCtrl = @(v,k) deltaCtrlStart + ((v-1)*Hu) + k;
varIdx.bObstAvoid = @(v,o,k) bObstAvoidStart + 2*(Hp*nObst*(v-1) + Hp*(o-1) +k-1) + (1:2);
varIdx.bVehAvoid = @(vi,vj,k)bVehAvoidStart + 2*Hp*nVeh*(vi-1) + 2*Hp*(vj - 1) + 2*(k-1) + (1:2);


%% Objective
f_MILP = zeros(NOV,1);
f_MIQP = zeros(NOV,1);
H_MIQP = sparse(NOV,NOV);
r_MIQP = 0;
for v = 1:nVeh
    
    cooperationCoeff = 1;    
    if isfield(scenario,'CooperationCoefficients')
        assert(size(scenario.CooperationCoefficients,1) == 1)
        assert(size(scenario.CooperationCoefficients,2) == nVeh)
        cooperationCoeff = scenario.CooperationCoefficients(v);
    end
    
    for k = 1:Hp-1
        f_MILP(varIdx.refDist(v,k)) = cooperationCoeff * scenario.Q(v);
        f_MIQP(varIdx.y(v,k)) = -cooperationCoeff * 2*scenario.Q(v)*refPoint(k,:,v);     
        H_MIQP(varIdx.y(v,k),varIdx.y(v,k)) = cooperationCoeff * eye(2) * 2 * scenario.Q(v);
        r_MIQP = r_MIQP + cooperationCoeff * scenario.Q(v) * (refPoint(k,:,v)*refPoint(k,:,v)');
    end
    f_MILP(varIdx.refDist(v,Hp)) = cooperationCoeff * scenario.Q_final(v);
    f_MIQP(varIdx.y(v,Hp)) = -cooperationCoeff * 2*scenario.Q_final(v)*refPoint(Hp,:,v);
    H_MIQP(varIdx.y(v,Hp),varIdx.y(v,Hp)) = cooperationCoeff * eye(2) * 2 * scenario.Q_final(v);
    r_MIQP = r_MIQP + cooperationCoeff * scenario.Q_final(v) * (refPoint(Hp,:,v)*refPoint(Hp,:,v)');
    for k = 1:Hu
        f_MILP(varIdx.deltaCtrl(v,k)) = cooperationCoeff * cfg.MIP_CPLEX.R_Gain * scenario.R(v);
        H_MIQP(varIdx.deltaCtrl(v,k),varIdx.deltaCtrl(v,k)) = cooperationCoeff * 2 * scenario.R(v);
    end
end

%% Constraints

%% The equality constraints
Aeq = zeros (nx*Hp*nVeh,NOV);
Beq = zeros (nx*Hp*nVeh,1);
rows = 0; % Equation/Constraint counter

% Predictions of the system states
for v = 1:nVeh
    rows = rows(end) + (1:nx);
    Aeq(rows,  varIdx.x(v,1)) =  eye(nx);
    Aeq(rows,  varIdx.u(v,1)) = -mpc(v).B(:,:,1);
    Beq(rows,  1)             =  mpc(v).A(:,:,1)*state0(v,:)' + mpc(v).E(:,1);
    
    for k = 1:Hp-1
        rows = rows(end) + (1:nx);
        Aeq(rows, varIdx.x(v,k+1))         =  eye(nx);
        Aeq(rows, varIdx.x(v,k))           = -mpc(v).A(:,:,k+1);
        Aeq(rows, varIdx.u(v,min(Hu,k+1))) = -mpc(v).B(:,:,k+1);
        Beq(rows, 1)                       =  mpc(v).E(:,k+1);
    end
end
assert(rows(end) == nx*Hp*nVeh);

%% The inequality Constraints

%% Specifying the Number of the required Constraints
nConstraints = nVeh*(...
    Hp*polyDegree ... % Trajectory deviation slack variable
    + 2*Hu ... % Delta U slack variable
    + 4*nObst*Hp  ... % Obstacle avoidance
    + 2*(nVeh-1)*Hp); % Vehicle avoidance


%% Inequality Constraints Matrices
Aineq = zeros(nConstraints,NOV);
Bineq = zeros(nConstraints,1);
rows = 0; % Equation/Constraint counter

%% Distance of vehicles positions to Reference Trajectory Constraints
for v = 1:nVeh
    for k = 1:Hp
        rows = rows(end) + (1:polyDegree);
        Aineq(rows, varIdx.refDist(v,k)) = -1;
        Aineq(rows, varIdx.y(v,k))       = W;
        Bineq(rows, 1)                   = W*refPoint(k,:,v)';
    end
end

%% Defining the change of Control variables
for v = 1:nVeh
    rows = rows(end) + (1:2);
    Aineq(rows, varIdx.u(v,1))          = [ 1 -1]';
    Aineq(rows, varIdx.deltaCtrl(v,1))  = [-1 -1]';
    Bineq(rows, 1)                      = ctrl0(v,1) * [1;-1];
    
    for k = 2:Hu
        rows = rows(end) + (1:2);
        Aineq(rows, varIdx.u(v,k))         = [ 1 -1]';
        Aineq(rows, varIdx.deltaCtrl(v,k)) = [-1 -1]';
        Aineq(rows, varIdx.u(v,k-1))       = [-1  1]';
    end
end

mip.avoidanceContraintsStart = rows(end)+1;

%% Obstacle Avoidance Constraints
for v = 1:nVeh
    for o = 1:nObst
        % - Always the l and w here are expressing the half length and
        % half width
        if (cfg.MIP_CPLEX.obstAsQCQP) % consider obst. as in QCQP
            c = 1;
            s = 0;
            l = scenario.dsafeObstacles(v,o);
            w = scenario.dsafeObstacles(v,o);
        else % consider obst. as rectangle
            % - Augmenting the obstacle dimensions for enough safe obstacle
            % avoidance
            l = scenario.obstacles(o,idx.length)/2 + RVeh(v,1);  % Squaring to make sure that the l is positive as
            w = scenario.obstacles(o,idx.width)/2 + RVeh(v,1);  % it may be negative dependingon the orientation
            % - this term for considering the sampling time effect
            % consodering the velocity of the vehicles and the obstacle
            l_cord = (state0(v,4) + scenario.obstacles(o,4))*dt;
            % - Including the cord length in the obstaccles dimensions
            l = l + l_cord*cos(pi/4)/2;
            w = w + l_cord*cos(pi/4)/2;
            % - Checking if the calculated dimensions are enough or the obstacles was originally small
            if (l < l_cord/2)
                l = l_cord/2;
            end
            %         l = l + cfg.MILP_CPLEX.dsafeObstacle;
            if (w < l_cord/2)
                w = l_cord/2;
            end
            %         w = w + cfg.MILP_CPLEX.dsafeObstacle;
            c = cos(scenario.obstacles(o,3));
            s = sin(scenario.obstacles(o,3));
        end
        
        for k = 1:Hp
            obst_x = iter.obstacleFutureTrajectories(o,idx.x,k);
            obst_y = iter.obstacleFutureTrajectories(o,idx.y,k);            
            rows = rows(end) + (1:4);
            Aineq(rows, varIdx.y(v,k)) = [-c -s; c s; -s c; s -c];
            Aineq(rows, varIdx.bObstAvoid(v,o,k)) = bigM*[-1 -1; 1 -1;-1 1;1 1];
            Bineq(rows, 1) = ...
                - [l l w w]' ...
                + obst_x * [-c c -s s]' ...
                + obst_y * [-s s c -c]' ...
                + bigM * [0 1 1 2]';
        end
    end
end

%% Vehicle Avoidance Constraints
if (nVeh > 1)
    for vi = 1:nVeh
        for vj = 1:nVeh
            avoidanceDist = scenario.dsafeVehicles(vi,vj);
            for k = 1:Hp
                if vi<vj
                    rows = rows(end) + (1:4);
                    Aineq(rows, varIdx.y(vi,k)) = [1 0;0 1;-1 0;0 -1];
                    Aineq(rows, varIdx.y(vj,k)) = [-1 0;0 -1;1 0;0 1];
                    Aineq(rows, varIdx.bVehAvoid(vi,vj,k)) = bigM*[-1 -1; 1 -1;-1 1;1 1];
                    Bineq(rows, 1) = bigM*[0 1 1 2]' -avoidanceDist*[1 1 1 1]';
                end
            end
        end
    end
end

assert(rows(end) == nConstraints);

%% Lower and Upper Bounds
% Setting the Lower bounds and Upper Bounds of the control variables and the change of the control variables
lb = -inf(NOV,1);
ub =  inf(NOV,1);

for v = 1:nVeh
    for k = 1:Hu
        lb(varIdx.u(v,k)) = -iter.uMax(v);
        ub(varIdx.u(v,k)) = iter.uMax(v);
        ub(varIdx.deltaCtrl(v,k)) = scenario.duLim;
    end
end

%% Specifying the type of each variable in the X 'variables array'.
ctype(1:bObstAvoidStart) = 'C';
ctype(bObstAvoidStart+1 : NOV) = 'B';


mip.f_MILP = f_MILP;
mip.f_MIQP = f_MIQP;
mip.H_MIQP = H_MIQP;
mip.r_MIQP = r_MIQP;
mip.Aineq = Aineq;
mip.Bineq = Bineq;
mip.Aeq = Aeq;
mip.Beq = Beq;
mip.lb = lb;
mip.ub = ub;
mip.ctype = ctype;
mip.varIdx = varIdx;

end