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

% Compute and convert the MPC matrices to QCQP form
function qcqp = convert_to_QCQP(scenario,iter,mpc)

%% RESULT FORM:
%   x = [ du1; du2;...;dunVeh];
%   minimize    x'*p0*x + q0'*x + r0
%   subject to  x'*pi*x + qi'*x + ri <= 0,   i = 1,...,m

dsafeExtra = 0;

%% PARAMETERS
nu = scenario.model.nu;
ny = scenario.model.ny;
Hp = scenario.Hp;
Hu = scenario.Hu;
nVeh = scenario.nVeh;
nObst = scenario.nObst;

%% RESULT MATRICES
p0 = zeros(nVeh*(nu*Hu));
q0 = zeros(nVeh*(nu*Hu),1);
r0 = 0;

p = cell(nVeh-1, nVeh-1, Hp);
q = cell(nVeh-1, nVeh-1, Hp);
r = cell(nVeh-1, nVeh-1, Hu);

p_obst = cell(nVeh, nObst, Hp);
q_obst = cell(nVeh, nObst, Hp);
r_obst = cell(nVeh, nObst, Hp);

%%  GENARATE MATRICES
for v=1:nVeh
    % OBJECTIVE FUNCTION MATRICES
    veh1_slice = blk(v,nu*Hu);
    
    cooperationCoeff = 1;    
    if isfield(scenario,'CooperationCoefficients')
        assert(size(scenario.CooperationCoefficients,1) == 1)
        assert(size(scenario.CooperationCoefficients,2) == nVeh)
        cooperationCoeff = scenario.CooperationCoefficients(v);
    end    
    p0(veh1_slice,veh1_slice) = cooperationCoeff*mpc(v).H;
    q0(veh1_slice,1) = cooperationCoeff*mpc(v).g;
    r0 = r0 + cooperationCoeff*mpc(v).r;
    % CONSTRAINTS MATRICES
    for k=1:Hp
        % VEHICLES AVOIDANCE
        intv = (k-1)*ny+1 : k*ny;
        for v2=(v+1):nVeh
            p{v,v2,k} = zeros( nVeh*(nu*Hu) );
            q{v,v2,k} = zeros( nVeh*(nu*Hu),1 );
            
            veh2_slice = blk(v2,nu*Hu);
                        
            p{v,v2,k}(veh1_slice,veh1_slice)  = - mpc( v).Theta(intv,:)'*mpc( v).Theta(intv,:);
            p{v,v2,k}(veh2_slice,veh2_slice)  = - mpc(v2).Theta(intv,:)'*mpc(v2).Theta(intv,:);
            p{v,v2,k}(veh1_slice,veh2_slice)  =   mpc( v).Theta(intv,:)'*mpc(v2).Theta(intv,:);
            p{v,v2,k}(veh2_slice,veh1_slice)  =   mpc(v2).Theta(intv,:)'*mpc( v).Theta(intv,:);
            
            b =  mpc(v).freeResponse(intv,:) - ...
                 mpc(v2).freeResponse(intv,:);
             
            q{v,v2,k}(veh1_slice  ,1) = -2*mpc(v).Theta(intv,:)'*b;
            q{v,v2,k}(veh2_slice ,1) =  2*mpc(v2).Theta(intv,:)'*b;
            r{v,v2,k} = (scenario.dsafeVehicles(v,v2) + dsafeExtra)^2 - b'*b;
        end
        % OBSTACLE AVOIDANCE
        % Obstacle in next time step
        for o =1:nObst
            p_obst{v,o,k} = zeros( nVeh*(nu*Hu) );
            q_obst{v,o,k} = zeros( nVeh*(nu*Hu),1 );
            p_obst{v,o,k}(veh1_slice,veh1_slice) = - mpc(v).Theta(intv,:)'*mpc(v).Theta(intv,:);
            b = mpc(v).freeResponse(intv,:) - iter.obstacleFutureTrajectories(o,:,k)';
            q_obst{v,o,k}(veh1_slice,1) = -2*mpc(v).Theta(intv,:)'*b;
            r_obst{v,o,k} = (scenario.dsafeObstacles(v,o) + dsafeExtra)^2 - b'*b;
        end
    end
end

for v=1:nVeh
    for k=1:Hp
        for v2=(v+1):nVeh
            p{v,v2,k} = symmetric(p{v,v2,k});
        end
        for o =1:nObst
            p_obst{v,o,k} = symmetric(p_obst{v,o,k});
        end
    end
end

%% QCQP structure
qcqp.p0 = p0;
qcqp.q0 = q0;
qcqp.r0 = r0;

qcqp.p = p;
qcqp.q = q;
qcqp.r = r;

qcqp.p_o = p_obst;
qcqp.q_o = q_obst;
qcqp.r_o = r_obst;
