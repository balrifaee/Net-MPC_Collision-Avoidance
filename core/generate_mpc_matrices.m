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

function mpc = generate_mpc_matrices( scenario,iter )

    % Compute all relevant discretization and MPC matrices for all
    % vehicles.
        
    nx = scenario.model.nx;
    nu = scenario.model.nu;
    ny = scenario.model.ny;
    nVeh = scenario.nVeh;
    Hp = scenario.Hp;
    Hu = scenario.Hu;
    dt = scenario.dt;

    mpc=struct.empty(nVeh,0);

    %% GENARATE MATRICES
    for v=1:nVeh
        for i=1:Hp
            mpc(v).Reference( blk(i,ny),1 ) = iter.ReferenceTrajectoryPoints( i,1:ny,v )';
        end

        mpc(v).A = zeros(nx,nx,Hp);
        mpc(v).B = zeros(nx,nu,Hp);
        mpc(v).E = zeros(nx,Hp);

        [ A,B,C,E ] ...
            = discretize( iter.x0(v,:), iter.u0(v,:), scenario.Lf(v), scenario.Lr(v), dt, scenario.model );
        [ Psi, Gamma, mpc(v).Theta, Pie ] ...
            = prediction_matrices( A,B,C,nx,nu,ny,Hp,Hu );  

        mpc(v).freeResponse = Psi *iter.x0(v,:)' + Gamma *iter.u0(v,:)' + Pie*E;

        [ mpc(v).H , mpc(v).g, mpc(v).r ] ...
            = mpc_cost_function_matrices( scenario.Q(v), scenario.R(v), nu,ny,Hp,Hu,mpc(v),scenario.Q_final(v) );

        % For the simple linearization, the same A,B,E are used in
        % every prediction step.
        for i=1:Hp
            mpc(v).A(:,:,i) = A;
            mpc(v).B(:,:,i) = B;
            mpc(v).E(:,i) = E;
        end
    end
end