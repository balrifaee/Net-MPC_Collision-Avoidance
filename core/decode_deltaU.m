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

function [Traj,U] = decode_deltaU(scenario,iter,mpc,du)

Hp = scenario.Hp;
Hu = scenario.Hu;
nVeh = scenario.nVeh;
nu = scenario.model.nu;
ny = scenario.model.ny;
U = zeros(Hp,nu,nVeh);
Traj = zeros(Hp,ny,nVeh);
du = reshape(du,[nu,Hu,nVeh]);

% Control values
for v=1:nVeh
    U(1,:,v) = du(:,1,v)' + iter.u0(v,:);
    for k=2:Hu
        U(k,:,v) = du(:,k,v)' + U(k-1,:,v);
    end
    for k=Hu+1:Hp
        U(k,:,v) = U(Hu,:,v);
    end
end

% Predicted trajectory
for v=1:nVeh
    X = mpc(v).freeResponse + mpc(v).Theta*du(:,:,v)';
    X=reshape(X,[scenario.model.ny,Hp]);
    for i=1:ny
        Traj(:,i,v) = X(i,:);
    end 
end
