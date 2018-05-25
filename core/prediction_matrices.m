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

function [ Psi, Gamma, Theta, Pie ] = prediction_matrices( A,B,C,nx,nu,ny,Hp, Hu )

    assert(Hu <= Hp);
    Theta = zeros( ny*Hp,nu*Hu );
    Psi = zeros( ny*Hp,nx );
    Gamma = zeros( ny*Hp,nu );
    Pie = zeros(ny*Hp,nx);

    powersCA = zeros(ny,nx,Hp+1);
    powersCA(:,:,1) = C*eye(nx,nx);
    summedPowersCA = zeros(ny,nx,Hp+1);
    summedPowersCA(:,:,1) = C*eye(nx,nx);
    for i=2:Hp+1
        powersCA(:,:,i) = C * A^(i-1);
        summedPowersCA(:,:,i) = powersCA(:,:,i) + summedPowersCA(:,:,i-1);
    end
    
    for i=1:Hp
        Psi( blk(i,ny),:) = powersCA(:,:,i+1);
        Gamma ( blk(i,ny),:) = summedPowersCA(:,:,i)*B;
        for iu=1:min(i,Hu)
            Theta( blk(i,ny), blk(iu,nu)) =  summedPowersCA(:,:,i-iu+1)*B;
        end        
        Pie(blk(i,ny),:) = summedPowersCA(:,:,i);
    end

end

