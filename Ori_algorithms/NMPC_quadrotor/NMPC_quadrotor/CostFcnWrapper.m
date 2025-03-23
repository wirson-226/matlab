function f = CostFcnWrapper(z, data,yref)
p = 18;
nx = 12;
pnx = p*nx;
nu = 4;
nmv = 4;
Ts = 0.1;
p1 = p+1;
uz = z(p*nx+1:end-1);
Iz2u = [eye(2*4);[zeros(64,4),repmat(eye(4),16,1)]];
X = zeros(p1,nx);
U = zeros(p1,nu);
Xz = reshape(z(1:p*nx), nx, p)';
Uz = reshape(Iz2u*uz,nmv,p)';
X(1,:) = data.state;
X(2:p1,:) = Xz;
U(1:p1-1,:) = Uz;
e = z(end);
Xi = X';
Ui = U';
f = 0;
OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
ManipulatedVariables = [0.1 0.1 0.1 0.1];
ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
for i = 1:p
    % Calculate plant outputs
    xk = Xi(:,i+1);
    uk = Ui(:,i+1);
    yk = xk;
    yerr = yk - yref(:,i);
    Qy = OutputVariables';
    wtYerr = Qy.*yerr;  
    f = f + wtYerr'*wtYerr;
    
    umvk = Ui(:,i);
    if i == 1
        duk = umvk - data.lastMV;
    else
        umvk1 = Ui(:,i-1);
        duk = umvk - umvk1;
    end
    Qu = ManipulatedVariables';
    wtu = Qu.*uk;
    f = f +wtu'*wtu;

%     size(wtu'*wtu)
    Qdu = ManipulatedVariablesRate';
    wtDu = Qdu.*duk;
    f = f + wtDu'*wtDu;
end
end