function f = CostFcnWrapper(z, data,yref)
p = 10;
nx = 4;
pnx = p*nx;
nu = 1;
nmv = 1;
Ts = 0.1;
p1 = p+1;
uz = z(p*nx+1:end-1);
Iz2u = [eye(5);[zeros(5,4),ones(5,1)]];
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
for i = 1:p
    % Calculate plant outputs
    xk = Xi(:,i+1);
    uk = Ui(:,i+1);
    yk = pendulumOutputFcn(xk, uk, []);
    yerr = yk - yref';
    Qy = [3;3];
    wtYerr = Qy.*yerr;  
    f = f + wtYerr'*wtYerr;
    umvk = Ui(1,i);
    if i == 1
        duk = umvk - data.lastMV;
    else
        umvk1 = Ui(1,i-1);
        duk = umvk - umvk1;
    end
    Qdu = 0.1;
    wtDu = Qdu.*duk;
    f = f + wtDu'*wtDu;
end
end