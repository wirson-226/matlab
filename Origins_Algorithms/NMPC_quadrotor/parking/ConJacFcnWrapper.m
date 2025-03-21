function [cineq, ceq, Jcineq, Jceq] = ConJacFcnWrapper(z, data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance)
p = 70;
nx = 3;
pnx = p*nx;
nu = 2;
nmv = 2;
Ts = 0.1;
p1 = p+1;
uz = z(p*nx+1:end-1);
X = zeros(p1,nx);
U = zeros(p1,nu);
Xz = reshape(z(1:p*nx), nx, p)';
Uz = reshape(uz,nmv,p)';
X(1,:) = data.state;
X(2:p1,:) = Xz;
U(1:p1-1,:) = Uz;
e = z(end);
cineq = parkingIneqConFcn(X,U,e,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
[Jx, Ju, Je] = parkingIneqConFcnJacobian(X,U,e,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
Jx = permute(Jx, [3,2,1]);
Ju = permute(Ju, [3,2,1]);
Je = Je(:);
[nc,nx,p] = size(Jx);
Jcineq = [reshape(Jx,nc,p*nx)';(reshape(Ju,nc,p*nmv))';Je'];
ceq = zeros(pnx,1);
ic = 1:nx; 
Ui = U';
Xi = X';
h = Ts/2;
Jx = zeros(pnx,nx,p);
Jmv = zeros(pnx,nmv,p);
Je = zeros(pnx,1);
for i = 1:p
    uk  = Ui(:,i);
    xk  = Xi(:,i);
    xk1 = Xi(:,i+1);
    fk  = parkingVehicleStateFcn(xk,  uk, ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
    fk1 = parkingVehicleStateFcn(xk1, uk, ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
    ceq(ic) = xk + h*(fk + fk1) - xk1;
    [Ak,  Bk]  = parkingVehicleStateJacobianFcn(xk,  uk, ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
    [Ak1, Bk1] = parkingVehicleStateJacobianFcn(xk1, uk, ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
    if i > 1
        for k=1:nx
            Jx(ic,k,i-1) = h.*Ak(:,k);   % scale by h and assign
            Jx(ic(k),k,i-1) = Jx(ic(k),k,i-1) + 1;  % + Identity
        end
    end
    for k=1:nx
        Jx(ic,k,i) = h.*Ak1(:,k);   % scale by h and assign
        Jx(ic(k),k,i) = Jx(ic(k),k,i) - 1;  % - Identity
    end
    val = h.*(Bk + Bk1);
    for k=1:nmv
        Jmv(ic,k,i) = val(:,k);
    end
    ic = ic + nx;
end
[nc,nx,p] = size(Jx);
Jceq = [reshape(Jx,nc,p*nx)';(reshape(Jmv,nc,p*nmv))';Je'];
end