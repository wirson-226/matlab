function f = CostFcnWrapper(z, data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance)
% 预测补偿
p = 70;
% 状态变量个数
nx = 3;
% 输入变量个数
nu = 2;
nmv = 2;
p1 = p+1;
% 从z中提取出来输入变量和状态变量
uz = z(p*nx+1:end-1);
X = zeros(p1,nx);
U = zeros(p1,nu);
Xz = reshape(z(1:p*nx), nx, p)';
Uz = reshape(uz,nmv,p)';
X(1,:) = data.state;
X(2:p1,:) = Xz;
U(1:p1-1,:) = Uz;
e = z(end);
f =  parkingCostFcn(X,U,e,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
end