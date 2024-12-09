function z = ESO(z, y, u, params)
% 使用控制器时的观测器项
    % ESO parameters
    b0 = params.b0;
    alpha1 = params.alpha1;
    alpha2 = params.alpha2;
    alpha3 = params.alpha3;
    beta1 = params.beta1;
    beta2 = params.beta2;
    beta3 = params.beta3;

    % Observer dynamics
    z1 = z(1) + params.Ts * (z(2) - beta1 * (z(1) - y));
    z2 = z(2) + params.Ts * (z(3) + b0 * u - beta2 * (z(1) - y));
    z3 = z(3) + params.Ts * (-beta3 * (z(1) - y));

    z = [z1; z2; z3];
end
