function z = ESO_NL(z, y, u, params, mode)
    % ESO parameters
    h = params.Ts;
    switch mode
        case 'roll'
            beta1 = params.beta1_roll;
            beta2 = params.beta2_roll;
            beta3 = params.beta3_roll;
            b = params.b_roll;
        case 'pitch'
            beta1 = params.beta1_pitch;
            beta2 = params.beta2_pitch;
            beta3 = params.beta3_pitch;
            b = params.b_pitch;
        case 'yaw'
            beta1 = params.beta1_yaw;
            beta2 = params.beta2_yaw;
            beta3 = params.beta3_yaw;
            b = params.b_yaw;
        case 'alt'
            beta1 = params.beta1_alt;
            beta2 = params.beta2_alt;
            beta3 = params.beta3_alt;
            b = params.b_alt;
    end
    alpha1 = params.alpha1;
    alpha2 = params.alpha2;
    delta = params.delta;

    % Observer dynamics
    e1 = z(1) - y;
    z1 = z(1) + h * (z(2) - beta1 * e1);
    z2 = z(2) + h * (z(3) - beta2 * fal(e1, alpha1, delta) + b * u);
    z3 = z(3) + h * (-beta3 * fal(e1, alpha2, delta));

    z = [z1; z2; z3];
end

function out = fal(e, alpha, delta)
    if abs(e) > delta
        out = abs(e)^alpha * sign(e);
    else
        out = e / delta^(1-alpha);
    end
end