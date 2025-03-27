function TF = extract_TF_from_params(params)
% 从给定的气动参数结构 params 和巡航速度 Va 提取线性化控制器用的 TF 系数
% 输出 TF 包含滚转、俯仰、空速控制所需的一阶/二阶系统系数

% 基础量
rho = params.rho;
S = params.S_wing;
b = params.b;
c = params.c;
Jx = params.Jx;
Jy = params.Jy;
m  = params.mass;
Va = 1.2* params.V_min;

% ----------- Roll Dynamics -----------
TF.a_phi1 = -0.5 * rho * Va^2 * S * b * params.C_ell_p * b / (2 * Va) / Jx;
TF.a_phi2 =  0.5 * rho * Va^2 * S * b * params.C_ell_delta_a / Jx;

% ----------- Pitch Dynamics -----------
TF.a_theta1 = -0.5 * rho * Va * c * S * params.C_m_q * c / (2 * Va) / Jy;
TF.a_theta2 = -0.5 * rho * Va^2 * c * S * params.C_m_alpha / Jy;
TF.a_theta3 =  0.5 * rho * Va^2 * c * S * params.C_m_delta_e / Jy;

% ----------- Airspeed Dynamics -----------
alpha_trim = 0;  % 可设为 trim 解的迎角，简化取 0
CD_trim = params.C_D_0 + params.C_D_alpha * alpha_trim;
TF.a_V1 = rho * Va * S * CD_trim / m;

% 螺旋桨推力影响（近似估计）
% 可根据电机推力测试修正
TF.a_V2 = params.S_prop / m * params.k_motor;

end
