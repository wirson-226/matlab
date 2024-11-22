% necessary_condition_for_formation_achievement, for K2 values range

% 定义L1和L2矩阵
L1 = [1 -1 0 0 0;
      0 1 -1 0 0;
      0 0 1 -1 0;
      0 0 0 1 -1;
     -1 0 0 0 1];
 
L2 = [2 -1 0 -1 0;
      0 2 -1 0 -1;
     -1 0 2 -1 0;
      0 -1 0 2 -1;
     -1 0 -1 0 2];

% 计算特征值
eig_L1 = eig(L1);
eig_L2 = eig(L2);

% 参数设置
k11_values = [-2, -1]; % k11的不同取值
k12_values = [-1.2, 0]; % k12的不同取值

% 生成k21和k22的范围
k21_values = -5:0.1:5;
k22_values = -5:0.1:5;

% 初始化结果矩阵
result_5_6_L1 = zeros(length(k21_values), length(k22_values), length(k11_values));
result_5_7_L1 = zeros(length(k21_values), length(k22_values), length(k11_values));
result_5_6_L2 = zeros(length(k21_values), length(k22_values), length(k11_values));
result_5_7_L2 = zeros(length(k21_values), length(k22_values), length(k11_values));

% 遍历k11和k12的组合
for k_idx = 1:length(k11_values)
    k11 = k11_values(k_idx);
    k12 = k12_values(k_idx);

    % 遍历所有L1的特征值，计算不等式
    for idx = 1:length(eig_L1)
        lambda_i = eig_L1(idx); % 使用L1的特征值
        Re_lambda_i = real(lambda_i);
        Im_lambda_i = imag(lambda_i);
        
        for i = 1:length(k21_values)
            for j = 1:length(k22_values)
                k21 = k21_values(i);
                k22 = k22_values(j);
                
                Psi_i = k12 * k11 - Re_lambda_i * (k12 * k21 + k11 * k22) + (Re_lambda_i^2 + Im_lambda_i^2) * k21 * k22;
                
                % 计算不等式(5-6)
                inequality_5_6 = -k12 + Re_lambda_i * k22;
                if inequality_5_6 > 0
                    result_5_6_L1(i, j, k_idx) = result_5_6_L1(i, j, k_idx) + 1;
                end
                
                % 计算不等式(5-7)
                inequality_5_7 = (-k12 + Re_lambda_i * k22) * Psi_i - (Im_lambda_i^2) * k21^2;
                if inequality_5_7 > 0
                    result_5_7_L1(i, j, k_idx) = result_5_7_L1(i, j, k_idx) + 1;
                end
            end
        end
    end

    % 遍历所有L2的特征值，计算不等式
    for idx = 1:length(eig_L2)
        lambda_i = eig_L2(idx); % 使用L2的特征值
        Re_lambda_i = real(lambda_i);
        Im_lambda_i = imag(lambda_i);
        
        for i = 1:length(k21_values)
            for j = 1:length(k22_values)
                k21 = k21_values(i);
                k22 = k22_values(j);
                
                Psi_i = k12 * k11 - Re_lambda_i * (k12 * k21 + k11 * k22) + (Re_lambda_i^2 + Im_lambda_i^2) * k21 * k22;
                
                % 计算不等式(5-6)
                inequality_5_6 = -k12 + Re_lambda_i * k22;
                if inequality_5_6 > 0
                    result_5_6_L2(i, j, k_idx) = result_5_6_L2(i, j, k_idx) + 1;
                end
                
                % 计算不等式(5-7)
                inequality_5_7 = (-k12 + Re_lambda_i * k22) * Psi_i - (Im_lambda_i^2) * k21^2;
                if inequality_5_7 > 0
                    result_5_7_L2(i, j, k_idx) = result_5_7_L2(i, j, k_idx) + 1;
                end
            end
        end
    end
end

% 计算同时满足两个不等式的区域
feasible_region_L1_k11_2_k12_neg1_2 = (result_5_6_L1(:, :, 1) == length(eig_L1)) & (result_5_7_L1(:, :, 1) == length(eig_L1));
feasible_region_L2_k11_neg1_k12_0 = (result_5_6_L2(:, :, 2) == length(eig_L2)) & (result_5_7_L2(:, :, 2) == length(eig_L2));

% 可视化L1矩阵下的可行域（k11 = -2, k12 = -1.2）
figure;
imagesc(k21_values, k22_values, feasible_region_L1_k11_2_k12_neg1_2');
colorbar;
colormap('jet'); % 使用 'jet' 颜色图
% title('L1矩阵下K2=(k_{21}, k_{22})可行域 (k11 = -2, k12 = -1.2)');
title(sprintf('L1矩阵下K2=(k_{21}, k_{22})可行域 (k_{11} = %d, k_{12} = %.1f)', k11_values(1), k12_values(1)));
xlabel('k_{21}');
ylabel('k_{22}');
set(gca, 'YDir', 'normal');

% 可视化L2矩阵下的可行域（k11 = -1, k12 = 0）
figure;
imagesc(k21_values, k22_values, feasible_region_L2_k11_neg1_k12_0');
colorbar;
colormap('jet'); % 使用 'jet' 颜色图
% title('L2矩阵下K2=(k_{21}, k_{22})可行域 (k11 = -1, k12 = 0)');  变量赋值化
title(sprintf('L2矩阵下K2=(k_{21}, k_{22})可行域 (k_{11} = %d, k_{12} = %.1f)', k11_values(2), k12_values(2)));
xlabel('k_{21}');
ylabel('k_{22}');
set(gca, 'YDir', 'normal');
