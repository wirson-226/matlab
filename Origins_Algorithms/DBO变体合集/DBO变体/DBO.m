function [fMin, bestX, Convergence_curve] = DBO(N, maxEvaluations, c, d, dim, fobj)
    
    P_percent = 0.2;  % 生產者比例
    pNum = round(N * P_percent);  % 生產者数量（总种群的20%）
    % Max_iter=maxEvaluations/N;
    lb = c .* ones(1, dim);  % 下界
    ub = d .* ones(1, dim);  % 上界
    Convergence_curve=[];
    % 初始化
    x = zeros(N, dim);
    fit = zeros(N, 1);
    
    for i = 1:N
        x(i, :) = lb + (ub - lb) .* rand(1, dim);
        fit(i) = fobj(x(i, :));
    end
    
    pFit = fit;
    pX = x;
    XX = pX;
    [fMin, bestI] = min(fit);  % 全局最佳适应度
    bestX = x(bestI, :);  % 全局最佳位置

    evalCount = 0;  % 评价次数初始化为N

    % 开始更新解
    while evalCount < maxEvaluations
        
        [fmax, B] = max(fit);
        worse = x(B, :);
        r2 = rand(1);

        % 生产者的更新
        for i = 1:pNum
            if (r2 < 0.9)
                
                    r1=rand(1);
                    a=rand(1,1);
                    if (a>0.1)
                        a=1;
                    else
                        a=-1;
                    end

                x(i, :) = pX(i, :) + 0.3 * abs(pX(i, :) - worse) + a * 0.1 * (XX(i, :));  % 方程 (1)
            else
                aaa = randperm(180, 1);
                if (aaa == 0 || aaa == 90 || aaa == 180)
                    x(i, :) = pX(i, :);
                end
                theta = aaa * pi / 180;   
                x(i, :) = pX(i, :) + tan(theta) .* abs(pX(i, :) - XX(i, :));  % 方程 (2)
            end
            
            x(i, :) = Bounds(x(i, :), lb, ub);    
            fit(i) = fobj(x(i, :));
            evalCount = evalCount + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(evalCount) = fMin;
            if evalCount>=maxEvaluations
                return;
            end
        end 
        
        [fMMin, bestII] = min(fit);  % 更新当前最佳适应度
        bestXX = x(bestII, :);  % 更新当前最佳位置 

        R = 1 - evalCount / maxEvaluations;  % 计算适应性降低比率
       
        % 变异更新
        Xnew1 = bestXX .* (1 - R);
        Xnew2 = bestXX .* (1 + R);  % 方程 (3)
        Xnew1 = Bounds(Xnew1, lb, ub);
        Xnew2 = Bounds(Xnew2, lb, ub);

        Xnew11 = bestX .* (1 - R); 
        Xnew22 = bestX .* (1 + R);  % 方程 (5)
        Xnew11 = Bounds(Xnew11, lb, ub);
        Xnew22 = Bounds(Xnew22, lb, ub);

        % 其他个体的更新
        for i = (pNum + 1):12  % 方程 (4)
            x(i, :) = bestXX + (rand(1, dim) .* (pX(i, :) - Xnew1) + rand(1, dim) .* (pX(i, :) - Xnew2));
            x(i, :) = Bounds(x(i, :), Xnew1, Xnew2);
            fit(i) = fobj(x(i, :));
            evalCount = evalCount + 1;  % 增加评价计数
            Convergence_curve(evalCount) = fMin;
            if evalCount>=maxEvaluations
                return;
            end

        end
        
        for i = 13:19  % 方程 (6)
            % if  rand>0.5
            x(i, :) = pX(i, :) + (randn(1) .* (pX(i, :) - Xnew11) + (rand(1, dim) .* (pX(i, :) - Xnew22)));
            x(i, :) = Bounds(x(i, :), lb, ub);
            fit(i) = fobj(x(i, :));
            evalCount = evalCount + 1;  % 增加评价计数
            Convergence_curve(evalCount) = fMin;
            if evalCount>=maxEvaluations
                return;
            end
            % else
            % r = norm(pX(i, :) - bestX);
            % a = 1; % 悬链面的参数
            % z = a * cosh(r / a); % 悬链面方程部分
            % step_size = 0.1; % 步长范围
            % % 生成新解
            % perturbation = rand(1, dim) * step_size; % 随机扰动
            % x(i, :) = bestX + (perturbation / (1 + z)); % 生成新位置
            % x(i, :) = Bounds(x(i, :), lb, ub);
            % fit(i) = fobj(x(i, :));
            % evalCount = evalCount + 1;  % 增加评价计数
            % Convergence_curve(evalCount) = fMin;
            % if evalCount>=maxEvaluations
            %     return;
            % end

            % end

        end
        
        for j = 20:N  % 方程 (7)
            x(j, :) = bestX + randn(1, dim) .* ((abs((pX(j, :) - bestXX))) + (abs((pX(j, :) - bestX)))) / 2;
            x(j, :) = Bounds(x(j, :), lb, ub);
            fit(j) = fobj(x(j, :));
            evalCount = evalCount + 1;  % 增加评价计数
            Convergence_curve(evalCount) = fMin;
            if evalCount>=maxEvaluations
                return;
            end
        end

        % 更新个体的最好适应度
        XX = pX;
        for i = 1:N 
            if (fit(i) < pFit(i))
                pFit(i) = fit(i);
                pX(i, :) = x(i, :);
            end
            
            if (pFit(i) < fMin)
                fMin = pFit(i);
                bestX = pX(i, :);
            end
        end

        Convergence_curve(evalCount) = fMin;  % 记录收敛值
       
        
    end

    
end

% 边界处理函数
    function s = Bounds(s, Lb, Ub)
        temp = s;
        I = temp < Lb;
        temp(I) = Lb(I);
        J = temp > Ub;
        temp(J) = Ub(J);
        s = temp;
    end
