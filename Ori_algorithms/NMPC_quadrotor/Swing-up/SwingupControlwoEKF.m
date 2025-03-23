nx = 4;
ny = 2;
nu = 1;
nlobj = nlmpc(nx, ny, nu);
Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;
nlobj.Model.StateFcn = "pendulumDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = 'pendulumOutputFcn';
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0; 0 0 1 0];
nlobj.Weights.OutputVariables = [3 3];
nlobj.Weights.ManipulatedVariablesRate = 0.1;
nlobj.Weights.ECR = eps;
nlobj.OV(1).Min = -10;
nlobj.OV(1).Max = 10;
nlobj.MV.Min = -100;
nlobj.MV.Max = 100;

%% Closed-Loop Simulation in MATLAB(R)
x = [0;0;-pi;0];
y = [x(1);x(3)];
EKF.State = x;
mv = 0;
yref1 = [0 0];
yref2 = [5 0];
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
% Run the simulation for |20| seconds.
Duration = 20;
xHistory = x;
for ct = 1:(20/Ts)
    % Set references
    if ct*Ts<10
        yref = yref1;
    else
        yref = yref2;
    end
    xk = x;
    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
    % Implement first optimal control move and update plant states.
    x = pendulumDT0(x,mv,Ts);
    % Save plant states for display.
    xHistory = [xHistory x]; %#ok<*AGROW>
end

%%
figure
dyn.l = 1;
for i = 1:size(xHistory,2)
    drawCartPole(i*Ts,xHistory(:,i),dyn);
    drawnow;
    pause(0.1);
end

figure
subplot(2,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time')
ylabel('z')
title('cart position')
subplot(2,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('zdot')
title('cart velocity')
subplot(2,2,3)
plot(0:Ts:Duration,xHistory(3,:))
xlabel('time')
ylabel('theta')
title('pendulum angle')
subplot(2,2,4)
plot(0:Ts:Duration,xHistory(4,:))
xlabel('time')
ylabel('thetadot')
title('pendulum velocity')
