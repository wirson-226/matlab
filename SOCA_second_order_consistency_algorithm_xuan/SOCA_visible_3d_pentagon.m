% Parameters
r = 6; % radius of the circular trajectory
% omega = 0.25; % angular velocity
omega = 0; % angular velocity
n = 5; % number of agents

% Time settings
T = 0:0.1:100; % simulation time

% Laplacian matrix for the ring topology
L = [ 1 -1  0  0  0;
      0  1 -1  0  0;
      0  0  1 -1  0;
      0  0  0  1 -1;
     -1  0  0  0  1];

% Controller parameters
K1 = [-2 -1.2];
K2 = [0.17 0.37];

% Initial states
xi = [5.59, 0.16, 0.11, -0.08;
      1.57, -0.10, 5.85, 0.00;
     -4.79, -0.28, 3.57, 0.04;
     -5.24, -0.04, -3.65, 0.04;
      1.79, 0.09, -5.84, 0.07]';

% State history
X_history = zeros(4*n, length(T));
X_history(:, 1) = xi(:);

% Dynamics simulation
for k = 2:length(T)
    % Reshape the state history to get current states
    x = reshape(X_history(:, k-1), 4, n);
    pos = x(1:2, :); % Position components
    vel = x(3:4, :); % Velocity components
    
    % Compute the control input for each agent
    u = zeros(2, n); % Initialize control input
    for i = 1:n
        % Position and velocity error terms
        pos_error = pos - pos(:, i);
        vel_error = vel - vel(:, i);
        
        % Control law
        u(:, i) = -sum(K1 * pos_error, 2) - sum(K2 * vel_error, 2);
    end
    
    % Update states
    dx = [vel; u];
    X_history(:, k) = X_history(:, k-1) + dx(:) * 0.1;
end

% Real-time trajectory observation
figure;
hold on;
colors = {'r', 'g', 'b', 'c', 'm'};
lines = gobjects(n, 1);

for i = 1:n
    lines(i) = plot(NaN, NaN, 'o-', 'Color', colors{i}, 'DisplayName', ['Agent ' num2str(i)]);
end

xlim([-10, 10]);
ylim([-10, 10]);
xlabel('x_i(t) (m)');
ylabel('y_i(t) (m)');
title('Real-time Position Trajectories');
legend;

for k = 1:length(T)
    for i = 1:n
        set(lines(i), 'XData', X_history(1+4*(i-1), 1:k), 'YData', X_history(2+4*(i-1), 1:k));
    end
    drawnow;
end

hold off;
