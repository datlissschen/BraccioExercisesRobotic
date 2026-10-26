%% Main
% Initialize
qA = [20;45];
qB = [45; -30];
qC = [120; -60];

% Calculate TCP
tcpA = forward(qA);
tcpB = forward(qB);
tcpC = forward(qC);

% Plot position of manipulator
figure('Name', 'Manipulator Visualization');
subplot(1,3,1); plot_manipulator(qA, tcpA, 'Position A');
subplot(1,3,2); plot_manipulator(qB, tcpB, 'Position B');
subplot(1,3,3); plot_manipulator(qC, tcpC, 'Position C');

%% Functions
% Function for calculating forward kinematics
function x = forward(q)
    l1 = 3;
    l2 = 1;

    theta1 = q(1);
    theta2 = q(2);

    x_TCP = l1*cosd(theta1) + l2*cosd(theta1+theta2);
    y_TCP = l1*sind(theta1) + l2*sind(theta1+theta2);

    % Return output as a 2x1 column vector
    x = [x_TCP; y_TCP];
end

% Function for plotting the manipulator
function plot_manipulator(q, x_TCP_vec, title_str)
    l1 = 3;
    theta1 = q(1);
    
    % elbow coordinates for first link
    x_elbow = l1*cosd(theta1);
    y_elbow = l1*sind(theta1);
    
    % coordinate arrays [Base, Elbow, TCP]
    X = [0, x_elbow, x_TCP_vec(1)];
    Y = [0, y_elbow, x_TCP_vec(2)];
    
    plot(X, Y, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    
    axis equal;
    xlim([-4 5]);
    ylim([-1 5]);
    grid on;
    title(title_str);
    xlabel('x_0');
    ylabel('y_0');
end