%% Exercise 1.1a
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

%% Exercise 1.1b
b_tcpX = [];
b_tcpY = [];
idx = 1;

for t1 = 0:110 
    for t2 = -120:120
        q = [t1, t2];
        tcp_current = forward(q);
        %store results
        b_tcpX(idx) = tcp_current(1);
        b_tcpY(idx) = tcp_current(2);
        
        idx = idx+1;
    end
end

% Plot permissible working range
figure('Name', 'Permissible Working Range')
plot(b_tcpX, b_tcpY, '.', 'Color', 'black') 
axis equal; % prevent workspace look distorted
title('Workspace of the 2-DOF Manipulator');
xlabel('x_0');
ylabel('y_0');

%% Exercise 1.1c
q_test = [20; 45];
l1 = 3;
l2 = 1;

A_0_1 = dh_matrix(q_test(1), 0, l1, 0); 
A_1_2 = dh_matrix(q_test(2), 0, l2, 0); 

% transformation (T_0_2) from KS0 to KS2
T_0_2 = A_0_1 * A_1_2;

% TCP-Position in KS0
tcp_dh = T_0_2(1:2, 4);

% Compare with Part A
tcp_geometric = forward(q_test);

disp('--- Compare results ---');
fprintf('Part A (Geometrisch): X = %.4f, Y = %.4f\n', tcp_geometric(1), tcp_geometric(2));
fprintf('Part C (DH-Matrizen): X = %.4f, Y = %.4f\n', tcp_dh(1), tcp_dh(2));

%% Exercise 1.1 d
% test values
xD = [3.5; 1.5];
xE = [2.0; 2.0];
xF = [1.0; 1.0];

% calculate
qD = inverse_kinematics(xD);
qE = inverse_kinematics(xE);
qF = inverse_kinematics(xF);

% plot
figure('Name', 'Inverse Kinematik - found solutions');

% Subplots
subplot(1,3,1); 
if ~isnan(qD(1))
    plot_manipulator(qD, xD, 'solution for x_D');
else
    title('x_D not permissable'); axis equal; xlim([-4 5]); ylim([-1 5]);
end

subplot(1,3,2); 
if ~isnan(qE(1))
    plot_manipulator(qE, xE, 'solution for x_E');
else
    title('x_E not permissable'); axis equal; xlim([-4 5]); ylim([-1 5]);
end

subplot(1,3,3); 
if ~isnan(qF(1))
    plot_manipulator(qF, xF, 'solution for x_F');
else
    title('x_F not permissable'); axis equal; xlim([-4 5]); ylim([-1 5]);
end

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
    
    plot(X, Y, '-o', 'LineWidth', 1, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    
    axis equal;
    xlim([-4 5]);
    ylim([-1 5]);
    grid on;
    title(title_str);
    xlabel('x_0');
    ylabel('y_0');
end

% function for calculating the dh matrix
function T = dh_matrix(theta, d, a, alpha)
    T = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
        0 sind(alpha) cosd(alpha) d;
        0 0 0 1];
end

function q = inverse_kinematics(x_TCP)
    l1 = 3;
    l2 = 1;
    x = x_TCP(1);
    y = x_TCP(2);
    
    % cos for theta2
    cos_th2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    
    % permissable range check
    if cos_th2 > 1 || cos_th2 < -1
        q = [NaN; NaN];
        return;
    end
    
    % calculate two options (Elbow Up / Elbow Down)
    th2_rad_1 = acos(cos_th2);
    th2_rad_2 = -acos(cos_th2);
    
    th1_rad_1 = atan2(y, x) - atan2(l2 * sin(th2_rad_1), l1 + l2 * cos_th2);
    th1_rad_2 = atan2(y, x) - atan2(l2 * sin(th2_rad_2), l1 + l2 * cos_th2);
    
    % convert to degree
    q1 = [th1_rad_1 * (180/pi); th2_rad_1 * (180/pi)];
    q2 = [th1_rad_2 * (180/pi); th2_rad_2 * (180/pi)];
    
    % Check restrictions: theta1 in [0, 110]° and theta2 in [-120, 120]°
    valid1 = (q1(1) >= 0 && q1(1) <= 110) && (q1(2) >= -120 && q1(2) <= 120);
    valid2 = (q2(1) >= 0 && q2(1) <= 110) && (q2(2) >= -120 && q2(2) <= 120);
    
    % choose viable solution
    if valid1
        q = q1;
    elseif valid2
        q = q2;
    else
        q = [NaN; NaN];
    end
end