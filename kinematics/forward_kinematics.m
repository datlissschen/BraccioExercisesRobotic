%% Initialize
qA = [20;45];
qB = [45; -30];
qC = [120; -60];

%% Calculate TCP
tcpA = forward(qA);
tcpB = forward(qB);
tcpC = forward(qC);

%% Function for calculating forward kinematics
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