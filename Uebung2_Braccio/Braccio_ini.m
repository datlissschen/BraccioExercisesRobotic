%=====================================
%
%   BRACCIO ROBOT SIMULATION & CONTROL
%
%-------------------------------------
%
%   Frank Bender, DHBW Stuttgart
%   23.12.25 - Initial Model Setup
%   04.01.26 - Model cleanup
%   28.07.26 - Added rigidBodyTree approach
%   29.07.26 - Now starting the inverse kinematics designer app
%   17.08.26 - Adding Teach-in Functionalities
%   23.08.26 - Switched to R2026a
%   05.09.26 - Test of HMI proto
%   18.09.26 - Redesign of HMI concept
%   20.10.26 - Animated gripper mechanism (braccioRobotVis)
%   20.10.26 - Cube pick and place interaction (braccioCubes)
%   21.10.26 - Cubes 25 mm, grasping adapted to calibrated real gripper
%=====================================

close all
clear
clc


dt = 0.01;                  % Zeitschrittweite in [s]
simParams.dt_filt = 0.01;   % Filterzeitkonstante in [s]
eTol = 0.01;                % Toleranzwert für Steilheitsbegrenzer [-]

%% DH Parameters

% Link 1 - Base
braccio.d1      = 0.0715;    % [m]
braccio.a1      = 0;    % [m]
braccio.alpha1  = deg2rad(90);    % [rad]
braccio.theta1  = 0;    % [rad]

% Link 2 - Shoulder
braccio.d2      = 0;    % [m]        
braccio.a2      = 0.125;    % [m] 
braccio.alpha2  = 0;    % [rad]
braccio.theta2  = deg2rad(90);    % [rad]

% Link 3 - Elbow
braccio.d3      = 0;    % [m]
braccio.a3      = 0.125;    % [m] 
braccio.alpha3  = 0;    % [rad]
braccio.theta3  = 0;    % [rad]

% Link 4 - Wrist
braccio.d4      = 0;    % [m]
braccio.a4      = 0;    % [m]
braccio.alpha4  = deg2rad(90);    % [rad]
braccio.theta4  = deg2rad(90);    % [rad]

% Link 5 - Wrist Rotation
braccio.d5      = 0.130;    % [m]
braccio.a5      = 0;    % [m]
braccio.alpha5  = 0;    % [rad]
braccio.theta5  = 0;    % [rad]

% -------------------------------------

%% rigidBodyTree
braccioRobot = rigidBodyTree;

% Dateipfade (aus deinem vorherigen Skript)
file_base  = 'Braccio_STL/ROBOT_STL/Base.STL';
file_link1 = 'Braccio_STL/ROBOT_STL/Link_1.STL';
file_link2 = 'Braccio_STL/ROBOT_STL/Link_2.STL'; 
file_link3 = 'Braccio_STL/ROBOT_STL/Link_3.STL';
file_link4 = 'Braccio_STL/ROBOT_STL/Link_4.STL';
file_grip  = 'Braccio_STL/TOOLS_STL/gripper.STL';

% Reduce polygon complexity
TR0 = stlread(file_base);
f0.faces = TR0.ConnectivityList;
f0.vertices = TR0.Points;
f0_reduced = reducepatch(f0, 0.1);
TR0_reduced = triangulation(f0_reduced.faces, f0_reduced.vertices);
reducedFilePath0 = fullfile(tempdir, 'Base_reduced.stl');
stlwrite(TR0_reduced,reducedFilePath0 );

TR1 = stlread(file_link1);
f1.faces = TR1.ConnectivityList;
f1.vertices = TR1.Points;
f1_reduced = reducepatch(f1, 0.1);
TR1_reduced = triangulation(f1_reduced.faces, f1_reduced.vertices);
reducedFilePath1 = fullfile(tempdir, 'Link_1_reduced.stl');
stlwrite(TR1_reduced,reducedFilePath1 );

TR2 = stlread(file_link2);
f2.faces = TR2.ConnectivityList;
f2.vertices = TR2.Points;
f2_reduced = reducepatch(f2, 0.1);
TR2_reduced = triangulation(f2_reduced.faces, f2_reduced.vertices);
reducedFilePath2 = fullfile(tempdir, 'Link_2_reduced.stl');
stlwrite(TR2_reduced,reducedFilePath2 );

TR3 = stlread(file_link3);
f3.faces = TR3.ConnectivityList;
f3.vertices = TR3.Points;
f3_reduced = reducepatch(f3, 0.1);
TR3_reduced = triangulation(f3_reduced.faces, f3_reduced.vertices);
reducedFilePath3 = fullfile(tempdir, 'Link_3_reduced.stl');
stlwrite(TR3_reduced,reducedFilePath3 );

TR4 = stlread(file_link4);
f4.faces = TR4.ConnectivityList;
f4.vertices = TR4.Points;
f4_reduced = reducepatch(f4, 0.1);
TR4_reduced = triangulation(f4_reduced.faces, f4_reduced.vertices);
reducedFilePath4 = fullfile(tempdir, 'Link_4_reduced.stl');
stlwrite(TR4_reduced,reducedFilePath4 );

TR5 = stlread(file_grip);
f5.faces = TR5.ConnectivityList;
f5.vertices = TR5.Points;
f5_reduced = reducepatch(f5, 0.1);
TR5_reduced = triangulation(f5_reduced.faces, f5_reduced.vertices);
reducedFilePath5 = fullfile(tempdir, 'Link_5_reduced.stl');
stlwrite(TR5_reduced,reducedFilePath5 );

% Define color for Braccio robot
braccioOrange = [1 0.3333 0];
gripperColor = [0.6667 0.6667 0.498];
%braccioOrange = [0.5 0.5 0.5];

% --- BASE ---
addVisual(braccioRobot.Base, 'Mesh', reducedFilePath0, trvec2tform([0 0 0]),FaceColor=braccioOrange, FaceAlpha=1);

% --- Link 1 (Shoulder) ---
body1 = rigidBody('body1');
jnt1  = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1, [braccio.a1, braccio.alpha1, braccio.d1, 0], 'dh');
jnt1.PositionLimits = ([0 180]-90)*pi/180;
body1.Joint = jnt1;
addVisual(body1, 'Mesh', reducedFilePath1, trvec2tform([0 0 0]),FaceColor=braccioOrange, FaceAlpha=1);
addBody(braccioRobot, body1, 'base');

% --- Link 2 (Elbow) ---
body2 = rigidBody('body2');
jnt2  = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2, [braccio.a2, braccio.alpha2, braccio.d2, 0], 'dh');
jnt2.HomePosition = braccio.theta2;
jnt2.PositionLimits = ([15, 165])*pi/180;
body2.Joint = jnt2;
addVisual(body2, 'Mesh', reducedFilePath2, trvec2tform([0 0 0]),FaceColor=braccioOrange, FaceAlpha=1);
addBody(braccioRobot, body2, 'body1');

% --- Link 3 (Wrist Pitch) ---
body3 = rigidBody('body3');
jnt3  = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3, [braccio.a3, braccio.alpha3, braccio.d3, 0], 'dh');
jnt3.PositionLimits = ([0 180]-90)*pi/180;
body3.Joint = jnt3;
addVisual(body3, 'Mesh', reducedFilePath3, trvec2tform([0 0 0]),FaceColor=braccioOrange, FaceAlpha=1);
addBody(braccioRobot, body3, 'body2');

% --- Link 4 (Wrist Roll) ---
body4 = rigidBody('body4');
jnt4  = rigidBodyJoint('jnt4','revolute');
setFixedTransform(jnt4, [braccio.a4, braccio.alpha4, braccio.d4, 0], 'dh');
jnt4.PositionLimits = ([0 180])*pi/180;
jnt4.HomePosition = braccio.theta4;
body4.Joint = jnt4;
addVisual(body4, 'Mesh', reducedFilePath4, trvec2tform([0 0 0]),FaceColor=braccioOrange, FaceAlpha=1);
addBody(braccioRobot, body4, 'body3');


% --- Link 5 (End Effector / Gripper Base) ---
body5 = rigidBody('body5');
jnt5  = rigidBodyJoint('jnt5','revolute');
setFixedTransform(jnt5, [braccio.a5, braccio.alpha5, braccio.d5, 0], 'dh');
jnt5.PositionLimits = ([0 180]-90)*pi/180;
body5.Joint = jnt5;
addVisual(body5, 'Mesh', reducedFilePath5, trvec2tform([0 0 0]),FaceColor=gripperColor, FaceAlpha=1);
addBody(braccioRobot, body5, 'body4');

% --- Define TCP -------------------
tcpBody = rigidBody('tcp');
tcpJoint = rigidBodyJoint('tcp_joint', 'fixed');
tcpPosition_xyz = [-0.006, 0, 0.05]; 
tform = trvec2tform(tcpPosition_xyz);
setFixedTransform(tcpJoint, tform);
tcpBody.Joint = tcpJoint;
addBody(braccioRobot, tcpBody, 'body5');

% --- Link 6 (Gripper) --------------------------
jnt6.PositionLimits = ([10 73])*pi/180;

% --- Visualisierungsmodell mit beweglichem Greifer (Simulation 3D Robot) ---
% Kopie von braccioRobot, gripper.STL in Zahnraeder, Lenker und Finger zerlegt.
% Konfiguration: [q1..q5, Braccio_gripperJoints(q6, braccioGripper)]
braccioRobotVis = copy(braccioRobot);
[braccioRobotVis, braccioGripper] = Braccio_gripperSetup(braccioRobotVis, file_grip, 'body5', gripperColor);
braccioInitConfigVis = [0 pi/2 0 pi/2 0, Braccio_gripperJoints(jnt6.PositionLimits(1), braccioGripper)'];

%%  Visualisierung
figure;
show(braccioRobot);
axis equal;

interactiveGUI = interactiveRigidBodyTree(braccioRobot,'MarkerScaleFactor',1);

%% Objekte

% Plattform
platform = collisionBox(0.6,0.3,0.02);
platform.Pose = trvec2tform([0.1 0 -0.01]);

% Aufnahmeposition
box_trans = [0.25 0 -.005];
pickupMarker = collisionBox(0.025,0.025,0.02);
pickupMarker.Pose = trvec2tform(box_trans);

% Würfel für Pick-and-Place
cube = collisionBox(0.025,0.025,0.025);
cube.Pose = trvec2tform([0.25 0 0.025/2+0.005]);

% Würfel für Regal
cube2 = collisionBox(0.025,0.025,0.025);
cube2.Pose = trvec2tform([0.35 0 0.025/2+0.005]);

% Aufnahmeposition Würfel 2
pickupMarker2 = collisionBox(0.025,0.025,0.02);
pickupMarker2.Pose = trvec2tform([0.35 0 -.005]);

% Hochregal
regal = collisionBox(0.04,0.06,0.1);
regal.Pose = trvec2tform([0.4 0 0.05]);

% Sortierbox 1 (150 mm lang in y, 100 mm breit in x)
box1_floor = collisionBox(0.1,0.15,0.005);
box1_floor.Pose = trvec2tform([0 0.225 -.015]);

box1_wall1 = collisionBox(0.1,0.005,0.1);
box1_wall1.Pose = trvec2tform([0 0.15 .05-0.02]);

box1_wall2 = collisionBox(0.1,0.005,0.1);
box1_wall2.Pose = trvec2tform([0 0.3 .05-0.02]);

box1_wall3 = collisionBox(0.005,0.15,0.1);
box1_wall3.Pose = trvec2tform([0.05 0.225 0.05-0.02]);

box1_wall4 = collisionBox(0.005,0.15,0.1);
box1_wall4.Pose = trvec2tform([-0.05 0.225 0.05-0.02]);

%% Würfel-Interaktion (Greifen und Absetzen in der Simulation)

% Logik: Braccio_cubeGrasp.m, Subsystem CubeInteraction im Simulationsmodell
braccioCubes.size = cube.X;                                       % Kantenlaenge [m]
braccioCubes.initPos = [tform2trvec(cube.Pose); tform2trvec(cube2.Pose)];
supportBoxes = {platform, pickupMarker, pickupMarker2, regal, box1_floor};
braccioCubes.supports = cell2mat(cellfun(@(b) [b.Pose(1,4)+[-1 1]*b.X/2, ...
    b.Pose(2,4)+[-1 1]*b.Y/2, b.Pose(3,4)+b.Z/2], supportBoxes, 'UniformOutput', false)'); % [xmin xmax ymin ymax zTop]
braccioCubes.groundZ = -0.02;                                     % Boden ausserhalb der Auflagen [m]
% Greifen eines 25-mm-Wuerfels (Greifer am realen Roboter kalibriert, siehe
% Braccio_gripperSetup): Federzinken beruehren ab q6 = 42 deg, bei 55 deg
% (Hochregal) sind sie ca. 9 deg eingefedert, ab ca. 58 deg liegen die starren
% Fingerteile an und der Servo kann nicht weiter schliessen
braccioCubes.q6Grasp   = 45*pi/180;   % Greifen beim Schliessen ueber diesen Winkel
braccioCubes.q6Release = 43*pi/180;   % Loslassen beim Oeffnen unter diesen Winkel
braccioCubes.q6Contact = 58*pi/180;   % Anschlag am Wuerfel (Greifer-Animation)
braccioCubes.graspCenter   = [0 0.002 0];      % Mitte Greifbereich im TCP-KS [m] (Fingermitte)
braccioCubes.graspHalfSize = [0.015 0.012 0.022]; % Halbe Abmessungen Greifbereich [m]
% Respawning: leere Startplaetze mit Wuerfeln aus einem Vorrat neu belegen
braccioCubes.initColor = [0 0 1; 0 1 0];            % Farben cube1 (blau), cube2 (gruen)
braccioCubes.colors = [0 0 1; 0 1 0; 1 0 0; 1 1 0]; % Zufallsfarben: blau, gruen, rot, gelb
braccioCubes.nPool = 6;                 % Anzahl Vorratswuerfel (Actors cubeP1..cubeP6)
braccioCubes.hiddenPos = [0 0 -1];      % Parkposition unsichtbarer Vorratswuerfel [m]
braccioCubes.respawnDelay = 1.0;        % Wartezeit bei leerem Startplatz [s]
braccioCubes.respawnClearance = 0.06;   % Mindestabstand TCP - Startplatz [m]
braccioCubes.slotTol = 0.012;           % Startplatz gilt als belegt innerhalb dieses Radius [m]
% Beim Respawning werden Wuerfel auf diesen Ablagen entfernt (Hochregal)
braccioCubes.clearZones = [regal.Pose(1,4)+[-1 1]*regal.X/2, ...
    regal.Pose(2,4)+[-1 1]*regal.Y/2, regal.Pose(3,4)+regal.Z/2]; % [xmin xmax ymin ymax zTop]
braccioCubes.dt = 0.01;                 % Schrittweite = FixedStep des Modells [s]

%% E-Fences / Arbeitsraumbeschränkung

% ----------- TODO ---------------------
efence1_coord.x = -0.2;  
% --------------------------------------
efence1 = collisionBox(0.002,0.3,0.3);
efence1.Pose = trvec2tform([efence1_coord.x 0 0.15]);


%% Inverse Kinematics Designer

inverseKinematicsDesigner("Braccio_iksession.mat")

%% Bahn für Regal-Positionierung
cartesianPath.time = [0       2       4       6       8       10      12      14      16];
cartesianPath.x =    [0.4     0.4     0.4     0.4     0.4     0.4     0.4     0.4     0.4]; % TODO: Hier müssen die x-Sollwerte passend zum Zeit-Vektor eingetragen werden
cartesianPath.z =    [0.2     0.2     0.2     0.2     0.2     0.2     0.2     0.2     0.2]; % TODO: Hier müssen die z-Sollwerte passend zum Zeit-Vektor eingetragen werden
cartesianPath.angle =[0       0       0       0       0       0       0       0       0];   % Soll-Winkelstellung
cartesianPath.q6 =   [10      60      60      60      60      10      10      10      10]*pi/180;   % Greifer Stellung
cartesianPath.startConfig = [0 0 0 0 0 0]'*pi/180; % TODO: Via IK Designer ermittelte Anfangskonfiguration

figure('name','Regal-Positionierung in kartesischen Koordinaten')
subplot(3,2,1);
plot(cartesianPath.time,cartesianPath.x,'r.-')
ylabel('x [m]')
grid on
subplot(3,2,3);
plot(cartesianPath.time,cartesianPath.z,'r.-')
ylabel('z [m]')
grid on
subplot(3,2,5);
plot(cartesianPath.time,cartesianPath.q6*180/pi,'r.-')
ylabel('q6 [deg]')
xlabel('Zeit t [s]')
subplot(3,2, [2 4 6]);
plot(cartesianPath.x,cartesianPath.z,'r-o')
xlabel('x [m]')
ylabel('z [m]')
grid on

