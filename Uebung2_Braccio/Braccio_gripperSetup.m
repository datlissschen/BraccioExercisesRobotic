function [robot, gripper] = Braccio_gripperSetup(robot, stlFile, parentName, color, reduceRatio)
%BRACCIO_GRIPPERSETUP Zerlegt gripper.STL in bewegte Teile und haengt sie
%   als eigene Koerper an den rigidBodyTree.
%
%   [robot, gripper] = Braccio_gripperSetup(robot, stlFile, parentName, color, reduceRatio)
%
%   Ein vorhandenes Visual von parentName wird durch die statischen Teile
%   (Rahmen, Servo, Schrauben) ersetzt.
%
%   Der Greifer besteht aus zwei Viergelenken (je Seite: Zahnrad-Arm,
%   Lenker, Finger als Koppel). Das rechte Zahnrad sitzt auf der Servowelle
%   (q6), das linke kaemmt 1:1 gegenlaeufig. Die innere Zinke jedes Fingers
%   ist federnd und wird als eigener Koerper mit Biegegelenk am Zinkenfuss
%   abgebildet. Alle Drehachsen liegen parallel zur x-Achse des
%   Elternkoerpers (body5).
%
%   Neue Gelenke (Reihenfolge in der Konfiguration nach jnt1..jnt5):
%     gripGearR, gripGearL, gripLinkR, gripLinkL, gripFingerR, gripFingerL,
%     gripTineR, gripTineL
%   Die Gelenkwinkel berechnet Braccio_gripperJoints(q6, gripper, cubeHeld).
%
%   F. Bender, DHBW Stuttgart
%   20.10.26 - Initial version
%   21.10.26 - Servo-Zuordnung am realen Greifer kalibriert
%   21.10.26 - Phasenversatz links, federnde innere Zinken

arguments
    robot rigidBodyTree
    stlFile (1,:) char
    parentName (1,:) char = 'body5'
    color (1,3) double = [0.6667 0.6667 0.498]
    reduceRatio (1,1) double = 0.1
end

%% Geometrie (STL-Koordinaten = Koordinaten von body5) [m]
% Drehpunkte in der y-z-Ebene, per Kreisfit aus den Bohrungen ermittelt
gripper.R.A = [ 12.56 -41.10]*1e-3;   % Zahnrad rechts (Servowelle)
gripper.R.D = [ 32.61 -41.00]*1e-3;   % Lenker rechts, Rahmenlager
gripper.R.B = [ 31.39 -17.74]*1e-3;   % Zahnrad-Arm <-> Finger
gripper.R.C = [ 55.99 -22.19]*1e-3;   % Lenker <-> Finger
gripper.L.A = [-11.49 -41.00]*1e-3;   % Zahnrad links (angetrieben)
gripper.L.D = [-31.49 -41.00]*1e-3;
gripper.L.B = [-23.65 -13.58]*1e-3;
gripper.L.C = [-48.50 -16.29]*1e-3;

% Innere Federzinken: Schlitzmitte (Trennlinie zur aeusseren Zinke) [y z; y z],
% Zinkenfuss (Biegegelenk) und innerste Stelle der Zinkenspitze
gripper.R.slot     = [ 37.35 -1.00;  35.45 50.00]*1e-3;
gripper.R.tineRoot = [ 34.86 -1.00]*1e-3;
gripper.R.tineTip  = [ 29.60 50.72]*1e-3;
gripper.L.slot     = [-28.30  5.00; -23.75 53.00]*1e-3;
gripper.L.tineRoot = [-25.49  5.00]*1e-3;
gripper.L.tineTip  = [-17.06 54.59]*1e-3;

% Servo-Zuordnung, kalibriert am realen Roboter (Schraubenpositionen in
% Video 20.10.26 und Fotos 21.10.26, Restfehler ca. 0.3 mm):
% q6 = 10 deg -> Zahnrad rechts -4.8 deg, q6 = 50 deg -> +26.8 deg (rel. STL-Lage)
% (ServoCtrl bildet q6 = 10..73 deg auf nur 500..1000 us ab)
gripper.q6_stl = 16.1*pi/180;         % Sollwinkel der im STL gezeichneten Lage [rad]
gripper.servoGain = 0.79;             % Zahnradwinkel je Sollwinkel [-]
gripper.gearRatio = -1;               % linkes Zahnrad dreht gegenlaeufig
gripper.phaseL = 12.5*pi/180;         % linkes Zahnrad real ca. 1 Zahn versetzt zum STL [rad]

%% STL in zusammenhaengende Teile zerlegen
TR = stlread(stlFile);
[~, ~, ic] = unique(round(TR.Points*1e7), 'rows');
F = ic(TR.ConnectivityList);
nV = max(ic);
P = zeros(nV, 3);
P(ic, :) = TR.Points;
G = graph([F(:,1); F(:,2); F(:,3)], [F(:,2); F(:,3); F(:,1)], [], nV);
comp = conncomp(G);
faceComp = comp(F(:,1));
nComp = max(comp);

% Zuordnung der Teile ueber ihren Schwerpunkt [mm] (Referenz: gripper.STL)
refParts = { ...
    'static',  [  0.16   6.82 -49.41];  % Rahmen
    'gearR',   [-11.21  24.41 -26.40];  % Zahnrad-Arm rechts
    'gearR',   [-13.74  13.45 -40.01];  % Servohorn
    'fingerL', [ -6.09 -31.90   6.07];
    'fingerR', [ -6.12  41.00   1.27];
    'linkL',   [-12.02 -40.02 -28.61];  % Lenker links vorne
    'linkR',   [-12.02  44.34 -31.56];  % Lenker rechts vorne
    'gearL',   [ -9.95 -16.40 -29.93];  % Zahnrad-Arm links
    'gearR',   [-11.06  12.56 -41.10];  % Schraube Servowelle
    'static',  [  2.38  12.56 -42.94];  % Servo
    'linkL',   [ -0.19 -40.02 -28.60];  % Lenker links hinten
    'linkR',   [ -0.19  44.34 -31.57];  % Lenker rechts hinten
    'static',  [ -0.21   1.95 -72.23];  % Schraube unten
    'static',  [ 10.83   7.76 -34.18];
    'static',  [ 10.83  17.37 -34.18];
    'static',  [ 17.66  12.61 -43.76]}; % Servo-Aufkleber
refCentroids = vertcat(refParts{:,2})*1e-3;

partName = strings(nComp, 1);
for k = 1:nComp
    c = mean(P(unique(F(faceComp == k, :)), :), 1);
    [dist, iRef] = min(vecnorm(refCentroids - c, 2, 2));
    if dist > 2e-3
        warning('Braccio:gripperPart', ...
            'Teil %d ohne Referenz (Abstand %.1f mm), wird statisch angenommen.', k, dist*1e3);
        partName(k) = "static";
    else
        partName(k) = refParts{iRef, 1};
    end
end

% Innere Zinken von den Fingern abtrennen (Flaechenschwerpunkt innerhalb
% der Schlitzmitte)
faceGroup = partName(faceComp(:));
fc = (P(F(:,1), :) + P(F(:,2), :) + P(F(:,3), :))/3;
faceGroup(faceGroup == "fingerR" & isTine(fc, gripper.R.slot, -1)) = "tineR";
faceGroup(faceGroup == "fingerL" & isTine(fc, gripper.L.slot, 1)) = "tineL";

%% Koerper anlegen
% Name, Elternkoerper, Gelenkpunkt [y z] im Koordinatensystem von body5
bodyDef = { ...
    'gripGearR',   parentName,  gripper.R.A, 'gearR';
    'gripGearL',   parentName,  gripper.L.A, 'gearL';
    'gripLinkR',   parentName,  gripper.R.D, 'linkR';
    'gripLinkL',   parentName,  gripper.L.D, 'linkL';
    'gripFingerR', 'gripGearR', gripper.R.B, 'fingerR';
    'gripFingerL', 'gripGearL', gripper.L.B, 'fingerL';
    'gripTineR',   'gripFingerR', gripper.R.tineRoot, 'tineR';
    'gripTineL',   'gripFingerL', gripper.L.tineRoot, 'tineL'};

% Statische Teile ersetzen das bisherige Greifer-Visual des Elternkoerpers
parentBody = copy(robot.getBody(parentName));
clearVisual(parentBody);
meshFile = writePartMesh("static", [0 0]);
addVisual(parentBody, 'Mesh', meshFile, eye(4), FaceColor=color, FaceAlpha=1);
replaceBody(robot, parentName, parentBody);

pivotOf = containers.Map({parentName}, {[0 0]});
for k = 1:size(bodyDef, 1)
    [name, parent, pivot, group] = bodyDef{k, :};
    body = rigidBody(name);
    jnt = rigidBodyJoint([name '_jnt'], 'revolute');
    jnt.JointAxis = [1 0 0];
    jnt.PositionLimits = [-pi pi];
    rel = pivot - pivotOf(parent);
    setFixedTransform(jnt, trvec2tform([0 rel]));
    body.Joint = jnt;
    meshFile = writePartMesh(string(group), pivot);
    addVisual(body, 'Mesh', meshFile, eye(4), FaceColor=color, FaceAlpha=1);
    addBody(robot, body, parent);
    pivotOf(name) = pivot;
end

    function file = writePartMesh(group, pivot)
        % Teile einer Gruppe zusammenfassen, reduzieren und relativ zum
        % Gelenkpunkt als STL in tempdir ablegen
        sel = faceGroup == group;
        fv.faces = F(sel, :);
        fv.vertices = P;
        if nnz(sel) > 2000
            fv = reducepatch(fv, reduceRatio);
        end
        [used, ~, iNew] = unique(fv.faces(:));
        faces = reshape(iNew, [], 3);
        verts = fv.vertices(used, :) - [0 pivot];
        file = fullfile(tempdir, sprintf('Braccio_gripper_%s.stl', group));
        stlwrite(triangulation(faces, verts), file);
    end
end

function tf = isTine(p, slot, side)
% Punkte jenseits der Schlitzmitte (side = -1: kleineres y, +1: groesseres y)
% und unterhalb des Schlitzanfangs
yMid = slot(1,1) + (slot(2,1) - slot(1,1))*(p(:,3) - slot(1,2))/(slot(2,2) - slot(1,2));
tf = p(:,3) > slot(1,2) & side*(p(:,2) - yMid) > 0;
end
