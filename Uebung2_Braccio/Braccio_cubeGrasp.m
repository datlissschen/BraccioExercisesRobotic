function [cubePos, cubeRot, cubeColor, cubeTransp, gripVis, graspIdx] = Braccio_cubeGrasp(T_tcp, q6, reset, respawn, P)
%BRACCIO_CUBEGRASP Greifen, Absetzen und Respawning der Wuerfel in der Simulation.
%
%   [cubePos, cubeRot, cubeColor, cubeTransp, gripVis, graspIdx] = ...
%       Braccio_cubeGrasp(T_tcp, q6, reset, respawn, P)
%
%   T_tcp      4x4 Pose des TCP im Basis-KS (Get Transform, Koerper 'tcp')
%   q6         Servowinkel Greifer [rad]
%   reset      true -> alle Wuerfel auf Startzustand zuruecksetzen
%   respawn    true -> leere Startplaetze automatisch neu belegen
%   P          Parameter-Struktur braccioCubes (siehe Braccio_kinematics_loesung.m)
%
%   Wuerfel 1..nSlot liegen zu Beginn auf den Startplaetzen P.initPos,
%   Wuerfel nSlot+1..nSlot+P.nPool bilden einen unsichtbaren Vorrat.
%
%   cubePos    Nx3 Wuerfelpositionen [m] im Unreal-KS (y gespiegelt)
%   cubeRot    Nx3 Wuerfelrotation [roll pitch yaw] [rad] im Unreal-KS
%   cubeColor  Nx3 RGB-Farbe
%   cubeTransp Nx1 Transparenz (0 = sichtbar, 1 = unsichtbar)
%   gripVis    3x1 [q6; Breite; y-Mitte] fuer die Greifer-Animation
%              (Braccio_gripperJoints): q6 auf P.q6Contact begrenzt und
%              Wuerfelbreite > 0, solange ein Wuerfel gehalten wird
%   graspIdx   Index des gehaltenen Wuerfels (0 = keiner)
%
%   Zustandsautomat je Wuerfel:
%     Liegt     -> Gegriffen: q6 ueberschreitet P.q6Grasp und der
%                  Wuerfelmittelpunkt liegt im Greifbereich (TCP-KS); der
%                  Wuerfel wird zwischen den Fingern zentriert und ausgerichtet
%     Gegriffen -> Liegt:     q6 < P.q6Release (Federzinken entspannt), der
%                  Wuerfel behaelt nur den Gierwinkel
%   Schwerkraft: Liegende Wuerfel fallen in jedem Schritt auf die hoechste
%   Auflageflaeche darunter (feste Auflage oder anderer Wuerfel -> Stapel).
%   Wird ein Wuerfel unten aus einem Stapel entnommen, rutschen die oberen nach.
%   Respawning: Ist ein Startplatz P.respawnDelay lang leer, kein Wuerfel
%   gegriffen und der TCP weiter als P.respawnClearance entfernt, erscheint
%   dort ein Vorratswuerfel mit zufaelliger Farbe aus P.colors. Ist der
%   Vorrat erschoepft, wird der am laengsten abgelegte Vorratswuerfel verwendet.
%   Gleichzeitig verschwinden die Wuerfel auf den Ablagen P.clearZones (Hochregal).
%
%   F. Bender, DHBW Stuttgart
%   20.10.26 - Initial version
%   20.10.26 - Wuerfelvorrat und Respawning
%   20.10.26 - Nachrutschen von Stapeln, Vorzeichen Unreal-Rotation korrigiert
%   21.10.26 - Realistischeres Greifen (Zentrieren, Ausrichten), Hochregal leeren
%   21.10.26 - Breite des gehaltenen Wuerfels fuer federnde Zinken

%#codegen

nSlot = size(P.initPos, 1);
N = nSlot + P.nPool;
persistent pos R color active stamp nStamp gIdx T_off q6Prev slotTimer
if isempty(pos) || reset
    pos = [P.initPos; repmat(P.hiddenPos, P.nPool, 1)];
    R = repmat(eye(3), 1, 1, N);
    color = [P.initColor; repmat(P.colors(1, :), P.nPool, 1)];
    active = [true(nSlot, 1); false(P.nPool, 1)];
    stamp = zeros(N, 1);
    nStamp = 0;
    gIdx = 0;
    T_off = eye(4);
    q6Prev = q6;
    slotTimer = zeros(nSlot, 1);
end

R_tcp = T_tcp(1:3, 1:3);
p_tcp = T_tcp(1:3, 4);

if gIdx == 0
    % Greifen nur beim Schliessen des Greifers (Flanke ueber q6Grasp)
    if q6Prev < P.q6Grasp && q6 >= P.q6Grasp
        best = inf;
        for i = 1:N
            pRel = R_tcp'*(pos(i, :)' - p_tcp) - P.graspCenter(:);
            if active(i) && all(abs(pRel) <= P.graspHalfSize(:)) && norm(pRel) < best
                best = norm(pRel);
                gIdx = i;
            end
        end
        if gIdx > 0
            % Die Finger zentrieren den Wuerfel in Schliessrichtung (TCP-y) und
            % richten die naechstgelegene Wuerfelflaeche an den Fingern aus
            pRel = R_tcp'*(pos(gIdx, :)' - p_tcp);
            pRel(2) = P.graspCenter(2);
            T_off = [alignToFingers(R_tcp'*R(:, :, gIdx)) pRel; 0 0 0 1];
            T_cube = T_tcp*T_off;
            R(:, :, gIdx) = T_cube(1:3, 1:3);
            pos(gIdx, :) = T_cube(1:3, 4)';
        end
    end
else
    if q6 < P.q6Release
        % Absetzen: auf Auflageflaeche fallen lassen, nur Gierwinkel behalten
        Ri = R(:, :, gIdx);
        yaw = atan2(Ri(2, 1), Ri(1, 1));
        R(:, :, gIdx) = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        nStamp = nStamp + 1;
        stamp(gIdx) = nStamp;
        gIdx = 0;
    else
        T_cube = T_tcp*T_off;
        R(:, :, gIdx) = T_cube(1:3, 1:3);
        pos(gIdx, :) = T_cube(1:3, 4)';
    end
end
q6Prev = q6;

% Schwerkraft: liegende Wuerfel von unten nach oben auf ihre Auflage absenken
% (abgesetzte Wuerfel fallen, Stapel rutschen nach, wenn ein Wuerfel darunter
% entnommen wird)
[~, order] = sort(pos(:, 3));
for n = 1:N
    i = order(n);
    if active(i) && i ~= gIdx
        zRest = supportHeight(pos(i, :), P, pos, active, [i gIdx]) + P.size/2;
        if zRest < pos(i, 3)
            pos(i, 3) = zRest;
        end
    end
end

% Respawning leerer Startplaetze
for s = 1:nSlot
    slotPos = P.initPos(s, :);
    occupied = false;
    for i = 1:N
        d = pos(i, :) - slotPos;
        if active(i) && norm(d(1:2)) < P.slotTol && abs(d(3)) < P.size/2
            occupied = true;
        end
    end
    if respawn && ~occupied && gIdx == 0 && norm(p_tcp' - slotPos) > P.respawnClearance
        slotTimer(s) = slotTimer(s) + P.dt;
    else
        slotTimer(s) = 0;
    end
    if slotTimer(s) >= P.respawnDelay
        % Abgelegte Wuerfel im Hochregal verschwinden mit dem neuen Wuerfel
        for i = 1:N
            if active(i) && i ~= gIdx && inZone(pos(i, :), P.clearZones)
                pos(i, :) = P.hiddenPos;
                R(:, :, i) = eye(3);
                active(i) = false;
                stamp(i) = 0;
            end
        end
        k = nextPoolCube(active, stamp, gIdx, nSlot, N);
        if k > 0
            pos(k, :) = slotPos;
            R(:, :, k) = eye(3);
            color(k, :) = P.colors(randi(size(P.colors, 1)), :);
            active(k) = true;
            stamp(k) = 0;
        end
        slotTimer(s) = 0;
    end
end

gripVis = [q6; 0; P.graspCenter(2)];
if gIdx > 0
    gripVis(1:2) = [min(q6, P.q6Contact); P.size];
end
graspIdx = gIdx;

% Umrechnung ins linkshaendige Unreal-KS (y gespiegelt). Unreal-Rotator:
% Pitch positiv = x-Achse nach oben, Yaw positiv = Drehung nach rechts,
% Roll positiv = rechte Seite nach unten -> [roll, -pitch, -yaw]
cubePos = [pos(:, 1), -pos(:, 2), pos(:, 3)];
cubeRot = zeros(N, 3);
for i = 1:N
    eul = rotm2eul(R(:, :, i), 'ZYX');               % [yaw pitch roll]
    cubeRot(i, :) = [eul(3), -eul(2), -eul(1)];
end
cubeColor = color;
cubeTransp = double(~active);
end

function Rn = alignToFingers(Rc)
% Wuerfelrotation relativ zum TCP so nachdrehen, dass die Wuerfelachse, die
% am ehesten in Schliessrichtung (TCP-y) zeigt, genau parallel dazu liegt
[~, k] = max(abs(Rc(2, :)));
v = Rc(:, k)*sign(Rc(2, k));
t = [0; 1; 0];
ax = cross(v, t);
s = norm(ax);
c = dot(v, t);
if s < 1e-9
    Rn = Rc;
    return
end
ax = ax/s;
K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
Rn = (eye(3) + s*K + (1 - c)*(K*K))*Rc;
end

function tf = inZone(p, Z)
% Wuerfel liegt auf (oder gestapelt ueber) einer der Ablagen
% Z: [xmin xmax ymin ymax zTop] je Zeile
tf = false;
for j = 1:size(Z, 1)
    if p(1) >= Z(j, 1) && p(1) <= Z(j, 2) && p(2) >= Z(j, 3) && p(2) <= Z(j, 4) ...
            && p(3) >= Z(j, 5)
        tf = true;
    end
end
end

function k = nextPoolCube(active, stamp, gIdx, nSlot, N)
% Freier Vorratswuerfel, sonst der am laengsten abgelegte (nicht gegriffene)
k = 0;
for i = nSlot+1:N
    if ~active(i)
        k = i;
        return
    end
end
oldest = inf;
for i = nSlot+1:N
    if i ~= gIdx && stamp(i) > 0 && stamp(i) < oldest
        oldest = stamp(i);
        k = i;
    end
end
end

function z = supportHeight(p, P, pos, active, ignore)
% Hoechste Auflageflaeche unter dem Wuerfel (Oberkante <= Wuerfelmitte):
% feste Auflagen aus P.supports oder andere liegende Wuerfel (Stapeln).
% ignore: Indizes, die nicht als Auflage zaehlen (Wuerfel selbst, gegriffener)
z = P.groundZ;
zBottom = p(3) - P.size/2;
for j = 1:size(P.supports, 1)
    s = P.supports(j, :);   % [xmin xmax ymin ymax zTop]
    if p(1) >= s(1) && p(1) <= s(2) && p(2) >= s(3) && p(2) <= s(4) ...
            && s(5) <= zBottom + P.size/2 && s(5) > z
        z = s(5);
    end
end
for i = 1:size(pos, 1)
    zTop = pos(i, 3) + P.size/2;
    if ~any(ignore == i) && active(i) && all(abs(pos(i, 1:2) - p(1:2)) < P.size) ...
            && zTop <= zBottom + P.size/2 && zTop > z
        z = zTop;
    end
end
end
