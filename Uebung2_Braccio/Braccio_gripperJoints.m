function qGrip = Braccio_gripperJoints(q6, gripper, cubeHeld)
%BRACCIO_GRIPPERJOINTS Gelenkwinkel des Greifer-Mechanismus aus dem Servowinkel q6.
%
%   qGrip = Braccio_gripperJoints(q6, gripper, cubeHeld)
%
%   q6      Servowinkel Greifer [rad] (10 deg = offen, 73 deg = geschlossen)
%   gripper Geometrie-Struktur aus Braccio_gripperSetup
%   cubeHeld [Breite; y-Mitte] des gehaltenen Wuerfels im body5-KS [m]
%            (optional, Breite 0 = kein Wuerfel)
%   qGrip   8x1 [gripGearR; gripGearL; gripLinkR; gripLinkL; gripFingerR;
%           gripFingerL; gripTineR; gripTineL]
%           Fingerwinkel relativ zum jeweiligen Zahnrad-Arm, Zinkenwinkel
%           relativ zum Finger
%
%   Je Seite wird das Viergelenk Rahmen - Zahnrad-Arm - Finger - Lenker
%   geschlossen geloest. Die innere Federzinke biegt sich so weit auf, dass
%   ihre Spitze an der Wuerfelflaeche anliegt. Codegenerierbar (MATLAB
%   Function Block).
%
%   F. Bender, DHBW Stuttgart
%   20.10.26 - Initial version
%   21.10.26 - Servo-Zuordnung am realen Greifer kalibriert
%   21.10.26 - Phasenversatz links, federnde innere Zinken

%#codegen

if nargin < 3
    cubeHeld = [0; 0];
end

phiR = gripper.servoGain*(q6 - gripper.q6_stl);
phiL = gripper.gearRatio*phiR + gripper.phaseL;

[linkR, fingerR, BR] = solveFourBar(gripper.R, phiR);
[linkL, fingerL, BL] = solveFourBar(gripper.L, phiL);

tineR = 0;
tineL = 0;
if cubeHeld(1) > 0
    tineR = tineDeflection(gripper.R, BR, fingerR, cubeHeld(2) + cubeHeld(1)/2, 1);
    tineL = tineDeflection(gripper.L, BL, fingerL, cubeHeld(2) - cubeHeld(1)/2, -1);
end

qGrip = [phiR; phiL; linkR; linkL; wrapAngle(fingerR - phiR); wrapAngle(fingerL - phiL); ...
    tineR; tineL];
end

function alpha = tineDeflection(s, B, theta, yFace, side)
% Biegewinkel der Zinke um ihren Fuss, damit die Zinkenspitze nicht ueber
% die Wuerfelflaeche yFace hinaus nach innen ragt
% side = 1: Wuerfelflaeche bei kleinerem y als die Zinke (rechter Finger)
Q = B + rot2(s.tineRoot - s.B, theta);
T = B + rot2(s.tineTip - s.B, theta);
alpha = 0;
if side*(T(1) - yFace) < 0
    % Q(1) + v(1)*cos(a) - v(2)*sin(a) = yFace  ->  r*cos(a + phi) = c
    v = T - Q;
    r = norm(v);
    phi = atan2(v(2), v(1));
    c = min(max((yFace - Q(1))/r, -1), 1);
    a1 = acos(c) - phi;
    a2 = -acos(c) - phi;
    a1 = wrapAngle(a1);
    a2 = wrapAngle(a2);
    if abs(a1) <= abs(a2)
        alpha = a1;
    else
        alpha = a2;
    end
end
end

function p = rot2(v, a)
p = [cos(a)*v(1) - sin(a)*v(2), sin(a)*v(1) + cos(a)*v(2)];
end

function a = wrapAngle(a)
a = atan2(sin(a), cos(a));
end

function [psi, theta, B] = solveFourBar(s, phi)
% psi:   Drehung des Lenkers um D
% theta: absolute Drehung des Fingers (Koppel)
% B:     aktuelle Lage des Gelenks Zahnrad-Arm <-> Finger
c = cos(phi); sn = sin(phi);
AB = s.B - s.A;
B = s.A + [c*AB(1) - sn*AB(2), sn*AB(1) + c*AB(2)];

% Schnitt der Kreise um B (Koppellaenge) und D (Lenkerlaenge),
% Loesung naeher an der Ausgangslage C waehlen
rBC = norm(s.C - s.B);
rDC = norm(s.C - s.D);
dv = s.D - B;
d = norm(dv);
d = min(max(d, abs(rBC - rDC) + 1e-9), rBC + rDC - 1e-9);
a = (rBC^2 - rDC^2 + d^2)/(2*d);
h = sqrt(max(rBC^2 - a^2, 0));
m = B + a*dv/d;
n = [-dv(2), dv(1)]/d;
C1 = m + h*n;
C2 = m - h*n;
if norm(C1 - s.C) <= norm(C2 - s.C)
    C = C1;
else
    C = C2;
end

psi   = angleBetween(s.C - s.D, C - s.D);
theta = angleBetween(s.C - s.B, C - B);
end

function a = angleBetween(v0, v1)
% Vorzeichenbehafteter Winkel von v0 nach v1 im Bereich (-pi, pi]
a = atan2(v0(1)*v1(2) - v0(2)*v1(1), v0(1)*v1(1) + v0(2)*v1(2));
end
