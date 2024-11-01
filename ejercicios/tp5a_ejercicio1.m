clc; clear; close all;

x = 1; % coordenadas del punto P respecto al sistema 0
y = 2;

a1 = 2; % Brazo 1
a2 = 1; % Brazo 2

P = [x;y;0;1];

r = sqrt(x^2 + y^2);
angulo_beta = atan2(P(2),P(1));

angulo_alfa = acos((a1^2+r^2-a2^2)/(2*a1*r));

q1_solucion1 = angulo_beta + angulo_alfa;
q1_solucion2 = angulo_beta - angulo_alfa;

T1_respectoa0_solucion1 = transl(a1*cos(q1_solucion1),a1*sin(q1_solucion1),0)*trotz(q1_solucion1);
T1_respectoa0_solucion2 = transl(a1*cos(q1_solucion2),a1*sin(q1_solucion2),0)*trotz(q1_solucion2);

P1_solucion1 = T1_respectoa0_solucion1\P;
P1_solucion2 = T1_respectoa0_solucion2\P;

q2_solucion1 = atan2(P1_solucion1(2),P1_solucion1(1));
q2_solucion2 = atan2(P1_solucion2(2),P1_solucion2(1));


fprintf("La solucion considerando codo arriba es: \nq1 = %0.4f°, q2 = %0.4f°\n\n",q1_solucion1*180/pi, q2_solucion1*180/pi);

fprintf("La solucion considerando codo abajo es: \nq1 = %0.4f°, q2 = %0.4f°\n\n",q1_solucion2*180/pi, q2_solucion2*180/pi);

dh = [0 0 a1 0 0;
      0 0 a2 0 0];
R = SerialLink(dh,'name','Robot 2gdl');
q = [0 0];
% Límites articulares basados en las restricciones físicas del robot
angulo_limite = 225;
R.qlim(1,1:2) = [-angulo_limite, angulo_limite]*pi/180;
R.qlim(2,1:2) = [ -angulo_limite,  angulo_limite]*pi/180;

figure
R.plot(q, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
R.teach();
