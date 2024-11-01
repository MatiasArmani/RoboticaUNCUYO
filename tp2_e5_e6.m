clc; clear; close all;
conversion = 180/pi;
plotvol([-2 2 -2 2 -2 2]); % Ajustar los límites para 3D

% Definir un marco de referencia en el origen
T0 = eye(4,4); % Matriz identidad 4x4 para 3D
trplot(T0, 'frame', '0', 'color', 'b'); % Graficar el marco

% Definir una traslación en 3D
Rotacion_1 = troty(pi/4);
Traslacion_1 = transl(0, 0, 1); % Traslación a (1, 1, 1)
M1 = Rotacion_1 * Traslacion_1;
M2 = Traslacion_1 * Rotacion_1; 

trplot(M1, 'frame', 'M1', 'color', 'r'); % Graficar el marco rotado y despues trasladado
trplot(M2, 'frame', 'M2', 'color', 'g'); % Graficar el marco trasladado y despues rotado

Op = [0.5; 0; 1; 1];
% plot_point(Op, 'label', 'Punto Op', 'solid', 'ko');
plot3(Op(1),Op(2),Op(3),'ko')
text(Op(1),Op(2),Op(3), sprintf('(%.1f, %.1f, %.1f)', Op(1),Op(2),Op(3)), 'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'bottom');
Op_respecto_m1 = inv(M1) * Op;
Op_respecto_m2 = inv(M2) * Op;
view(3);

fprintf("El vector posicion del punto Op respecto del sistema de coordenadas M1 es: \n");
disp(Op_respecto_m1(1:3));

fprintf("El vector posicion del punto Op respecto del sistema de coordenadas M2 es: \n");
disp(Op_respecto_m2(1:3));

