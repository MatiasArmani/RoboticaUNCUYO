clc; clear; close all;

% Define la variable simbólica
syms theta
equation = -5*sin(theta) - 1 == 9.2361 - 2*4;  % Define la ecuación
solution = solve(equation, theta);  % Resuelve la ecuación

% Convierte la solución a valor numérico
numeric_solution = double(solution);

angulo = numeric_solution(1);

plotvol([0 10 0 6]);
T0 = eye(3,3);
trplot2(T0, 'frame', '0');
Traslacion = transl2(7,4);
Rotacion = trot2(angulo);
trplot2(Traslacion*Rotacion, 'framelabel', 'M', 'color', 'r');

% Punto Ma en el marco M
Punto_aM = [2; 1; 1];

% Punto Ma en el marco O
Punto_ao = Traslacion*Rotacion*Punto_aM;

% Graficar el punto Ma respecto a O
plot_point(h2e(Punto_ao), 'label', 'Punto_Aa0', 'solid', 'ko');

% Graficar el vector desde el origen de M al punto Ma
quiver(7, 4, sqrt(2^2+1), 0, 0, 'k', 'LineWidth', 1.5);  % Vector en negro desde el origen de M

% Etiquetar el primer vector (en el marco M)
text(7 + 1, 4, '(2,1)', 'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'bottom');

% Graficar el vector desde el origen O al punto Ma
quiver(0, 0, Punto_ao(1), Punto_ao(2), 0, 'r', 'LineWidth', 1);  % Vector en rojo desde el origen O

% Etiquetar el segundo vector (en el marco O)
text(Punto_ao(1)-0.5, Punto_ao(2)-0.7, sprintf('(%.1f, %.1f)', Punto_ao(1), Punto_ao(2)), 'FontSize', 10, 'Color', 'r', 'VerticalAlignment', 'bottom');

% Mostrar los resultados en consola
fprintf("El angulo es: \r\n theta = %0.2f\r\nLa matriz de transformacion homogenea correspondiente es:\n", angulo*180/pi);
disp(Traslacion*Rotacion);
fprintf("El vector Ma respecto del sistema de coordenadas O es: \n");
disp(h2e(Punto_ao));

