clc, close, clear

% Consigna A
a = -17/180 * pi;  % Ángulo en radianes
Rot1 = [cos(a) -sin(a) 0; 
        sin(a)  cos(a) 0; 
        0        0    1];  % Matriz de rotación para la primera consigna

v_M1 = [1; 0.5; 0];  % Vector en el sistema M
v_O1 = Rot1 * v_M1;  % Vector transformado al sistema O

soO = SO3;           % Sistema de coordenadas original
soM1 = SO3.Rz(-17/180 * pi);   % Rotación alrededor del eje Z en 17 grados

% Mostrar resultados de la primera consigna
disp('Consigna A');
disp('Matriz de rotación:');
Rot1
disp('Coordenadas del punto respecto al sistema O:');
v_O1

% Graficar resultados de la primera consigna
figure(1);
view(30,30);
soO.plot('frame','O');
hold on
soM1.plot('frame','M','color','red');
plot3(v_O1(1),v_O1(2),v_O1(3),'ko')
text(v_O1(1), v_O1(2), v_O1(3), sprintf('(%.2f, %.2f, %.2f)', v_O1), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','b');
text(v_O1(1), v_O1(2), v_O1(3)-0.15, sprintf('(%.2f, %.2f, %.2f)', v_M1), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','r');

title('Consigna A');
hold off

% Consigna B
soM2 = SO3.Rx(35/180 * pi);   % Rotación alrededor del eje X en 35 grados
Rot2 = soM2.R;       % Matriz de rotación para la segunda consigna

v_M2 = [0; 0; 1];    % Vector en el sistema M
v_O2 = Rot2 * v_M2;  % Vector transformado al sistema O

% Mostrar resultados de la segunda consigna
disp('Consigna B');
disp('Matriz de rotación:');
Rot2
disp('Coordenadas del punto respecto al sistema O:');
v_O2

% Graficar resultados de la segunda consigna
figure(2);
view(30,30);
soO.plot('frame','O');
hold on
soM2.plot('frame','M','color','red');
plot3(v_O2(1),v_O2(2),v_O2(3),'ko');
text(v_O2(1), v_O2(2), v_O2(3), sprintf('(%.2f, %.2f, %.2f)', v_O2), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','b');
text(v_O2(1), v_O2(2), v_O2(3)-0.15, sprintf('(%.2f, %.2f, %.2f)', v_M2), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','r');
title('Consigna B');
hold off

% Consigna C
soM3 = SO3.Ry(90/180 * pi);   % Rotación alrededor del eje X en 35 grados (igual que en consigna B)
Rot3 = soM3.R;       % Matriz de rotación para la tercera consigna

v_M3 = [1; 0.5; 0.3];    % Vector en el sistema M
v_O3 = Rot3 * v_M3;  % Vector transformado al sistema O

% Mostrar resultados de la tercera consigna
disp('Consigna C');
disp('Matriz de rotación:');
Rot3
disp('Coordenadas del punto respecto al sistema O:');
v_O3

% Graficar resultados de la tercera consigna
figure(3);
view (30,30)
soO.plot('frame','O');
hold on
soM3.plot('frame','M','color','red');
plot3(v_O3(1),v_O3(2),v_O3(3),'ko');
text(v_O3(1), v_O3(2), v_O3(3), sprintf('(%.2f, %.2f, %.2f)', v_O3), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','b');
text(v_O3(1), v_O3(2), v_O3(3)-0.15, sprintf('(%.2f, %.2f, %.2f)', v_M3), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','Color','r');
title('Consigna C');
hold off