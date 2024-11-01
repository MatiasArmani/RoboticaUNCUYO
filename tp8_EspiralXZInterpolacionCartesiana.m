clc; clear; close all;
%run("cargarModelo.m");
% Definir el robot (modelo de 6 grados de libertad)
d1 = 0.150; d6 = 0.150; d3 = 0.500; a4 = 0.500; dEspatula = 0.0; hBase = 0;

dh = [
    0   d1    0    pi/2   0;
    0    0    0   -pi/2   0;
    0   d3    0    pi/2   0;
    0    0   a4   -pi/2   0;
    0    0    0    pi/2   0;
    0   d6    0       0   0];

R = SerialLink(dh, 'name', 'Robot Heladero', 'plotopt', {'scale', 0.8, 'jointdiam', 1.4, 'jointcolor', [0 0.3 0.5], 'name', 'tilesize', 0.2, 'tile1color', [1 1 1], 'noname', 'lightpos', [0 0 10]});
R.base = transl(0, 0, hBase);
R.tool = transl(0, 0, dEspatula);
R.qlim(1:6, :) = [-180 180; -180 180; -180 180; -180 180; -180 180; -180 180]*pi/180;

% Parámetros de la espiral en el plano XZ
n_puntos = 50;  % Número de puntos principales en la espiral
radio = 0.225;  % Radio de la espiral
vueltas = 3;  % Número de vueltas
altura_inicial = 0.25;  % Altura inicial en Y
altura_final = 0.7;  % Altura final en Y
centro_x = 0.45;  % Centro de la espiral en X
centro_z = 0.3;  % Centro de la espiral en Z

% Generar los puntos principales de la espiral
theta = linspace(0, vueltas * 2 * pi, n_puntos);  % Ángulo para las vueltas
x = centro_x + radio * cos(theta);  % Coordenada X de la espiral
z = centro_z + radio * sin(theta);  % Coordenada Z de la espiral
y = linspace(altura_inicial, altura_final, n_puntos);  % Coordenada Y (altura)

% Almacenar los puntos principales en la trayectoria
trayectoria_cartesiana = [x' y' z'];

% Interpolación continua entre puntos usando mstraj
% Parámetros de interpolación
velocidad = 0.5;  % Velocidad constante
t_intervalo = 0.1;  % Intervalo de tiempo
q0 = [0, 0, 0, 0, 0, 0];  % Ángulo inicial del robot

% Generar la trayectoria interpolada en el espacio cartesiano
trayectoria_interpolada = mstraj(trayectoria_cartesiana, repmat(velocidad, 1, 3), [], trayectoria_cartesiana(1, :), t_intervalo, 0.1);

% Inicializar matriz para almacenar las coordenadas articulares
q_trayectoria = zeros(size(trayectoria_interpolada, 1), 6);

% Calcular la cinemática inversa para cada punto interpolado
for i = 1:size(trayectoria_interpolada, 1)
    pos_actual = trayectoria_interpolada(i, :);
    T_actual = transl(pos_actual);
    q_trayectoria(i, :) = cinInversa(R, T_actual, 1, dh, q0);
    q0 = q_trayectoria(i, :);
end

% Configuración del gráfico 3D para el robot
figure
R.plot(q_trayectoria(1, :));
hold on;

% Ploteo de los puntos principales de la espiral en el espacio
plot3(x, y, z, 'ro', 'MarkerSize', 2, 'MarkerFaceColor', 'r');

% Inicializar las matrices para la curva de la trayectoria interpolada
curve_x = [];
curve_y = [];
curve_z = [];

% Animación de la trayectoria y ploteo de la curva interpolada
dt = 0.01;
for i = 1:size(q_trayectoria, 1)
    R.plot(q_trayectoria(i, :));

    % Obtener la posición del efector final
    T_efector = R.fkine(q_trayectoria(i, :));

    % Almacenar las coordenadas del efector
    curve_x = [curve_x, T_efector.t(1)];
    curve_y = [curve_y, T_efector.t(2)];
    curve_z = [curve_z, T_efector.t(3)];

    % Graficar cada nuevo punto en la curva
    plot3(curve_x(end), curve_y(end), curve_z(end), 'b.', 'MarkerSize', 5);
    drawnow;
    pause(dt);
end

% Ajustes finales del gráfico
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Simulación de la trayectoria en espiral interpolada en el plano XZ (sin detenciones en cada punto)');

%% Grafico de posiciones articulares
figure
q_traj_total = q_trayectoria;
n = size(q_traj_total,1);
t = 0:dt:(n-1)*dt;
grid on
hold on
colores = ['b','g','c','m','k','y'];
for i=1:6
plot(t,q_traj_total(:,i),colores(i))
end
legend('q1','q2','q3','q4','q5','q6');
xlabel('Tiempo (s)');
ylabel('Angulo articular (rad)');
title('Posiciones articulares');
%% Grafico de velocidades articulares
figure
grid on
hold on
dq = DerivacionNumerica(q_traj_total,t);
for i=1:6
plot(t,dq(:,i),colores(i))
end
legend('dq1','dq2','dq3','dq4','dq5','dq6');
xlabel('Tiempo (s)');
ylabel('Velocidad articular (rad/s)');
title('Velocidades de coordenadas articulares');
%% Grafico de aceleraciones articulares
ddq = DerivacionNumerica(dq,t);
figure
grid on
hold on
for i=1:6
plot(t,ddq(:,i),colores(i))
end
legend('ddq1','ddq2','ddq3','ddq4','ddq5','ddq6');
xlabel('Tiempo (s)');
ylabel('Aceleracion articular (rad/s^2)');
title('Aceleraciones de coordenadas articulares');
