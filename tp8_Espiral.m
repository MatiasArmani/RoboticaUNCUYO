clc; clear; close all;

% Definir el robot (tu modelo de 6 grados de libertad ya definido)
d1 = 0.150; d6 = 0.150; d3 = 0.500; a4 = 0.500; dEspatula = 0.0; hBase = 0;

dh = [
    0   d1    0    pi/2   0;
    0    0    0   -pi/2   0;
    0   d3    0    pi/2   0;
    0    0   a4   -pi/2   0;
    0    0    0    pi/2   0;
    0   d6    0       0   0];

R = SerialLink(dh,'name','Robot Heladero','plotopt',{'scale',0.8,'jointdiam',1.4,'jointcolor',[0 0.3 0.5],'name','tilesize',0.2,'tile1color',[1 1 1],'noname','lightpos',[0 0 10]});
R.base = transl(0, 0, hBase);
R.tool = transl(0, 0, dEspatula);
R.qlim(1:6, :) = [-180 180; -180 180; -180 180; -180 180; -180 180; -180 180]*pi/180;

% Parámetros de la espiral
n_puntos = 75;  % Número de puntos en la trayectoria
radio = 0.6;  % Radio de la espiral
vueltas = 3;  % Número de vueltas
altura_inicial = 0.3;  % Altura inicial en z
altura_final = 0.7;  % Altura final en z
centro_x = 0;  % Centro de la espiral en el eje x
centro_y = 0;  % Centro de la espiral en el eje y

% Generar la trayectoria en espiral
theta = linspace(0, vueltas*2*pi, n_puntos);  % Ángulo para las vueltas
x = centro_x + radio * cos(theta);  % Coordenada x de la espiral
y = centro_y + radio * sin(theta);  % Coordenada y de la espiral
z = linspace(altura_inicial, altura_final, n_puntos);  % Coordenada z de la espiral

trayectoria_cartesiana = [x; y; z];  % Trajectory in Cartesian coordinates

% Inicializar matriz para guardar los ángulos articulares
q_trayectoria = zeros(n_puntos, 6);
q0 = [0,0,0,0,0,0];  % Ángulo inicial del robot

puntos_interpolacion = 6;

P1 = [x(1);y(1);z(1)];
q0 = cinInversa(R,transl(P1),1,dh,q0);
q_traj_total = zeros(n_puntos*puntos_interpolacion,6);
% Para cada punto en la trayectoria cartesiana, calcular la cinemática inversa
for i = 1:n_puntos
    pos_actual = trayectoria_cartesiana(:, i)';
    T_actual = transl(pos_actual);  % Matriz de transformación
    q_trayectoria(i, :) = cinInversa(R, T_actual, 1, dh, q0);  % Cinemática inversa para el punto
    q_traj = jtraj(q0,q_trayectoria(i,:),puntos_interpolacion);
    q_traj_total(puntos_interpolacion*(i-1)+1:puntos_interpolacion*(i-1)+puntos_interpolacion,:) = q_traj;
    q0 = q_trayectoria(i, :);  % Actualizar el ángulo inicial para la siguiente iteración
end

% Configuración del gráfico 3D para el robot
figure;
R.plot(q_trayectoria(1, :));  % Mostrar la posición inicial del robot
hold on;

% Ploteo de la trayectoria de la espiral en el espacio
plot3(x, y, z, 'r.', 'MarkerSize', 5);  % Puntos de la espiral en rojo

% Inicializar la curva roja
curve_x = [];
curve_y = [];
curve_z = [];

% Animación de la trayectoria y ploteo de la curva
dt=0.05;
for i = 1:size(q_traj_total, 1)
    R.plot(q_traj_total(i, :));  % Dibujar el robot en cada posición de la trayectoria
    
    % Obtener la posición del efector final
    T_efector = R.fkine(q_traj_total(i, :));  % Cinemática directa para obtener posición
    
    % Extraer las coordenadas x, y, z del efector final
    curve_x = [curve_x, T_efector.t(1)];
    curve_y = [curve_y, T_efector.t(2)];
    curve_z = [curve_z, T_efector.t(3)];
    
    % Graficar cada nuevo punto en la curva roja
    plot3(curve_x(end), curve_y(end), curve_z(end), 'b.', 'MarkerSize', 5);  % Ploteo de punto
    drawnow;  % Actualizar la gráfica en cada iteración
    
    pause(dt);  % Controlar la velocidad de la animación
end

% Ajustes finales del gráfico
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Simulación de la trayectoria en espiral del robot');
%% Grafico de posiciones articulares
figure
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