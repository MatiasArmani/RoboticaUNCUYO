% Cargar el modelo y configuración inicial
clc; clear; close all;
run("cargarModelo.m");
robot2;

% Mostrar el menú para seleccionar la trayectoria
opcion = menu('Seleccione la trayectoria a simular:', ...
              'Trayectoria 1', 'Trayectoria 2', 'Trayectoria 3','Trayectoria 4');

% Asignar el nombre del archivo según la opción seleccionada
switch opcion
    case 1
        archivo_matrices = 'matrices_transformacion1.txt';
        q0 = [-0.0002   -0.0001   -3.2038    0.3774   -0.3140    3.1410];
    case 2
        archivo_matrices = 'matrices_transformacion4.txt';
        q0 = [ 0.0001   -0.0000   -3.1416    0.2511   -0.3770    3.1415];
    case 3
        archivo_matrices = 'matrices_transformacion3.txt';
        q0 = [2.9537   -0.0001   -0.0022    0.1270    0.3140    0.0023];
    case 4
        archivo_matrices = 'matrices_transformacion6.txt';
        q0 = [0.0000    0.0002   -0.0009   -0.3769    0.2514    0.0008];
    otherwise
        error('Opción no válida.');
end

% Leer y procesar el archivo
fileID = fopen(archivo_matrices, 'r');
if fileID == -1
    error('No se pudo abrir el archivo.');
end

% Inicializar una lista para almacenar matrices de transformación
matrices = {};

% Leer el archivo línea por línea y construir las matrices de 4x4
while ~feof(fileID)
    matriz = zeros(4, 4);
    for i = 1:4
        linea = fgetl(fileID);
        if ischar(linea)
            valores = sscanf(linea, '%f');
            if length(valores) == 4
                matriz(i, :) = valores';
            end
        end
    end
    matrices{end+1} = matriz; % Agregar la matriz a la lista
end

fclose(fileID);

% Inicializar la trayectoria interpolada completa
T_trayectoria_interpolada = [];

% Interpolar usando ctraj entre cada par de matrices consecutivas
n_interpolaciones = 20;
for i = 1:length(matrices) - 1
    T_inicial = matrices{i};
    T_final = matrices{i + 1};
    
    % Interpolación entre T_inicial y T_final con ctraj
    T_interpolada = ctraj(T_inicial, T_final, n_interpolaciones);
    
    % Almacenar en la trayectoria completa
    T_trayectoria_interpolada = cat(3, T_trayectoria_interpolada, T_interpolada);
end

% Inicializar matriz para almacenar las coordenadas articulares
q_trayectoria = zeros(size(T_trayectoria_interpolada, 3), 6);

% Calcular la cinemática inversa para cada matriz interpolada
for i = 1:size(T_trayectoria_interpolada, 3)
    T_actual = T_trayectoria_interpolada(:, :, i);
    q_trayectoria(i, :) = R.ikine(T_actual,'q0',q0);
    % q_trayectoria(i, :) = cinInversa(R, T_actual, 1, dh, q0); % Cinemática inversa
    q0 = q_trayectoria(i, :);  % Actualizar q0 para la próxima iteración
end

% Simulación de la trayectoria interpolada en el robot
R.plot(q_trayectoria, 'fps', 60, 'trail', {'r', 'LineWidth', 2});% Posición inicial
hold on;

% Ajustes finales del gráfico
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Simulación de la trayectoria interpolada en el espacio 3D usando ctraj');

%% Grafico de posiciones articulares
q_traj_total = q_trayectoria;
figure
n = size(q_traj_total,1);
dt = 0.05;
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
