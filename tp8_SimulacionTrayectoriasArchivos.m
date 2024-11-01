%% Cargar el modelo y configuración inicial
clc; clear; close all;
run("cargarModelo.m");
robot2;

articular = false;
q_tramo = [];
%% Mostrar el menú para seleccionar la trayectoria
opcion = menu('Seleccione la trayectoria a simular:', ...
              'Trayectoria 1', 'Trayectoria 2', 'Trayectoria 3','Trayectoria 4','Trayectoria 5', 'Trayectoria 6','Trayectoria 7');

% Asignar el nombre del archivo según la opción seleccionada
switch opcion
    case 1
        archivo_matrices = 'anexos/matrices_transformacion1.txt';
        q0 = [-0.0002   -0.0001   -3.2038    0.3774   -0.3140    3.1410];
    case 2
        archivo_matrices = 'anexos/matrices_transformacion4.txt';
        q0 = [ 0.0001   -0.0000   -3.1416    0.2511   -0.3770    3.1415];
    case 3
        archivo_matrices = 'anexos/matrices_transformacion3.txt';
        q0 = [2.9537   -0.0001   -0.0022    0.1270    0.3140    0.0023];
    case 4
        archivo_matrices = 'anexos/matrices_transformacion6.txt';
        q0 = [0.0000    0.0002   -0.0009   -0.3769    0.2514    0.0008];
    case 5
        archivo_matrices = 'anexos/matrices_transformacion7.txt';
        q0 = [0.0000    0.0002   -0.0009   -0.3769    0.2514    0.0008];
    case 6
        archivo_matrices = 'anexos/matrices_transformacion8.txt';
        q0 = [0.0000    0.0002   -0.0009   -0.3769    0.2514    0.0008];
    case 7
        archivo_matrices = 'anexos/matrices_transformacion10.txt';
        q0 = [0.000000 0.000000 0.486727 -0.752210 0.000000 0.000000];
        articular = true;
    otherwise
        error('Opción no válida.');
end

% Leer y procesar el archivo
fileID = fopen(archivo_matrices, 'r');
if fileID == -1
    error('No se pudo abrir el archivo.');
end

% Leer el archivo línea por línea y construir las matrices de 4x4
matrices = {};  % Store matrices
vectores_q = {};  % Store q vectors
aproximar_o_interpolar = [];  % Store approximation flags

while ~feof(fileID)
    primera_linea = fgetl(fileID);
    
    if contains(primera_linea, '# Vector q:')
        % Read q vector with approximation flag
        linea = fgetl(fileID);
        valores = sscanf(linea, '%f')';
        
        % Extract q vector (first 6 components) and approximation flag (7th component)
        q = valores(1:6);
        aproximar_o_interpolar = [aproximar_o_interpolar; valores(7)];
        vectores_q{end+1} = q;
        
        % Skip "# Matriz T:" line
        fgetl(fileID);
        
        % Read matrix T
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
        matrices{end+1} = matriz;
    else
        % Original logic for old format
        matriz = zeros(4, 4);
        matriz(1, :) = sscanf(primera_linea, '%f')';
        for i = 2:4
            linea = fgetl(fileID);
            if ischar(linea)
                valores = sscanf(linea, '%f');
                if length(valores) == 4
                    matriz(i, :) = valores';
                end
            end
        end
        matrices{end+1} = matriz;
        vectores_q{end+1} = [];
        aproximar_o_interpolar = [aproximar_o_interpolar; 0]; % Default to interpolation
    end
end

fclose(fileID);

%% Proceso de cálculo
% Inicializar la trayectoria interpolada completa
T_trayectoria_interpolada = [];
q_trayectoria_interpolada = [];
% Inicializar matriz para almacenar las coordenadas articulares

% Interpolar usando ctraj entre cada par de matrices consecutivas
n_interpolaciones = 20;
i = 1;
falta_aproximar = false;
while i < length(matrices)
    T_inicial = matrices{i};
    T_final = matrices{i + 1};
    
    % Interpolación entre T_inicial y T_final con ctraj
    T_interpolada = ctraj(T_inicial, T_final, n_interpolaciones);
    
    % Almacenar en la trayectoria completa
    T_trayectoria_interpolada = cat(3, T_trayectoria_interpolada, T_interpolada);

    if articular
        q_inicial = vectores_q{i};
        q_final = vectores_q{i+1};
        if aproximar_o_interpolar(i) == 0
            q_tramo = jtraj(q_inicial, q_final, n_interpolaciones);
        else
            % Collect consecutive points to approximate
            puntos_aproximar = q_inicial; % Primera columna
            j = i + 1;
            f = length(aproximar_o_interpolar);
            while j <= length(aproximar_o_interpolar) && aproximar_o_interpolar(j) == 1
                puntos_aproximar = [puntos_aproximar; vectores_q{j}]; % Añadir columna
                j = j + 1;
                falta_aproximar = true;
            end
            puntos_aproximar = [puntos_aproximar; vectores_q{j}];
            if falta_aproximar
                % Generate trajectory through all points
                q_tramo = mstraj(puntos_aproximar(2:end,:), ...  % Waypoints (sin incluir el primero)
                [], ...                                                % Velocidades máximas
                0.5*diag(eye(j-i)), ...                                           % tiempos entre puntos
                puntos_aproximar(1,:), ...                            % Punto inicial (q1)
                0.01, ...                                               % Intervalo de tiempo
                0.3);
                
                % Skip the points we just processed
                i = j - 1 
                falta_aproximar = false;
            end
            
        end
    end
    q_trayectoria_interpolada = cat(1, q_trayectoria_interpolada, q_tramo);
    i = i+1;
end



% Calcular la cinemática inversa para cada matriz interpolada
if articular
        q_trayectoria = q_trayectoria_interpolada;

else
    for i = 1:size(T_trayectoria_interpolada, 3)
        T_actual = T_trayectoria_interpolada(:, :, i);
        % q_trayectoria(i, :) = R.ikine(T_actual,'q0',q0);
        q_trayectoria(i, :) = cinInversa(R, T_actual, 1, dh, q0); % Cinemática inversa
        q0 = q_trayectoria(i, :);  % Actualizar q0 para la próxima iteración
    end
end

% Simulación de la trayectoria interpolada en el robot
R.plot(q_trayectoria, 'fps', 50, 'trail', {'r', 'LineWidth', 2});% Posición inicial
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