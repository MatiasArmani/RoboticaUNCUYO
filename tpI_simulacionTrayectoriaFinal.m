% Main script
clear; clc; close all;
robot2;

% Parámetros por tramo (ejemplo para 5 tramos)
params = struct([]);
params(1).dt = 0.01;
params(1).t_desac = 0.2;
params(1).t_paso = 0.3;
params(2).dt = 0.01;
params(2).t_desac = 0.3;
params(2).t_paso = 0.5;
params(3).dt = 0.01;
params(3).t_desac = 0.2;
params(3).t_paso = 0.5;
params(4).dt = 0.01;
params(4).t_desac = 0.3;
params(4).t_paso = 0.5;
params(5).dt = 0.01;
params(5).t_desac = 0.2;
params(5).t_paso = 0.8;


% Leer archivo
fileID = fopen('./anexos/trayectoria_final_completa.txt', 'r');
if fileID == -1
    error('No se pudo abrir el archivo');
end

% Inicializar variables
matrices = {};
vectores_q = {};
q_interpolado = [];
aproximar_o_interpolar = [];
tramo_actual = 1;
contenido_tramo = {};
linea = fgetl(fileID);

% Leer archivo por tramos
while ischar(linea)
    if strcmp(strtrim(linea), '_____')
        % Procesar tramo anterior si existe
        if ~isempty(contenido_tramo)
            [matrices_tramo, vectores_tramo, aprox_tramo,q_interpolado] = procesarTramo(R,dh,contenido_tramo, q_interpolado, params(tramo_actual),100);
            matrices = [matrices, matrices_tramo];
            vectores_q = [vectores_q, vectores_tramo];
            aproximar_o_interpolar = [aproximar_o_interpolar; aprox_tramo];
            tramo_actual = tramo_actual + 1;
            contenido_tramo = {};
        end
    else
        contenido_tramo{end+1} = linea;
    end
    linea = fgetl(fileID);
end

% Procesar último tramo
if ~isempty(contenido_tramo)
    [matrices_tramo, vectores_tramo, aprox_tramo,q_interpolado] = procesarTramo(R,dh,contenido_tramo, q_interpolado, params(tramo_actual),100);
    matrices = [matrices, matrices_tramo];
    vectores_q = [vectores_q, vectores_tramo];
    aproximar_o_interpolar = [aproximar_o_interpolar; aprox_tramo];
end

fclose(fileID);

% Crear figura y botones
run("cargarModelo.m");
btn = uicontrol('Style', 'pushbutton', 'String', 'Pausar', ...
    'Position', [20 20 100 40], 'Callback', 'uiwait(gcf)');

% Ploteo inicial del robot
R.plot(q_interpolado(1, :), 'trail', {'r', 'LineWidth', 2});
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Simulación de la trayectoria interpolada en el espacio 3D usando ctraj');

hold on;
% Esperar a que se toque el botón de iniciar
btn.String = 'Iniciar';
btn.Callback = @(src, event) animarTrayectoria(R,q_interpolado, src, event);
uiwait(fig);


% Grafico de posiciones articulares
q_traj_total = q_interpolado;
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
% Grafico de velocidades articulares
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
% Grafico de aceleraciones articulares
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


% Callback para animar la trayectoria
function animarTrayectoria(R,q,~, ~)
R.plot(q, 'fps', 120, 'trail', {'r', 'LineWidth', 2});
end

% Función auxiliar para procesar cada tramo
function [matrices, vectores_q, aproximar,q_interpolado] = procesarTramo(R, dh, contenido, q_interpolado, params, n_interpolaciones)
matrices = {};
vectores_q = {};
aproximar = [];

% Determinar tipo de tramo
primera_linea = contenido{1};
if contains(primera_linea, '# Vector q:')
    articular = true;
    % Procesar formato con vectores q
    i = 1;
    while i <= length(contenido)
        if contains(contenido{i}, '# Vector q:')
            % Leer vector q
            valores = sscanf(contenido{i+1}, '%f')';
            q = valores(1:6);
            aproximar = [aproximar; valores(7)];
            vectores_q{end+1} = q;

            % Saltar línea de "# Matriz T:"
            matriz = zeros(4, 4);
            for j = 1:4
                valores = sscanf(contenido{i+2+j}, '%f');
                matriz(j,:) = valores';
            end
            matrices{end+1} = matriz;
            i = i + 7;
        else
            i = i + 1;
        end
    end
else
    articular = false;
    % Procesar formato solo matrices
    for i = 1:4:length(contenido)
        matriz = zeros(4, 4);
        for j = 1:4
            if i+j-1 <= length(contenido)
                valores = sscanf(contenido{i+j-1}, '%f');
                matriz(j,:) = valores';
            end
        end
        matrices{end+1} = matriz;
        vectores_q{end+1} = [];
    end
end

i = 1;
falta_aproximar = false;
while i < length(matrices)
    if articular
        q_inicial = vectores_q{i};
        q_final = vectores_q{i+1};
        if aproximar(i) == 0
            q_tramo = jtraj(q_inicial, q_final, n_interpolaciones);
        else
            % Collect consecutive points to approximate
            puntos_aproximar = q_inicial; % Primera columna
            j = i+1;
            while j <= length(aproximar) && aproximar(j) == 1
                puntos_aproximar = [puntos_aproximar; vectores_q{j}]; % Añadir columna
                j = j + 1;
                falta_aproximar = true;
            end
            if falta_aproximar
                % Generate trajectory through all points
                q_tramo = mstraj(puntos_aproximar(2:end,:), ...  % Waypoints (sin incluir el primero)
                    [], ...                                                % Velocidades máximas
                    params.t_paso*diag(eye(j-i-1)), ...                                           % tiempos entre puntos
                    puntos_aproximar(1,:), ...                            % Punto inicial (q1)
                    params.dt, ...                                               % Intervalo de tiempo
                    params.t_desac);

                % Skip the points we just processed
                i = j-1;
                falta_aproximar = false;
            end
        end
        q_interpolado = cat(1, q_interpolado, q_tramo);
    else
        T_inicial = matrices{i};
        T_final = matrices{i + 1};

        % Interpolación entre T_inicial y T_final con ctraj
        T_interpolada = ctraj(T_inicial, T_final, n_interpolaciones);

        for k = 1:n_interpolaciones
            q0 = q_interpolado(length(q_interpolado(:,1)), :);
            q_interpolado = cat(1, q_interpolado, cinInversa(R, T_interpolada(:, :, k), 1, dh, q0));
        end

    end
    i = i+1;
end

end
