% Main script
clear; clc; close all;
run("cargarModelo.m");
robot2;

% Parámetros por tramo (ejemplo para 5 tramos)
params = struct([]);
params(1).dt = 0.01;
params(1).t_desac = 0.3;
params(1).t_paso = 0.5;
% ... definir para cada tramo

% Mostrar menú para seleccionar archivo
[filename, pathname] = uigetfile('*.txt', 'Seleccionar archivo de trayectoria');
if filename == 0
    return;
end

% Leer archivo
fileID = fopen(fullfile(pathname, filename), 'r');
if fileID == -1
    error('No se pudo abrir el archivo');
end

% Inicializar variables
matrices = {};
vectores_q = {};
aproximar_o_interpolar = [];
tramo_actual = 1;
contenido_tramo = {};
linea = fgetl(fileID);

% Leer archivo por tramos
while ischar(linea)
    if strcmp(strtrim(linea), '_____')
        % Procesar tramo anterior si existe
        if ~isempty(contenido_tramo)
            [matrices_tramo, vectores_tramo, aprox_tramo] = procesarTramo(contenido_tramo, params(tramo_actual));
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
    [matrices_tramo, vectores_tramo, aprox_tramo] = procesarTramo(contenido_tramo, params(tramo_actual));
    matrices = [matrices, matrices_tramo];
    vectores_q = [vectores_q, vectores_tramo];
    aproximar_o_interpolar = [aproximar_o_interpolar; aprox_tramo];
end

fclose(fileID);

% Simulación de la trayectoria interpolada en el robot
R.plot(vectores_q, 'fps', 120, 'trail', {'r', 'LineWidth', 2});% Posición inicial
hold on;

% Ajustes finales del gráfico
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Simulación de la trayectoria interpolada en el espacio 3D usando ctraj');

% Grafico de posiciones articulares
q_traj_total = q_trayectoria_interpolada;
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

% Función auxiliar para procesar cada tramo
function [matrices, vectores_q, aproximar] = procesarTramo(contenido, params)
    matrices = {};
    vectores_q = {};
    aproximar = [];
    
    % Determinar tipo de tramo
    primera_linea = contenido{1};
    if contains(primera_linea, '# Vector q:')
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
                    valores = sscanf(contenido{i+3+j}, '%f');
                    matriz(j,:) = valores';
                end
                matrices{end+1} = matriz;
                i = i + 7;
            else
                i = i + 1;
            end
        end
    else
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
            aproximar = [aproximar; 0];
        end
    end
end


